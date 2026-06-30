"""Shared target-mission state machine core (async, non-blocking)."""

from __future__ import annotations

import asyncio
import sys
import time
from pathlib import Path
from typing import Any, Callable, Literal, Protocol, runtime_checkable

_SRC = Path(__file__).resolve().parents[2] / "src"
_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from spray_config import (  # noqa: E402
    GpsSurveyedSafetyParams,
    ObstacleSafetyParams,
    PointSprayParams,
)

from config import RPP_STALE
from gps_safety import (
    GPS_SAFETY_NA,
    RESUME_POLICY_AUTO,
    RUNTIME_POLICY_FAIL,
    GpsSafetyVerdict,
    evaluate_gps_surveyed_safety,
    local_ned_gps_status,
)
from logging_setup import get_logger
from mission_placement import GPS_SURVEYED, LOCAL_NED
from spray_safety import force_spray_off_confirmed

from target_mission.leg_geometry import build_target_leg
from target_mission.types import (
    OBSTACLE_BLOCKED,
    OBSTACLE_MISSING,
    OBSTACLE_NOT_CONFIGURED,
    OBSTACLE_OK,
    OBSTACLE_STALE,
    PAUSED_STATES,
    TERMINAL_POINT_STATES,
    SprayRuntimeSchemaError,
    TargetExecutionMode,
    TargetMissionRun,
    TargetMissionRunFailure,
    TargetMissionState,
    _DWELL_POLL_REQUIRED_FIELDS,
    _PARENT_ABORT_REASONS,
    _SKIP_ACCEPTED_STATES,
    _TERMINAL_REASON_PRIORITY,
)

log = get_logger("server.target_mission.core")


@runtime_checkable
class TargetPoint(Protocol):
    north_m: float
    east_m: float
    dwell_s: float | None
    mark: bool
    source_index: int


class TargetMissionCoreMixin:
    """Shared per-target execution loop for point and future target missions."""

    _DRAIN_TIMEOUT_S = 6.0

    def _emit_lifecycle_event(
        self,
        run: TargetMissionRun | None,
        kind: Literal[
            "leg_started",
            "arrived",
            "dwell_started",
            "marked",
            "paused",
            "resumed",
            "skipped",
        ],
        **kwargs: Any,
    ) -> None:
        raise NotImplementedError

    def _on_waiting_for_continue(
        self,
        run: TargetMissionRun | None,
        *,
        point_index: int,
        source_index: int,
    ) -> None:
        """Optional hook for manual-continue waits (point missions emit events)."""
        return None

    def _emit_navigation_only_completion(
        self,
        run: TargetMissionRun | None,
        *,
        point_index: int,
        source_index: int,
    ) -> None:
        """Hook: per-target completion for a finished navigate-only (mark=false) leg.

        Marked legs already emit their completion event when the dwell finishes
        (the ``"marked"`` lifecycle event), and the final target's completion is
        carried by the terminal event from ``terminal_cleanup``. This hook covers
        the remaining gap: a non-terminal ``mark=false`` target that finished
        arrival + settle but would otherwise emit no completion event. Point
        missions leave this as a no-op (their protocol differs); the verified
        adapter overrides it to emit a single non-terminal ``target_completed``.
        """
        return None

    def _build_target_leg(
        self,
        state: dict[str, Any],
        point: TargetPoint,
        params: PointSprayParams,
    ) -> tuple[list[tuple[float, float]], dict[str, Any]]:
        return build_target_leg(state, point.north_m, point.east_m, params)


    def set_logger(self, cb: Callable[[str, str], None]) -> None:
        self._log_cb = cb

    def _record(self, level: str, message: str) -> None:
        if self._log_cb is not None:
            self._log_cb(level, message)
        getattr(log, level if level in ("info", "warning", "error", "debug") else "info")(message)

    def is_paused(self) -> bool:
        return self._status.state in PAUSED_STATES

    def _spray_runtime_fingerprint(self, status: dict[str, Any]) -> tuple[int, int, float]:
        return (
            int(status.get("configuration_revision", -1)),
            int(status.get("model_revision", -1)),
            float(status.get("timestamp_monotonic_s", 0.0)),
        )

    def _validate_dwell_poll_status(self, status: dict[str, Any]) -> None:
        for field in _DWELL_POLL_REQUIRED_FIELDS:
            if field not in status:
                raise SprayRuntimeSchemaError(
                    f"spray runtime status missing required field {field!r}"
                )
        for field in (
            "commanded_on",
            "confirmed_off",
            "off_acknowledged",
            "active_dwell",
            "status_stale",
        ):
            if not isinstance(status[field], bool):
                raise SprayRuntimeSchemaError(
                    f"spray runtime status field {field!r} must be bool"
                )
        for field in ("dwell_command_id", "dwell_point_index"):
            if status[field] is not None and not isinstance(status[field], int):
                raise SprayRuntimeSchemaError(
                    f"spray runtime status field {field!r} must be int or null"
                )
        mission_id = status.get("dwell_mission_id")
        if mission_id is not None and not isinstance(mission_id, str):
            raise SprayRuntimeSchemaError(
                "spray runtime status field dwell_mission_id must be str or null"
            )
        if status.get("active_dwell") and (
            not isinstance(mission_id, str) or not mission_id
        ):
            raise SprayRuntimeSchemaError(
                "active dwell requires non-empty dwell_mission_id"
            )

    def _bind_dwell_identity(
        self,
        run: TargetMissionRun,
        *,
        command_id: int,
        command_revision: int,
        point_index: int,
        source_index: int,
    ) -> None:
        parent_id = run.parent_mission_id or run.mission_id
        config_revision = self._config.revision if self._config is not None else 0
        run.active_dwell_command_id = command_id
        run.active_dwell_command_revision = command_revision
        run.active_dwell_configuration_revision = config_revision
        run.active_dwell_point_index = point_index
        run.active_dwell_source_index = source_index
        run.dwell_revision_invalid = False
        self._write(
            run,
            parent_mission_id=parent_id,
            point_mission_generation=run.generation,
            active_dwell_command_id=command_id,
            active_dwell_command_revision=command_revision,
            active_dwell_configuration_revision=config_revision,
            active_dwell_point_index=point_index,
            active_dwell_source_index=source_index,
        )

    def _invalidate_dwell_identity(self, run: TargetMissionRun | None) -> None:
        if run is None:
            return
        run.dwell_revision_invalid = True
        run.active_dwell_command_id = None
        run.active_dwell_command_revision = None
        run.active_dwell_configuration_revision = None
        run.active_dwell_point_index = None
        run.active_dwell_source_index = None
        self._write(
            run,
            dwell_ownership_invalidated=True,
            active_dwell_command_id=None,
            active_dwell_command_revision=None,
            active_dwell_configuration_revision=None,
            active_dwell_point_index=None,
            active_dwell_source_index=None,
        )

    def _dwell_identity_matches(
        self,
        run: TargetMissionRun,
        status: dict[str, Any],
        offboard_ctrl,
    ) -> bool:
        if run.dwell_revision_invalid:
            return False
        if not self._is_current(run):
            return False
        if run.generation != self._generation:
            return False
        if offboard_ctrl is not None and getattr(
            offboard_ctrl, "running_mission_id", None
        ) not in {None, run.parent_mission_id, run.mission_id}:
            return False
        expected_id = run.active_dwell_command_id
        if expected_id is None:
            return False
        seen_id = status.get("dwell_command_id")
        if seen_id is None or int(seen_id) != expected_id:
            return False
        mission_id = status.get("dwell_mission_id")
        if not isinstance(mission_id, str) or not mission_id:
            return False
        if mission_id not in {run.mission_id, run.parent_mission_id}:
            return False
        if int(status.get("dwell_point_index", -1)) != int(
            run.active_dwell_point_index if run.active_dwell_point_index is not None else -1
        ):
            return False
        config_revision = run.active_dwell_configuration_revision
        if config_revision is not None and int(
            status.get("configuration_revision", -1)
        ) != config_revision:
            return False
        return True

    async def _cancel_dwell_service(self, ros_node) -> dict[str, Any]:
        if ros_node is None or not hasattr(ros_node, "cancel_spray_dwell_async"):
            return {"success": False, "message": "dwell cancel unavailable"}
        try:
            ok, message = await asyncio.wait_for(
                ros_node.cancel_spray_dwell_async(),
                timeout=self._DRAIN_TIMEOUT_S,
            )
            return {"success": bool(ok), "message": message or ""}
        except asyncio.TimeoutError:
            return {"success": False, "message": "dwell cancel timed out", "timeout": True}
        except Exception as exc:
            return {"success": False, "message": str(exc)}

    async def _force_spray_off_with_result(
        self,
        ros_node,
        *,
        check_cancel=None,
        require_confirm: bool = True,
        timeout_s: float | None = None,
    ) -> dict[str, Any]:
        result = await force_spray_off_confirmed(
            ros_node,
            timeout_s=self._DRAIN_TIMEOUT_S if timeout_s is None else timeout_s,
            check_cancel=check_cancel,
        )
        payload = result.as_dict()
        if require_confirm and not result.success:
            payload["recovery_required"] = bool(
                result.recovery_required or (result.attempted and result.live)
            )
        return payload

    async def _parent_abort_terminal(
        self,
        offboard_ctrl,
        run: TargetMissionRun | None,
        *,
        reason: str,
    ) -> dict[str, Any] | None:
        if offboard_ctrl is None or not hasattr(offboard_ctrl, "abort_async"):
            return None
        if reason not in _PARENT_ABORT_REASONS:
            return None
        try:
            result = await offboard_ctrl.abort_async()
            if run is not None and self._is_current(run):
                recovery = not bool(result.get("success", False))
                spray_off = result.get("spray_off_result") or {}
                if spray_off.get("recovery_required"):
                    recovery = True
                self._write(
                    run,
                    recovery_required=recovery,
                    terminal_safety_ok=not recovery,
                    terminal_failure_reason=(
                        "; ".join(result.get("errors") or [])
                        or result.get("message", "")
                    ),
                    spray_off_result=spray_off or None,
                )
            return result
        except Exception as exc:
            self._record("error", f"parent abort terminal failed: {exc}")
            if run is not None and self._is_current(run):
                self._write(
                    run,
                    recovery_required=True,
                    terminal_safety_ok=False,
                    terminal_failure_reason=str(exc),
                )
            return {"success": False, "message": str(exc)}

    def set_obstacle_clear(self, clear: bool) -> None:
        self._obstacle_clear = bool(clear)
        self._obstacle_last_recv = time.monotonic()
        self._status.obstacle_clear = self._obstacle_clear
        blocked, state, age_ms = self._obstacle_gate()
        self._status.obstacle_integration_enabled = self._obstacle_params().enabled
        self._status.obstacle_signal_state = state
        self._status.obstacle_signal_age_ms = age_ms

    def _obstacle_params(self) -> ObstacleSafetyParams:
        if self._config is not None:
            return self._config.obstacle
        return ObstacleSafetyParams()

    def _obstacle_signal_age_s(self) -> float | None:
        if self._obstacle_last_recv is None:
            return None
        return max(0.0, time.monotonic() - self._obstacle_last_recv)

    def _obstacle_gate(self) -> tuple[bool, str, float | None]:
        """Evaluate the obstacle hook.

        Returns ``(should_pause, signal_state, age_ms)``. When the integration
        is disabled the hook is ``not_configured`` and never pauses (and never
        silently reports clear). When enabled, a missing or stale signal is
        fail-closed (pause), as is an explicit blocked report.
        """
        params = self._obstacle_params()
        age_s = self._obstacle_signal_age_s()
        age_ms = age_s * 1000.0 if age_s is not None else None
        if not params.enabled:
            return False, OBSTACLE_NOT_CONFIGURED, age_ms
        if self._obstacle_last_recv is None:
            return True, OBSTACLE_MISSING, None
        if age_s is not None and age_s > params.signal_max_age_s:
            return True, OBSTACLE_STALE, age_ms
        if not self._obstacle_clear:
            return True, OBSTACLE_BLOCKED, age_ms
        return False, OBSTACLE_OK, age_ms

    def _write_obstacle_status(self, run: TargetMissionRun | None) -> tuple[bool, str]:
        blocked, state, age_ms = self._obstacle_gate()
        self._write(
            run,
            obstacle_clear=self._obstacle_clear,
            obstacle_integration_enabled=self._obstacle_params().enabled,
            obstacle_signal_state=state,
            obstacle_signal_age_ms=age_ms,
        )
        return blocked, state

    def _gps_safety_params(self) -> GpsSurveyedSafetyParams:
        if self._config is not None:
            return self._config.gps_safety
        return GpsSurveyedSafetyParams()

    def _gps_applies(self) -> bool:
        return self._source_frame == GPS_SURVEYED

    def _evaluate_gps_safety(
        self,
        state: dict[str, Any],
        *,
        recovery_since: float | None = None,
        paused: bool = False,
    ) -> GpsSafetyVerdict:
        if not self._gps_applies():
            return GpsSafetyVerdict(ok=True, gps_safety_state=GPS_SAFETY_NA)
        params = self._gps_safety_params()
        coords = [(p.north_m, p.east_m) for p in self._points]
        return evaluate_gps_surveyed_safety(
            state,
            self._origin_gps,
            coords,
            params,
            recovery_since=recovery_since,
            fault_count=self._gps_fault_count,
            last_fault_time_s=self._gps_last_fault_time,
            paused=paused,
        )

    def _write_gps_verdict(self, run: TargetMissionRun | None, verdict: GpsSafetyVerdict) -> None:
        if not self._gps_applies():
            self._write(run, **local_ned_gps_status())
            return
        self._write(run, **verdict.as_status_dict())

    def _point_params(self) -> PointSprayParams:
        if self._config is not None:
            return self._config.point
        return PointSprayParams()

    def _write_leg_diagnostics(
        self, run: TargetMissionRun | None, diag: dict[str, Any]
    ) -> None:
        self._write(run, **diag)

    async def _handle_hold_drift(
        self,
        run: TargetMissionRun,
        ros_node,
        hold_owner,
        point: TargetPoint,
        params: PointSprayParams,
        phase: str,
        *,
        error_m: float,
    ) -> str | None:
        """Cancel dwell/spray and pause or fail when hold drift exceeds tolerance."""
        self._record(
            "warning",
            f"hold drift {error_m:.3f} m > {params.hold_drift_tolerance_m:.3f} m during {phase}",
        )
        self._write(
            run,
            dwell_cancelled=phase == TargetMissionState.DWELLING.value
            or bool(self._status.active_dwell),
            active_dwell=False,
            dwell_remaining_s=0.0,
            active_dwell_command_id=None,
            last_failure_reason=(
                f"hold drift {error_m:.3f} m exceeded tolerance "
                f"{params.hold_drift_tolerance_m:.3f} m"
            ),
        )
        was_spraying = (
            phase == TargetMissionState.DWELLING.value or bool(self._status.active_dwell)
        )
        await self._confirm_spray_off(run, ros_node, require_confirm=was_spraying)
        if params.hold_drift_policy == "pause":
            await self._pause_cycle(
                run,
                ros_node,
                hold_owner,
                point,
                phase,
                pause_reason="operator",
            )
            return self._resume_phase_after_pause(phase)
        raise RuntimeError(self._status.last_failure_reason)

    async def _poll_hold_drift(
        self,
        run: TargetMissionRun,
        ros_node,
        hold_owner,
        point: TargetPoint,
        params: PointSprayParams,
        phase: str,
    ) -> str | None:
        if hold_owner is None or not hold_owner.active:
            return None
        hold_owner.refresh(ros_node)
        self._merge_hold_status(run, hold_owner, ros_node)
        error_m = hold_owner.hold_error_m(ros_node)
        if error_m is None:
            return None
        if error_m > params.hold_drift_tolerance_m:
            return await self._handle_hold_drift(
                run,
                ros_node,
                hold_owner,
                point,
                params,
                phase,
                error_m=error_m,
            )
        return None

    def _merge_hold_status(self, run: TargetMissionRun | None, hold_owner, ros_node) -> None:
        if hold_owner is None:
            return
        hold = hold_owner.as_dict(ros_node)
        self._write(
            run,
            setpoint_source=hold["setpoint_source"],
            hold_active=hold["hold_active"],
            hold_north_m=hold["hold_north_m"],
            hold_east_m=hold["hold_east_m"],
            hold_heading_ned_rad=hold["hold_heading_ned_rad"],
            hold_error_m=hold["hold_error_m"],
        )

    def _is_current(self, run: TargetMissionRun) -> bool:
        return self._run_token is run and self._generation == run.generation

    def _operation_coordinator(self):
        try:
            from main import operation_coordinator

            return operation_coordinator
        except Exception:
            return None

    def _check_operation_generation(self, run: TargetMissionRun) -> None:
        coordinator = self._operation_coordinator()
        if coordinator is None:
            return
        current = coordinator.current_generation()
        if run.operation_generation == 0:
            run.operation_generation = current
            return
        if run.operation_generation != current:
            raise asyncio.CancelledError()

    def _is_terminal_state(self) -> bool:
        return self._status.state in TERMINAL_POINT_STATES

    def _terminal_reject(self) -> tuple[bool, str, int] | None:
        if self._is_terminal_state():
            return False, f"point mission is terminal: {self._status.state.value}", 409
        return None

    def _terminal_reason_priority(self, reason: str) -> int:
        return _TERMINAL_REASON_PRIORITY.get(reason, 0)

    def _write(self, run: TargetMissionRun | None, **changes: Any) -> None:
        if run is not None and not self._is_current(run):
            return
        for key, value in changes.items():
            setattr(self._status, key, value)

    def _distance_to_point(self, state: dict[str, Any], point: TargetPoint) -> float:
        return (
            (float(state.get("pos_n", 0.0)) - point.north_m) ** 2
            + (float(state.get("pos_e", 0.0)) - point.east_m) ** 2
        ) ** 0.5

    def _update_live_diagnostics(
        self,
        run: TargetMissionRun,
        ros_node,
        point: TargetPoint,
        params: PointSprayParams,
        *,
        arrival_met: bool | None = None,
        settle_met: bool | None = None,
    ) -> None:
        state = ros_node.get_state()
        changes: dict[str, Any] = {
            "target_north_m": point.north_m,
            "target_east_m": point.east_m,
            "current_distance_m": self._distance_to_point(state, point),
            "mark_enabled": point.mark,
        }
        if arrival_met is not None:
            changes["arrival_met"] = arrival_met
        if settle_met is not None:
            changes["settle_met"] = settle_met
        elif arrival_met is None:
            changes["arrival_met"] = self._arrival_conditions_met(state, point, params)
        self._write(run, **changes)

    def _resume_phase_after_pause(self, phase: str) -> str:
        if phase == TargetMissionState.WAITING_FOR_CONTINUE.value:
            return "waiting_for_continue"
        return "navigating"

    async def _gps_fail_cycle(
        self,
        run: TargetMissionRun,
        ros_node,
        hold_owner,
        point: TargetPoint,
        phase: str,
        verdict: GpsSafetyVerdict,
    ) -> None:
        during_dwell = phase == TargetMissionState.DWELLING.value
        dwell_cancelled = during_dwell or bool(self._status.active_dwell)
        self._write(
            run,
            state=TargetMissionState.FAILED_GPS_SAFETY,
            last_transition=f"gps_fail:{phase}",
            dwell_cancelled=dwell_cancelled,
            active_dwell=False,
            dwell_remaining_s=0.0,
            active_dwell_command_id=None,
            pre_pause_state=phase,
            paused_point_index=self._status.current_point_index,
            pause_reason="gps_safety",
            waiting_for_continue=False,
            last_error=verdict.reason,
            last_failure_reason=verdict.reason,
            resume_available=False,
            ready=False,
            run_active=False,
        )
        self._write_gps_verdict(run, verdict)
        self._record(
            "error",
            f"point mission GPS-safety FAIL during {phase} at point "
            f"{self._status.current_point_index}: {verdict.reason}",
        )
        await self._confirm_spray_off(run, ros_node, require_confirm=dwell_cancelled)
        state = ros_node.get_state()
        north = float(state.get("pos_n", 0.0))
        east = float(state.get("pos_e", 0.0))
        heading = state.get("heading_ned_rad")
        if heading is not None:
            heading = float(heading)
        if hold_owner is not None:
            hold_owner.activate(
                ros_node,
                north_m=north,
                east_m=east,
                heading_ned_rad=heading,
                reason="gps_safety",
            )
            self._merge_hold_status(run, hold_owner, ros_node)

    async def _pause_cycle(
        self,
        run: TargetMissionRun,
        ros_node,
        hold_owner,
        point: TargetPoint,
        phase: str,
        *,
        pause_reason: str,
    ) -> None:
        during_dwell = phase == TargetMissionState.DWELLING.value
        dwell_cancelled = during_dwell or bool(self._status.active_dwell)
        transient = TargetMissionState.PAUSING
        if pause_reason == "obstacle" and during_dwell:
            transient = TargetMissionState.OBSTACLE_DURING_DWELL
        elif pause_reason == "gps_safety" and during_dwell:
            transient = TargetMissionState.GPS_DURING_DWELL
        self._write(
            run,
            state=transient,
            last_transition=f"pausing:{phase}",
            dwell_cancelled=dwell_cancelled,
            active_dwell=False,
            dwell_remaining_s=0.0,
            active_dwell_command_id=None,
            pre_pause_state=phase,
            paused_point_index=self._status.current_point_index,
            pause_reason=pause_reason,
            waiting_for_continue=False,
        )
        self._record(
            "info",
            f"point mission pausing ({pause_reason}) during {phase} at point "
            f"{self._status.current_point_index}",
        )
        await self._confirm_spray_off(run, ros_node, require_confirm=dwell_cancelled)
        state = ros_node.get_state()
        north = float(state.get("pos_n", 0.0))
        east = float(state.get("pos_e", 0.0))
        heading = state.get("heading_ned_rad")
        if heading is not None:
            heading = float(heading)
        if hold_owner is not None:
            hold_owner.activate(
                ros_node,
                north_m=north,
                east_m=east,
                heading_ned_rad=heading,
                reason=pause_reason,
            )
            self._merge_hold_status(run, hold_owner, ros_node)
        target = {
            "operator": TargetMissionState.PAUSED_HOLD,
            "obstacle": TargetMissionState.PAUSED_OBSTACLE,
            "gps_safety": TargetMissionState.PAUSED_GPS_SAFETY,
        }[pause_reason]
        run.resume_gate = asyncio.get_running_loop().create_future()
        self._write(
            run,
            state=target,
            resume_available=True,
            last_transition=f"paused:{phase}",
        )
        self._emit_lifecycle_event(run, "paused",
            point_index=self._status.current_point_index,
            source_index=point.source_index,
            reason=pause_reason,
        )
        params = self._gps_safety_params()
        try:
            while not run.resume_gate.done():
                if pause_reason == "gps_safety":
                    if self._gps_recovery_since is None:
                        recovery_since = None
                    else:
                        recovery_since = self._gps_recovery_since
                    verdict = self._evaluate_gps_safety(
                        ros_node.get_state(),
                        recovery_since=recovery_since,
                        paused=True,
                    )
                    if verdict.ok:
                        self._gps_recovery_since = self._gps_recovery_since or time.monotonic()
                        verdict = self._evaluate_gps_safety(
                            ros_node.get_state(),
                            recovery_since=self._gps_recovery_since,
                            paused=True,
                        )
                    else:
                        self._gps_recovery_since = None
                    self._write_gps_verdict(run, verdict)
                    if (
                        params.resume_policy == RESUME_POLICY_AUTO
                        and verdict.recovery_ready
                        and not run.resume_gate.done()
                    ):
                        run.resume_gate.set_result(True)
                        break
                await asyncio.sleep(0.02)
            if not run.resume_gate.done():
                await run.resume_gate
        except asyncio.CancelledError:
            raise
        finally:
            run.resume_gate = None
        self._check_cancel(run)
        self._gps_recovery_since = None
        self._record(
            "info",
            f"point mission resuming ({pause_reason}) into {phase} at point "
            f"{self._status.current_point_index}",
        )
        self._write(run, state=TargetMissionState.RESUMING, last_transition="resuming", resume_available=False)
        self._emit_lifecycle_event(run, "resumed",
            point_index=self._status.current_point_index,
            source_index=point.source_index,
            reason=pause_reason,
        )
        if hold_owner is not None:
            hold_owner.deactivate(ros_node)
            self._merge_hold_status(run, hold_owner, ros_node)

    async def _poll_interruptions(
        self,
        run: TargetMissionRun,
        ros_node,
        hold_owner,
        point: TargetPoint,
        phase: str,
    ) -> str | None:
        if self._gps_applies():
            verdict = self._evaluate_gps_safety(ros_node.get_state(), paused=self.is_paused())
            if not verdict.ok:
                self._gps_fault_count += 1
                self._gps_last_fault_time = time.monotonic()
                self._gps_recovery_since = None
                verdict.gps_fault_count = self._gps_fault_count
                verdict.last_gps_fault_time_s = self._gps_last_fault_time
                self._write_gps_verdict(run, verdict)
                self._record("warning", f"GPS safety fault: {verdict.reason}")
                if self._gps_safety_params().runtime_policy == RUNTIME_POLICY_FAIL:
                    await self._gps_fail_cycle(run, ros_node, hold_owner, point, phase, verdict)
                    raise RuntimeError(verdict.reason)
                await self._pause_cycle(
                    run, ros_node, hold_owner, point, phase, pause_reason="gps_safety"
                )
                return self._resume_phase_after_pause(phase)
            self._write_gps_verdict(run, verdict)
        obstacle_blocked, obstacle_state = self._write_obstacle_status(run)
        if obstacle_blocked:
            self._record(
                "warning",
                f"obstacle hook {obstacle_state} during {phase}; pausing",
            )
            await self._pause_cycle(
                run, ros_node, hold_owner, point, phase, pause_reason="obstacle"
            )
            return self._resume_phase_after_pause(phase)
        if run.skip_requested:
            ok, _ = await self._skip_cycle(
                run,
                ros_node,
                hold_owner,
                self._status.current_point_index,
                self._status.current_point_index >= len(self._resolved_points) - 1,
            )
            return "skip" if ok else None
        if run.pause_requested:
            run.pause_requested = False
            await self._pause_cycle(
                run, ros_node, hold_owner, point, phase, pause_reason="operator"
            )
            return self._resume_phase_after_pause(phase)
        return None

    async def _run(self, run: TargetMissionRun, ros_node, offboard_ctrl, hold_owner) -> None:
        try:
            params = self._config.point if self._config else PointSprayParams()
            total = len(self._resolved_points)
            index = 0
            while index < total:
                point = self._resolved_points[index]
                self._check_cancel(run)
                self._write(
                    run,
                    current_point_index=index,
                    next_point_index=index,
                    mark_enabled=point.mark,
                    arrival_met=False,
                    settle_met=False,
                    obstacle_clear=self._obstacle_clear,
                )
                self._update_live_diagnostics(run, ros_node, point, params)
                is_last = index >= total - 1
                skipped = await self._execute_point(
                    run,
                    ros_node,
                    hold_owner,
                    offboard_ctrl,
                    point,
                    params,
                    index,
                    is_last=is_last,
                )
                if skipped:
                    if is_last:
                        break
                    index += 1
                    continue
                # Pure mark=false legs never engage spray, so a stale spray node
                # must not fail navigation. Marked legs require confirmed OFF.
                await self._confirm_spray_off(
                    run, ros_node, require_confirm=point.mark
                )
                self._write(
                    run,
                    last_completed_point_index=index,
                    next_point_index=None if is_last else index + 1,
                    active_dwell=False,
                    dwell_remaining_s=0.0,
                    active_dwell_command_id=None,
                    arrival_met=True,
                    settle_met=True,
                )
                # Non-terminal navigate-only (mark=false) targets emit no
                # completion via the marking path; emit exactly one per-target
                # completion here so every original index reaches completion.
                # The final target's completion is carried by the terminal event.
                if not is_last and not point.mark:
                    self._emit_navigation_only_completion(
                        run,
                        point_index=index,
                        source_index=point.source_index,
                    )
                if is_last:
                    break
                if self._execution_mode == TargetExecutionMode.MANUAL:
                    await self._wait_for_continue(run, ros_node, hold_owner, point, index)
                else:
                    self._write(
                        run,
                        state=TargetMissionState.ADVANCING,
                        last_transition=f"advanced:{index}",
                    )
                index += 1
            await self._confirm_spray_off(
                run, ros_node, require_confirm=self._spray_ever_on
            )
            if self._status.active_dwell:
                raise RuntimeError("point completion blocked: dwell still active")
            completion = None
            if self._is_current(run) and offboard_ctrl is not None:
                completion = await offboard_ctrl.complete_async()
            elif offboard_ctrl is None:
                raise RuntimeError("parent controller unavailable for completion")
            if completion is None or not completion.get("success", False):
                reason = (
                    (completion or {}).get("message", "")
                    or "parent completion terminalization failed"
                )
                warnings = "; ".join((completion or {}).get("warnings") or [])
                if warnings:
                    reason = f"{reason}: {warnings}"
                from mission_ops import MissionOperation, MissionOperationCoordinator

                coordinator = self._operation_coordinator() or MissionOperationCoordinator()
                token = await coordinator.begin(
                    MissionOperation.COMPLETION, timeout_s=0.25
                )
                try:
                    await self.terminal_cleanup(
                        ros_node,
                        hold_owner,
                        reason="completion_degraded",
                        terminal_state=TargetMissionState.FAILED,
                        operation_token=token,
                        offboard_ctrl=offboard_ctrl,
                        require_spray_confirm=True,
                    )
                finally:
                    await coordinator.finish(token)
                return
            terminal_safety_ok = True
            terminal_failure_reason = ""
            if self._resolved_points and hold_owner is not None:
                last = self._resolved_points[-1]
                hold_owner.activate(
                    ros_node,
                    north_m=last.north_m,
                    east_m=last.east_m,
                    reason="mission_complete",
                )
                self._merge_hold_status(run, hold_owner, ros_node)
                if not hold_owner.active:
                    terminal_safety_ok = False
                    terminal_failure_reason = "terminal hold failed to activate"
                    self._record(
                        "error",
                        f"terminal safety degraded: {terminal_failure_reason}",
                    )
                else:
                    hold_owner.refresh(ros_node)
            from mission_ops import MissionOperation, MissionOperationCoordinator

            coordinator = self._operation_coordinator() or MissionOperationCoordinator()
            token = await coordinator.begin(MissionOperation.COMPLETION, timeout_s=0.25)
            try:
                if not coordinator.is_current(token):
                    return
                await self.terminal_cleanup(
                    ros_node,
                    hold_owner,
                    reason="normal_completion",
                    terminal_state=TargetMissionState.COMPLETED,
                    operation_token=token,
                    offboard_ctrl=offboard_ctrl,
                    require_spray_confirm=True,
                )
            finally:
                await coordinator.finish(token)
            if self._run_token is run:
                self._write(
                    run,
                    ready=False,
                    next_point_index=None,
                    target_north_m=None,
                    target_east_m=None,
                    current_distance_m=None,
                    mark_enabled=False,
                    terminal_safety_ok=terminal_safety_ok,
                    terminal_safety_reason=terminal_failure_reason,
                    terminal_failure_reason=terminal_failure_reason,
                    recovery_required=not terminal_safety_ok,
                    spray_off_result=(completion or {}).get("spray_off_result"),
                )
        except asyncio.CancelledError:
            if not run.terminal_cleanup_started and self._is_current(run):
                self._write(
                    run,
                    state=TargetMissionState.FAILED,
                    last_error="cancelled",
                    last_failure_reason="cancelled",
                    last_transition="operator_abort",
                    ready=False,
                    run_active=False,
                    waiting_for_continue=False,
                )
            raise
        except TargetMissionRunFailure as exc:
            terminal = exc.terminal_state or (
                TargetMissionState.FAILED_GPS_SAFETY
                if self._status.state == TargetMissionState.FAILED_GPS_SAFETY
                else TargetMissionState.FAILED
            )
            await self._terminal_cleanup_run_failure(
                run,
                ros_node,
                hold_owner,
                offboard_ctrl,
                cleanup_reason=exc.cleanup_reason,
                terminal_state=terminal,
                error_message=str(exc),
            )
            self._record("error", f"point mission failed: {exc}")
        except SprayRuntimeSchemaError as exc:
            await self._terminal_cleanup_run_failure(
                run,
                ros_node,
                hold_owner,
                offboard_ctrl,
                cleanup_reason="dwell_fault",
                terminal_state=TargetMissionState.FAILED,
                error_message=str(exc),
            )
            self._record("error", f"point mission spray schema fault: {exc}")
        except Exception as exc:
            terminal = (
                TargetMissionState.FAILED_GPS_SAFETY
                if self._status.state == TargetMissionState.FAILED_GPS_SAFETY
                else TargetMissionState.FAILED
            )
            await self._terminal_cleanup_run_failure(
                run,
                ros_node,
                hold_owner,
                offboard_ctrl,
                cleanup_reason="dwell_fault",
                terminal_state=terminal,
                error_message=str(exc),
            )
            self._record("error", f"point mission failed: {exc}")
        finally:
            if not run.terminal_cleanup_started:
                cancelled = run.cancel_event.is_set()
                if cancelled:
                    self._write(run, run_active=False, waiting_for_continue=False)
                elif ros_node is not None:
                    # Terminal safety net. Must NOT escape as an unretrieved task
                    # exception (success path is not awaited). Always command OFF;
                    # record honest degraded diagnostics if a sprayed run can't confirm.
                    try:
                        confirmed = await self._force_spray_off_confirmed(
                            ros_node, require_confirm=False
                        )
                        if self._spray_ever_on and not confirmed:
                            self._record(
                                "error",
                                "terminal cleanup: spray OFF not confirmed after spraying run",
                            )
                            self._write(
                                run,
                                terminal_safety_ok=False,
                                terminal_safety_reason=(
                                    "spray OFF not confirmed during terminal cleanup"
                                ),
                            )
                    except asyncio.CancelledError:
                        raise
                    except Exception as exc:  # pragma: no cover - defensive
                        self._record("error", f"terminal cleanup spray-off error: {exc}")
                        self._write(
                            run,
                            terminal_safety_ok=False,
                            terminal_safety_reason=str(exc),
                        )
                self._write(run, run_active=False, waiting_for_continue=False)

    def _check_cancel(self, run: TargetMissionRun) -> None:
        if (
            run.cancel_event.is_set()
            or run.terminal_cleanup_started
            or not self._is_current(run)
        ):
            raise asyncio.CancelledError()
        self._check_operation_generation(run)

    async def _wait_for_continue(
        self, run: TargetMissionRun, ros_node, hold_owner, point: TargetPoint, completed_index: int
    ) -> None:
        if hold_owner is not None:
            hold_owner.activate(
                ros_node,
                north_m=point.north_m,
                east_m=point.east_m,
                reason="manual_wait",
            )
            self._merge_hold_status(run, hold_owner, ros_node)
        run.continue_gate = asyncio.get_running_loop().create_future()
        self._write(
            run,
            state=TargetMissionState.WAITING_FOR_CONTINUE,
            waiting_for_continue=True,
            last_transition=f"waiting_for_continue:{completed_index}",
        )
        self._on_waiting_for_continue(
            run,
            point_index=completed_index,
            source_index=point.source_index,
        )
        try:
            params = self._point_params()
            while not run.continue_gate.done():
                resume = await self._poll_interruptions(
                    run,
                    ros_node,
                    hold_owner,
                    point,
                    TargetMissionState.WAITING_FOR_CONTINUE.value,
                )
                if resume == "waiting_for_continue":
                    break
                drift = await self._poll_hold_drift(
                    run,
                    ros_node,
                    hold_owner,
                    point,
                    params,
                    TargetMissionState.WAITING_FOR_CONTINUE.value,
                )
                if drift is not None:
                    return
                if run.continue_gate.done():
                    break
                await asyncio.sleep(0.02)
            if not run.continue_gate.done():
                await run.continue_gate
        except asyncio.CancelledError:
            raise
        finally:
            run.continue_gate = None
            if hold_owner is not None:
                hold_owner.deactivate(ros_node)
                self._merge_hold_status(run, hold_owner, ros_node)
        self._check_cancel(run)
        self._write(
            run,
            waiting_for_continue=False,
            state=TargetMissionState.ADVANCING,
            last_transition=f"continued:{completed_index}",
        )

    async def _publish_fresh_leg(
        self, run, ros_node, point, params: PointSprayParams
    ) -> None:
        if (
            run.cancel_event.is_set()
            or run.terminal_cleanup_started
            or not self._is_current(run)
        ):
            raise asyncio.CancelledError()
        coordinator = self._operation_coordinator()
        if coordinator is not None and run.operation_generation != coordinator.current_generation():
            raise asyncio.CancelledError()
        state = ros_node.get_state()
        if not state.get("pose_received", False):
            raise RuntimeError("no rover pose for point leg")
        published, diag = self._build_target_leg(state, point, params)
        self._write_leg_diagnostics(run, diag)
        ros_node.publish_path(
            published,
            spray_flags=[False] * len(published),
            runtime_entry=True,
        )
        self._emit_lifecycle_event(run, "leg_started",
            point_index=self._status.current_point_index,
            source_index=point.source_index,
            mark=point.mark,
        )

    async def _execute_point(
        self,
        run,
        ros_node,
        hold_owner,
        offboard_ctrl,
        point,
        params,
        index,
        *,
        is_last: bool = False,
    ) -> bool:
        phase = "navigating"
        started = time.monotonic()
        while phase != "done":
            if phase == "navigating":
                self._write(run, state=TargetMissionState.PREPARING_LEG, last_transition=f"preparing_leg:{index}")
                await ros_node.cancel_spray_dwell_async()
                await self._publish_fresh_leg(run, ros_node, point, params)
                started = time.monotonic()
                self._write(run, state=TargetMissionState.NAVIGATING, last_transition=f"navigating:{index}")
                next_phase = await self._wait_arrival(
                    run, ros_node, hold_owner, point, params, started
                )
                if next_phase == "navigating":
                    continue
                if next_phase == "waiting_for_continue":
                    return False
                if next_phase == "skip":
                    return True
                phase = "settling"
            elif phase == "settling":
                self._write(run, state=TargetMissionState.SETTLING, last_transition=f"settling:{index}")
                next_phase = await self._wait_settled(
                    run, ros_node, hold_owner, point, params, started
                )
                if next_phase == "navigating":
                    phase = "navigating"
                    continue
                if next_phase == "waiting_for_continue":
                    return False
                if next_phase == "skip":
                    return True
                phase = "dwelling" if point.mark else "done"
            elif phase == "dwelling":
                if hold_owner is not None:
                    hold_owner.activate(
                        ros_node,
                        north_m=point.north_m,
                        east_m=point.east_m,
                        reason="dwell",
                    )
                    self._merge_hold_status(run, hold_owner, ros_node)
                self._write(run, state=TargetMissionState.DWELLING, last_transition=f"dwelling:{index}")
                self._command_seq += 1
                command_id = self._command_seq
                command_revision = time.monotonic_ns()
                self._bind_dwell_identity(
                    run,
                    command_id=command_id,
                    command_revision=command_revision,
                    point_index=index,
                    source_index=point.source_index,
                )
                self._write(run, dwell_cancelled=False)
                dwell_s = float(point.dwell_s or params.default_dwell_s)
                ok, why = await ros_node.start_spray_dwell_async(
                    mission_id=run.mission_id,
                    point_index=index,
                    duration_s=dwell_s,
                    command_id=command_id,
                    configuration_revision=self._config.revision,
                )
                if not ok:
                    await self._handle_dwell_start_failure(
                        run,
                        ros_node,
                        offboard_ctrl,
                        command_id=command_id,
                        point_index=index,
                        service_error=why or "dwell rejected",
                    )
                    raise RuntimeError(why or "dwell rejected")
                # Spray has now been engaged this run → terminal/cancel cleanup
                # must require confirmed OFF.
                self._spray_ever_on = True
                self._emit_lifecycle_event(run, "dwell_started",
                    point_index=index,
                    source_index=point.source_index,
                    dwell_command_id=command_id,
                    dwell_remaining_s=dwell_s,
                )
                next_phase = await self._wait_dwell_complete(
                    run,
                    ros_node,
                    hold_owner,
                    offboard_ctrl,
                    point,
                    dwell_s,
                    command_id,
                    params,
                )
                self._write(run, active_dwell_command_id=None)
                if hold_owner is not None and not is_last:
                    hold_owner.deactivate(ros_node)
                    self._merge_hold_status(run, hold_owner, ros_node)
                if next_phase == "navigating":
                    phase = "navigating"
                    continue
                if next_phase == "skip":
                    return True
                phase = "done"
        return False

    def _telemetry_stale(self, state: dict[str, Any]) -> bool:
        pose_age = float(state.get("pose_age_ms", float("inf")))
        velocity_age = state.get("velocity_age_ms")
        return (
            pose_age > 500.0
            or velocity_age is None
            or float(velocity_age) > 500.0
            or int(state.get("rpp_state", RPP_STALE)) == RPP_STALE
        )

    def _arrival_conditions_met(self, state, point, params) -> bool:
        if self._telemetry_stale(state):
            return False
        dist = self._distance_to_point(state, point)
        return (
            dist <= params.arrival_tolerance_m
            and float(state.get("speed_m_s", 0.0)) <= params.settle_speed_mps
            and abs(float(state.get("yaw_rate_rad_s", 0.0))) <= params.settle_yaw_rate_rad_s
        )

    async def _wait_arrival(self, run, ros_node, hold_owner, point, params, started) -> str:
        while True:
            self._check_cancel(run)
            resume = await self._poll_interruptions(
                run, ros_node, hold_owner, point, TargetMissionState.NAVIGATING.value
            )
            if resume is not None:
                return resume
            if time.monotonic() - started > params.leg_timeout_s:
                raise TimeoutError(f"leg timeout at point {self._status.current_point_index}")
            state = ros_node.get_state()
            if self._telemetry_stale(state):
                raise RuntimeError("stale telemetry during navigation")
            arrival_met = self._arrival_conditions_met(state, point, params)
            self._update_live_diagnostics(run, ros_node, point, params, arrival_met=arrival_met, settle_met=False)
            if hold_owner is not None and hold_owner.active:
                hold_owner.refresh(ros_node)
                self._merge_hold_status(run, hold_owner, ros_node)
            if arrival_met:
                self._emit_lifecycle_event(run, "arrived",
                    point_index=self._status.current_point_index,
                    source_index=point.source_index,
                )
                return "settling"
            await asyncio.sleep(0.05)

    async def _wait_settled(self, run, ros_node, hold_owner, point, params, started) -> str:
        settled_since = None
        while True:
            self._check_cancel(run)
            resume = await self._poll_interruptions(
                run, ros_node, hold_owner, point, TargetMissionState.SETTLING.value
            )
            if resume is not None:
                return resume
            if time.monotonic() - started > params.leg_timeout_s:
                raise TimeoutError(f"settle timeout at point {self._status.current_point_index}")
            state = ros_node.get_state()
            if self._telemetry_stale(state):
                raise RuntimeError("stale telemetry during settle")
            arrival_met = self._arrival_conditions_met(state, point, params)
            if arrival_met:
                settled_since = settled_since or time.monotonic()
                if time.monotonic() - settled_since >= params.settle_time_s:
                    self._update_live_diagnostics(
                        run, ros_node, point, params, arrival_met=True, settle_met=True
                    )
                    return "dwelling"
                self._update_live_diagnostics(
                    run, ros_node, point, params, arrival_met=True, settle_met=False
                )
            else:
                settled_since = None
                self._update_live_diagnostics(
                    run, ros_node, point, params, arrival_met=False, settle_met=False
                )
            await asyncio.sleep(0.05)

    async def _handle_dwell_start_failure(
        self,
        run: TargetMissionRun,
        ros_node,
        offboard_ctrl,
        *,
        command_id: int,
        point_index: int,
        service_error: str,
    ) -> None:
        status = ros_node.get_spray_runtime_status()
        if self._dwell_identity_matches(run, status, offboard_ctrl) and bool(
            status.get("active_dwell", False)
        ):
            self._record(
                "warning",
                f"dwell start reported failure ({service_error}) but runtime shows active; "
                "cancelling",
            )
            self._invalidate_dwell_identity(run)
            dwell_cancel = await self._cancel_dwell_service(ros_node)
            spray_off = await self._force_spray_off_with_result(
                ros_node, require_confirm=True
            )
            recovery = bool(
                spray_off.get("recovery_required")
                or not spray_off.get("success", False)
            )
            self._write(
                run,
                dwell_cancel_result=dwell_cancel,
                spray_off_result=spray_off,
                recovery_required=recovery,
                terminal_safety_ok=not recovery,
                terminal_failure_reason=service_error if recovery else "",
            )

    async def _handle_dwell_identity_fault(
        self,
        run: TargetMissionRun,
        ros_node,
        offboard_ctrl,
        *,
        reason: str,
    ) -> None:
        raise TargetMissionRunFailure(reason, cleanup_reason="dwell_fault")

    async def _wait_dwell_complete(
        self,
        run,
        ros_node,
        hold_owner,
        offboard_ctrl,
        point,
        dwell_s,
        command_id,
        params,
    ) -> str:
        deadline = time.monotonic() + dwell_s + 1.0
        observed_active = False
        while time.monotonic() < deadline:
            self._check_cancel(run)
            resume = await self._poll_interruptions(
                run, ros_node, hold_owner, point, TargetMissionState.DWELLING.value
            )
            if resume is not None:
                return resume
            drift = await self._poll_hold_drift(
                run,
                ros_node,
                hold_owner,
                point,
                params,
                TargetMissionState.DWELLING.value,
            )
            if drift is not None:
                return drift
            status = ros_node.get_spray_runtime_status()
            self._validate_dwell_poll_status(status)
            if status["status_stale"]:
                raise RuntimeError("spray runtime status is stale")
            fingerprint = self._spray_runtime_fingerprint(status)
            if run.spray_runtime_fingerprint is None:
                run.spray_runtime_fingerprint = fingerprint
            elif fingerprint[:2] != run.spray_runtime_fingerprint[:2]:
                await self._handle_dwell_identity_fault(
                    run,
                    ros_node,
                    offboard_ctrl,
                    reason="spray runtime restarted during dwell",
                )
            elif fingerprint[2] + 1e-3 < run.spray_runtime_fingerprint[2]:
                await self._handle_dwell_identity_fault(
                    run,
                    ros_node,
                    offboard_ctrl,
                    reason="spray runtime timestamp regressed during dwell",
                )
            if not self._dwell_identity_matches(run, status, offboard_ctrl):
                await self._handle_dwell_identity_fault(
                    run,
                    ros_node,
                    offboard_ctrl,
                    reason="dwell identity mismatch",
                )
            if status.get("last_error") or not status.get("ready", False):
                raise RuntimeError(status.get("last_error") or "spray node is not ready")
            active = status["active_dwell"]
            self._write(
                run,
                active_dwell=active,
                dwell_remaining_s=float(status.get("dwell_remaining_s", 0.0)),
                active_dwell_command_id=command_id,
            )
            if active:
                observed_active = True
            elif observed_active:
                if (
                    not status["commanded_on"]
                    and status["confirmed_off"]
                    and status["off_acknowledged"]
                ):
                    self._invalidate_dwell_identity(run)
                    self._emit_lifecycle_event(run, "marked",
                        point_index=self._status.current_point_index,
                        source_index=point.source_index,
                        dwell_command_id=command_id,
                    )
                    return "done"
            await asyncio.sleep(0.05)
        raise TimeoutError(
            "dwell never became active" if not observed_active else "dwell completion timeout"
        )

    async def _confirm_spray_off(self, run, ros_node, *, require_confirm: bool = True) -> bool:
        result = await self._force_spray_off_with_result(
            ros_node,
            check_cancel=lambda: self._check_cancel(run),
            require_confirm=require_confirm,
            timeout_s=1.0,
        )
        if result.get("success"):
            return True
        if require_confirm:
            raise TimeoutError(result.get("message") or "spray OFF not confirmed")
        self._record(
            "warning",
            "spray OFF commanded but not confirmed (spray status stale/unavailable); "
            "proceeding for non-spraying leg",
        )
        return False

    async def _force_spray_off_confirmed(
        self, ros_node, *, check_cancel=None, require_confirm: bool = True
    ) -> bool:
        """Always command dwell-cancel + spray OFF; optionally require confirmation.

        The OFF command is issued unconditionally. ``require_confirm=True``
        (the default, used for marked legs, spraying pause/fault, and all
        stop/abort/clear/terminal cleanup) waits for confirmed OFF and raises
        ``TimeoutError`` if the spray node never confirms. ``require_confirm=
        False`` (pure ``mark=false`` navigation) treats an unconfirmable/stale
        spray node as a logged warning and returns ``False`` rather than
        failing the mission. Returns whether confirmation was observed.
        """
        result = await force_spray_off_confirmed(
            ros_node,
            timeout_s=1.0,
            check_cancel=check_cancel,
        )
        if result.success:
            return True
        if require_confirm:
            raise TimeoutError(result.message)
        self._record(
            "warning",
            "spray OFF commanded but not confirmed (spray status stale/unavailable); "
            "proceeding for non-spraying leg",
        )
        return False

    async def _skip_cycle(
        self,
        run: TargetMissionRun,
        ros_node,
        hold_owner,
        index: int,
        is_last: bool,
    ) -> tuple[bool, str]:
        if run.terminal_cleanup_started or not self._is_current(run):
            return False, "skip aborted: run is stale"
        coordinator = self._operation_coordinator()
        if coordinator is not None:
            try:
                self._check_operation_generation(run)
            except asyncio.CancelledError:
                return False, "skip preempted"
        point = self._resolved_points[index]
        during_dwell = self._status.state in {
            TargetMissionState.DWELLING,
            TargetMissionState.OBSTACLE_DURING_DWELL,
            TargetMissionState.GPS_DURING_DWELL,
        } or bool(self._status.active_dwell)
        if hold_owner is not None and hold_owner.active:
            hold_owner.deactivate(ros_node)
            self._merge_hold_status(run, hold_owner, ros_node)
        if during_dwell:
            await self._cancel_dwell_service(ros_node)
            spray_off = await self._force_spray_off_with_result(
                ros_node, require_confirm=True
            )
            if spray_off.get("recovery_required") or not spray_off.get("success", False):
                run.skip_requested = False
                self._write(run, skip_pending=False)
                return False, "skip failed: spray OFF not confirmed"
        elif point.mark or self._spray_ever_on:
            await self._force_spray_off_with_result(ros_node, require_confirm=False)
        else:
            await self._force_spray_off_with_result(ros_node, require_confirm=False)
        self._invalidate_dwell_identity(run)
        if run.continue_gate is not None and not run.continue_gate.done():
            run.continue_gate.cancel()
        if run.resume_gate is not None and not run.resume_gate.done():
            run.resume_gate.set_result("skip")
        run.continue_gate = None
        skipped = list(self._status.skipped_point_indices)
        skipped.append(index)
        self._emit_lifecycle_event(run, "skipped",
            point_index=index,
            source_index=point.source_index,
            reason=run.skip_reason or "operator_skip",
        )
        run.skip_requested = False
        self._write(
            run,
            skip_pending=False,
            last_skipped_point_index=index,
            skipped_point_indices=skipped,
            last_completed_point_index=index,
            active_dwell=False,
            dwell_remaining_s=0.0,
            active_dwell_command_id=None,
            arrival_met=True,
            settle_met=True,
            state=TargetMissionState.ADVANCING if not is_last else TargetMissionState.ADVANCING,
            last_transition=f"skipped:{index}",
            next_point_index=None if is_last else index + 1,
        )
        return True, "skipped"
