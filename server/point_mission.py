"""Point-mission orchestrator state machine (async, non-blocking)."""

from __future__ import annotations

import asyncio
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Literal

_SRC = Path(__file__).resolve().parents[1] / "src"
_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from point_ingest import SprayPoint, points_from_staged_dict  # noqa: E402
from point_leg_trajectory import PointLegTrajectoryMode  # noqa: E402
from spray_config import (  # noqa: E402
    GpsSurveyedSafetyParams,
    PointSprayParams,
    SprayConfiguration,
)

from gps_safety import GPS_SAFETY_NA
from logging_setup import get_logger
from mission_placement import GPS_SURVEYED, LOCAL_NED, PlacementError, resolve_surveyed_points
from models import MissionState, PointMissionEvent, PointTerminalCleanupResult
from point_events import get_point_event_journal, utc_ts
from target_mission.core import TargetMissionCoreMixin
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
    _PARENT_ABORT_REASONS,
    _SKIP_ACCEPTED_STATES,
    _TERMINAL_REASON_PRIORITY,
)

try:
    from mission_ops import MissionOperationToken
except ImportError:  # pragma: no cover
    MissionOperationToken = Any  # type: ignore[misc,assignment]

log = get_logger("server.point_mission")

# Backward-compatible aliases
PointMissionState = TargetMissionState
PointMissionRun = TargetMissionRun
PointMissionRunFailure = TargetMissionRunFailure
PointExecutionMode = TargetExecutionMode


@dataclass
class PointMissionStatus:
    state: PointMissionState = PointMissionState.IDLE
    mission_id: str = ""
    generation: int = 0
    current_point_index: int = 0
    total_points: int = 0
    active_dwell: bool = False
    dwell_remaining_s: float = 0.0
    last_transition: str = ""
    last_error: str = ""
    ready: bool = False
    source_frame: str = ""
    resolved_runtime_frame: str = ""
    point_execution_mode: str = PointExecutionMode.AUTO.value
    waiting_for_continue: bool = False
    last_completed_point_index: int | None = None
    next_point_index: int | None = None
    target_north_m: float | None = None
    target_east_m: float | None = None
    current_distance_m: float | None = None
    arrival_met: bool = False
    settle_met: bool = False
    mark_enabled: bool = True
    active_dwell_command_id: int | None = None
    parent_mission_id: str = ""
    point_mission_generation: int = 0
    active_dwell_command_revision: int | None = None
    active_dwell_configuration_revision: int | None = None
    active_dwell_point_index: int | None = None
    active_dwell_source_index: int | None = None
    recovery_required: bool = False
    terminal_failure_reason: str = ""
    dwell_cancel_result: dict[str, Any] | None = None
    spray_off_result: dict[str, Any] | None = None
    last_failure_reason: str = ""
    run_active: bool = False
    obstacle_clear: bool = True
    obstacle_integration_enabled: bool = False
    obstacle_signal_state: str = "not_configured"
    obstacle_signal_age_ms: float | None = None
    terminal_safety_ok: bool = True
    terminal_safety_reason: str = ""
    pause_reason: str = ""
    pre_pause_state: str = ""
    paused_point_index: int | None = None
    resume_available: bool = False
    dwell_cancelled: bool = False
    dwell_ownership_invalidated: bool = False
    setpoint_source: str = "rpp"
    hold_active: bool = False
    hold_north_m: float | None = None
    hold_east_m: float | None = None
    hold_heading_ned_rad: float | None = None
    hold_error_m: float | None = None
    gps_safety_state: str = GPS_SAFETY_NA
    gps_safety_ok: bool = True
    gps_required_fix_type: int | None = None
    gps_current_fix_type: int | None = None
    gps_global_position_age_ms: float | None = None
    gps_local_pose_age_ms: float | None = None
    gps_fix_age_ms: float | None = None
    gps_pose_global_skew_ms: float | None = None
    gps_anchor_valid: bool | None = None
    gps_last_safety_reason: str = ""
    gps_fault_count: int = 0
    gps_last_fault_time_s: float | None = None
    gps_recovery_ready: bool = False
    gps_runtime_policy: str | None = None
    gps_resume_policy: str | None = None
    point_leg_trajectory_mode: str = PointLegTrajectoryMode.TWO_POINT.value
    point_leg_spacing_m: float = 0.08
    point_leg_published_count: int | None = None
    point_leg_conditioned_count: int | None = None
    active_trajectory_mode: str | None = None
    point_leg_length_m: float | None = None
    last_skipped_point_index: int | None = None
    skipped_point_indices: list[int] = field(default_factory=list)
    skip_pending: bool = False

    def as_dict(self) -> dict[str, Any]:
        return {
            "point_mission_state": self.state.value,
            "point_mission_id": self.mission_id,
            "point_mission_generation": self.generation,
            "current_point_index": self.current_point_index,
            "total_points": self.total_points,
            "active_dwell": self.active_dwell,
            "dwell_remaining_s": self.dwell_remaining_s,
            "last_transition": self.last_transition,
            "last_error": self.last_error,
            "ready": self.ready,
            "source_frame": self.source_frame,
            "resolved_runtime_frame": self.resolved_runtime_frame,
            "point_execution_mode": self.point_execution_mode,
            "waiting_for_continue": self.waiting_for_continue,
            "last_completed_point_index": self.last_completed_point_index,
            "next_point_index": self.next_point_index,
            "target_north_m": self.target_north_m,
            "target_east_m": self.target_east_m,
            "current_distance_m": self.current_distance_m,
            "arrival_met": self.arrival_met,
            "settle_met": self.settle_met,
            "mark_enabled": self.mark_enabled,
            "active_dwell_command_id": self.active_dwell_command_id,
            "parent_mission_id": self.parent_mission_id,
            "point_mission_generation": self.point_mission_generation,
            "active_dwell_command_revision": self.active_dwell_command_revision,
            "active_dwell_configuration_revision": (
                self.active_dwell_configuration_revision
            ),
            "active_dwell_point_index": self.active_dwell_point_index,
            "active_dwell_source_index": self.active_dwell_source_index,
            "recovery_required": self.recovery_required,
            "terminal_failure_reason": self.terminal_failure_reason,
            "dwell_cancel_result": self.dwell_cancel_result,
            "spray_off_result": self.spray_off_result,
            "last_failure_reason": self.last_failure_reason,
            "run_active": self.run_active,
            "obstacle_clear": self.obstacle_clear,
            "obstacle_integration_enabled": self.obstacle_integration_enabled,
            "obstacle_signal_state": self.obstacle_signal_state,
            "obstacle_signal_age_ms": self.obstacle_signal_age_ms,
            "terminal_safety_ok": self.terminal_safety_ok,
            "terminal_safety_reason": self.terminal_safety_reason,
            "pause_reason": self.pause_reason,
            "pre_pause_state": self.pre_pause_state,
            "paused_point_index": self.paused_point_index,
            "resume_available": self.resume_available,
            "dwell_cancelled": self.dwell_cancelled,
            "dwell_ownership_invalidated": self.dwell_ownership_invalidated,
            "setpoint_source": self.setpoint_source,
            "hold_active": self.hold_active,
            "hold_north_m": self.hold_north_m,
            "hold_east_m": self.hold_east_m,
            "hold_heading_ned_rad": self.hold_heading_ned_rad,
            "hold_error_m": self.hold_error_m,
            "gps_safety_state": self.gps_safety_state,
            "gps_safety_ok": self.gps_safety_ok,
            "gps_required_fix_type": self.gps_required_fix_type,
            "gps_current_fix_type": self.gps_current_fix_type,
            "gps_global_position_age_ms": self.gps_global_position_age_ms,
            "gps_local_pose_age_ms": self.gps_local_pose_age_ms,
            "gps_fix_age_ms": self.gps_fix_age_ms,
            "gps_pose_global_skew_ms": self.gps_pose_global_skew_ms,
            "gps_anchor_valid": self.gps_anchor_valid,
            "gps_last_safety_reason": self.gps_last_safety_reason,
            "gps_fault_count": self.gps_fault_count,
            "gps_last_fault_time_s": self.gps_last_fault_time_s,
            "gps_recovery_ready": self.gps_recovery_ready,
            "gps_runtime_policy": self.gps_runtime_policy,
            "gps_resume_policy": self.gps_resume_policy,
            "point_leg_trajectory_mode": self.point_leg_trajectory_mode,
            "point_leg_spacing_m": self.point_leg_spacing_m,
            "point_leg_published_count": self.point_leg_published_count,
            "point_leg_conditioned_count": self.point_leg_conditioned_count,
            "active_trajectory_mode": self.active_trajectory_mode,
            "point_leg_length_m": self.point_leg_length_m,
            "last_skipped_point_index": self.last_skipped_point_index,
            "skipped_point_indices": list(self.skipped_point_indices),
            "skip_pending": self.skip_pending,
        }

    def as_spray_status_dict(self) -> dict[str, Any]:
        """Point status view for /api/spray/status without spray-runtime collisions."""
        payload = self.as_dict()
        payload.update(
            {
                "point_ready": self.ready,
                "point_active_dwell": self.active_dwell,
                "point_dwell_remaining_s": self.dwell_remaining_s,
                "point_last_transition": self.last_transition,
                "point_last_error": self.last_error,
                "point_hold_active": self.hold_active,
            }
        )
        for key in (
            "ready",
            "active_dwell",
            "dwell_remaining_s",
            "last_transition",
            "last_error",
            "hold_active",
        ):
            payload.pop(key, None)
        return payload


class PointMissionOrchestrator(TargetMissionCoreMixin):
    # Drain budget for cancel_and_drain. Sized above the worst-case task
    # unwind: the run's ``finally`` forces spray OFF, which issues spray
    # services each bounded by ~5 s timeouts. A short budget here would expire
    # before the task drains and (previously) skip the safety cleanup; the
    # cleanup now runs unconditionally regardless of this timeout.
    _DRAIN_TIMEOUT_S = 6.0

    def __init__(self) -> None:
        self._status = PointMissionStatus()
        self._points: list[SprayPoint] = []
        self._resolved_points: list[SprayPoint] = []
        self._config: SprayConfiguration | None = None
        self._execution_mode = PointExecutionMode.AUTO
        self._task: asyncio.Task | None = None
        self._run_token: PointMissionRun | None = None
        self._generation = 0
        self._command_seq = 0
        self._source_frame = ""
        self._origin_gps: tuple[float, float] | None = None
        self._obstacle_clear = True
        self._obstacle_last_recv: float | None = None
        self._spray_ever_on = False
        self._gps_recovery_since: float | None = None
        self._gps_fault_count = 0
        self._gps_last_fault_time: float | None = None
        self._log_cb: Callable[[str, str], None] | None = None
        self._command_lock = asyncio.Lock()
        self._event_lock = threading.RLock()
        self._terminal_cleanup_reason: str | None = None



    @property
    def status(self) -> PointMissionStatus:
        return self._status

    def is_active(self) -> bool:
        return self._task is not None and not self._task.done()

    def _mark_offboard_terminal(self, offboard_ctrl, state: MissionState) -> None:
        """Legacy state-only writes — COMPLETED is forbidden; use parent terminal APIs."""
        if offboard_ctrl is None:
            return
        if state == MissionState.COMPLETED:
            raise RuntimeError(
                "point mission must not mark parent COMPLETED directly; "
                "use complete_async()"
            )
        offboard_ctrl.state = state
        if hasattr(offboard_ctrl, "_running_mission_id"):
            offboard_ctrl._running_mission_id = None
        try:
            from control_arbiter import get_control_arbiter

            get_control_arbiter().mark_idle_if_not_joystick()
        except Exception:
            log.exception("point mission terminal arbiter cleanup failed")

    async def _terminal_cleanup_run_failure(
        self,
        run: PointMissionRun,
        ros_node,
        hold_owner,
        offboard_ctrl,
        *,
        cleanup_reason: Literal[
            "dwell_fault", "start_failure", "completion_degraded"
        ],
        terminal_state: PointMissionState,
        error_message: str,
        abort_parent: bool = True,
    ) -> None:
        from mission_ops import MissionOperation, MissionOperationCoordinator

        coordinator = self._operation_coordinator() or MissionOperationCoordinator()
        token = await coordinator.begin(MissionOperation.ABORT, timeout_s=0.25)
        try:
            if not run.terminal_cleanup_started:
                await self.terminal_cleanup(
                    ros_node,
                    hold_owner,
                    reason=cleanup_reason,
                    terminal_state=terminal_state,
                    operation_token=token,
                    offboard_ctrl=offboard_ctrl,
                    require_spray_confirm=True,
                )
            if abort_parent and offboard_ctrl is not None:
                await offboard_ctrl.abort_async()
            self._status.last_error = error_message
            self._status.last_failure_reason = error_message
            self._status.terminal_failure_reason = error_message
            self._status.ready = False
        finally:
            await coordinator.finish(token)

    def _emit_lifecycle_event(
        self,
        run: PointMissionRun | None,
        kind: str,
        **kwargs: Any,
    ) -> None:
        mapping = {
            "leg_started": "point_leg_started",
            "arrived": "point_arrived",
            "dwell_started": "point_dwell_started",
            "marked": "point_marked",
            "paused": "point_paused",
            "resumed": "point_resumed",
            "skipped": "point_skipped",
        }
        event_type = mapping.get(kind)
        if event_type is None:
            raise ValueError(f"unknown lifecycle event kind: {kind!r}")
        self._emit_point_event(run, event_type, **kwargs)

    def _on_waiting_for_continue(
        self,
        run: PointMissionRun | None,
        *,
        point_index: int,
        source_index: int,
    ) -> None:
        self._emit_point_event(
            run,
            "point_waiting_for_continue",
            point_index=point_index,
            source_index=source_index,
        )

    def _build_point_leg(
        self,
        state: dict[str, Any],
        point: SprayPoint,
        params: PointSprayParams,
    ) -> tuple[list[tuple[float, float]], dict[str, Any]]:
        return build_target_leg(state, point.north_m, point.east_m, params)











    def _build_event(
        self,
        event_type: str,
        *,
        point_index: int | None = None,
        source_index: int | None = None,
        terminal: bool = False,
        reason: str = "",
        mark: bool | None = None,
        dwell_command_id: int | None = None,
        dwell_remaining_s: float | None = None,
    ) -> PointMissionEvent:
        status = self._status
        ts = utc_ts()
        resolved_index = point_index if point_index is not None else status.current_point_index
        return PointMissionEvent(
            event_id=0,
            ts=ts,
            timestamp=ts,
            event_type=event_type,  # type: ignore[arg-type]
            mission_id=status.mission_id,
            parent_mission_id=status.parent_mission_id or status.mission_id,
            point_mission_generation=status.generation,
            generation=status.generation,
            point_index=resolved_index,
            source_index=source_index,
            point_mission_state=status.state.value,
            mark=mark if mark is not None else status.mark_enabled,
            dwell_command_id=dwell_command_id if dwell_command_id is not None else status.active_dwell_command_id,
            dwell_remaining_s=(
                dwell_remaining_s
                if dwell_remaining_s is not None
                else status.dwell_remaining_s
            ),
            hold_active=status.hold_active,
            obstacle_signal_state=status.obstacle_signal_state,
            gps_safety_state=status.gps_safety_state,
            terminal=terminal,
            reason=reason,
            message=reason,
            status=status.as_dict(),
        )

    def _emit_point_event(
        self,
        run: PointMissionRun | None,
        event_type: str,
        *,
        terminal: bool = False,
        reason: str = "",
        **kwargs: Any,
    ) -> None:
        if run is not None and run.terminal_event_emitted and terminal:
            return
        with self._event_lock:
            event = self._build_event(event_type, terminal=terminal, reason=reason, **kwargs)
            get_point_event_journal().append(event)
        if run is not None and terminal:
            run.terminal_event_emitted = True




    def load(
        self,
        *,
        mission_id: str,
        points: list[SprayPoint],
        config: SprayConfiguration,
        execution_mode: PointExecutionMode | str = PointExecutionMode.AUTO,
    ) -> None:
        """Synchronous load for an idle orchestrator (used by unit callers)."""
        if self.is_active():
            raise RuntimeError("active point mission must be replaced asynchronously")
        mode = (
            execution_mode
            if isinstance(execution_mode, PointExecutionMode)
            else PointExecutionMode.parse(execution_mode)
        )
        self._install(mission_id, points, config, LOCAL_NED, None, mode)

    async def replace_from_staged(
        self,
        staged: dict[str, Any],
        config: SprayConfiguration,
        ros_node,
        offboard_ctrl=None,
    ) -> None:
        await self.cancel_and_drain(
            ros_node, reason="reload", offboard_ctrl=offboard_ctrl
        )
        rows = staged.get("point_mission_points") or []
        points = points_from_staged_dict(
            rows,
            default_dwell_s=config.point.default_dwell_s,
            max_dwell_s=config.point.max_dwell_s,
        )
        frame = str(staged.get("point_source_frame") or "").upper()
        anchor = staged.get("anchor")
        if not frame:
            raise PlacementError("Point mission is missing explicit point_source_frame metadata")
        if frame not in {LOCAL_NED, GPS_SURVEYED}:
            raise PlacementError(f"unsupported Point source_frame {frame!r}")
        origin = None
        if frame == GPS_SURVEYED:
            if not anchor or anchor.get("lat") is None or anchor.get("lon") is None:
                raise PlacementError("GPS_SURVEYED Point mission is missing its survey anchor")
            origin = (float(anchor["lat"]), float(anchor["lon"]))
        mode = PointExecutionMode.parse(staged.get("point_execution_mode", PointExecutionMode.AUTO.value))
        self._install(str(staged.get("mission_id", "") or ""), points, config, frame, origin, mode)

    def _install(
        self,
        mission_id,
        points,
        config,
        source_frame,
        origin_gps,
        execution_mode: PointExecutionMode,
    ) -> None:
        self._generation += 1
        self._points = list(points)
        self._resolved_points = []
        self._config = config
        self._execution_mode = execution_mode
        self._source_frame = source_frame
        self._origin_gps = origin_gps
        self._run_token = None
        self._status = PointMissionStatus(
            state=PointMissionState.IDLE,
            mission_id=mission_id,
            generation=self._generation,
            total_points=len(points),
            ready=True,
            last_transition="loaded",
            source_frame=source_frame,
            point_execution_mode=execution_mode.value,
        )

    def _empty_status(self, *, last_transition: str, ready: bool = False) -> PointMissionStatus:
        return PointMissionStatus(
            state=PointMissionState.IDLE,
            last_transition=last_transition,
            ready=ready,
        )

    async def clear_mission(
        self, ros_node, *, reason: str = "cleared", offboard_ctrl=None
    ) -> None:
        """Cancel any active run, force spray OFF, and reset to unloaded IDLE."""
        await self.cancel_and_drain(
            ros_node, reason=reason, offboard_ctrl=offboard_ctrl
        )
        self._points = []
        self._resolved_points = []
        self._config = None
        self._execution_mode = PointExecutionMode.AUTO
        self._source_frame = ""
        self._origin_gps = None
        self._run_token = None
        self._task = None
        self._status = self._empty_status(last_transition=reason)

    async def cancel_and_drain(
        self,
        ros_node,
        *,
        reason: str = "cancelled",
        offboard_ctrl=None,
    ) -> None:
        """Cancel the active run and guarantee cleanup.

        Nested try/finally ensures task-drain timeout, ``CancelledError``, and
        intermediate cleanup exceptions cannot skip dwell cancel or forced OFF.
        """
        run, task = self._run_token, self._task
        dwell_cancel_result: dict[str, Any] | None = None
        spray_off_result: dict[str, Any] | None = None
        recovery_required = False
        terminal_failure_reason = ""
        terminal_safety_ok = True
        try:
            if run is not None:
                run.cancel_event.set()
                run.pause_requested = False
                if run.continue_gate is not None and not run.continue_gate.done():
                    run.continue_gate.cancel()
                if run.resume_gate is not None and not run.resume_gate.done():
                    run.resume_gate.cancel()
                self._write(
                    run,
                    state=PointMissionState.ABORTING,
                    last_transition=reason,
                    ready=False,
                    waiting_for_continue=False,
                    run_active=False,
                )
            self._invalidate_dwell_identity(run)
            if task is not None and not task.done():
                task.cancel()
                try:
                    await asyncio.wait_for(
                        asyncio.shield(task), timeout=self._DRAIN_TIMEOUT_S
                    )
                except asyncio.CancelledError:
                    pass
                except asyncio.TimeoutError:
                    self._record(
                        "error",
                        f"point mission cancellation did not drain within "
                        f"{self._DRAIN_TIMEOUT_S}s ({reason}); forcing cleanup",
                    )
                except Exception:
                    pass
        finally:
            try:
                if ros_node is not None:
                    try:
                        dwell_cancel_result = await self._cancel_dwell_service(ros_node)
                    except asyncio.CancelledError:
                        raise
                    except Exception as exc:
                        dwell_cancel_result = {
                            "success": False,
                            "message": str(exc),
                        }
            finally:
                try:
                    if ros_node is not None:
                        try:
                            spray_off_result = await self._force_spray_off_with_result(
                                ros_node, require_confirm=True
                            )
                        except asyncio.CancelledError:
                            raise
                        except Exception as exc:
                            spray_off_result = {
                                "success": False,
                                "message": str(exc),
                                "recovery_required": True,
                            }
                finally:
                    if dwell_cancel_result is not None and not dwell_cancel_result.get(
                        "success", False
                    ):
                        recovery_required = True
                        terminal_safety_ok = False
                        terminal_failure_reason = (
                            dwell_cancel_result.get("message")
                            or "dwell cancel failed during cancellation"
                        )
                    if spray_off_result is not None and (
                        spray_off_result.get("recovery_required")
                        or not spray_off_result.get("success", False)
                    ):
                        recovery_required = True
                        terminal_safety_ok = False
                        terminal_failure_reason = (
                            spray_off_result.get("failure_reason")
                            or spray_off_result.get("message")
                            or terminal_failure_reason
                            or "spray OFF not confirmed during cancellation"
                        )
                        self._record(
                            "error",
                            f"forced spray-off during {reason} not confirmed: "
                            f"{terminal_failure_reason}",
                        )
                    if reason in _PARENT_ABORT_REASONS:
                        parent = await self._parent_abort_terminal(
                            offboard_ctrl, run, reason=reason
                        )
                        if parent is not None and not parent.get("success", False):
                            recovery_required = True
                            terminal_safety_ok = False
                            terminal_failure_reason = (
                                terminal_failure_reason
                                or parent.get("message", "")
                            )
                    if self._run_token is run:
                        self._write(
                            run,
                            active_dwell=False,
                            dwell_remaining_s=0.0,
                            active_dwell_command_id=None,
                            active_dwell_command_revision=None,
                            active_dwell_configuration_revision=None,
                            active_dwell_point_index=None,
                            active_dwell_source_index=None,
                            run_active=False,
                            waiting_for_continue=False,
                            dwell_cancel_result=dwell_cancel_result,
                            spray_off_result=spray_off_result,
                            recovery_required=recovery_required,
                            terminal_safety_ok=terminal_safety_ok,
                            terminal_failure_reason=terminal_failure_reason,
                            terminal_safety_reason=terminal_failure_reason,
                        )
                        self._task = None
                        self._run_token = None

    async def abort(self, ros_node, offboard_ctrl=None) -> None:
        await self.cancel_and_drain(
            ros_node, reason="abort", offboard_ctrl=offboard_ctrl
        )

    async def stop_mission(
        self,
        ros_node,
        hold_owner,
        *,
        reason: str = "stopped",
        offboard_ctrl=None,
    ) -> None:
        """Legacy stop entry — prefer terminal_cleanup via mission services."""
        if hold_owner is not None:
            hold_owner.deactivate(ros_node)
        await self.cancel_and_drain(
            ros_node, reason=reason, offboard_ctrl=offboard_ctrl
        )

    async def terminal_cleanup(
        self,
        ros_node,
        hold_owner,
        *,
        reason: Literal[
            "normal_completion",
            "operator_stop",
            "operator_abort",
            "emergency_stop",
            "start_failure",
            "restart_stop_first",
            "completion_degraded",
            "dwell_fault",
        ],
        terminal_state: PointMissionState,
        operation_token: MissionOperationToken,
        offboard_ctrl=None,
        require_spray_confirm: bool = True,
    ) -> PointTerminalCleanupResult:
        async with self._command_lock:
            coordinator = self._operation_coordinator()
            if coordinator is not None:
                try:
                    operation_token.raise_if_stale(coordinator.current_generation())
                except Exception:
                    if operation_token.is_preempted():
                        return PointTerminalCleanupResult(
                            success=False,
                            idempotent=False,
                            reason=reason,
                            point_mission_state=self._status.state.value,
                            hold_deactivated=False,
                            terminal_event_emitted=False,
                            recovery_required=False,
                            message="terminal cleanup preempted",
                        )
            prior = self._terminal_cleanup_reason
            if prior is not None and self._terminal_reason_priority(
                prior
            ) >= self._terminal_reason_priority(reason):
                return PointTerminalCleanupResult(
                    success=True,
                    idempotent=True,
                    reason=reason,
                    point_mission_state=self._status.state.value,
                    hold_deactivated=False,
                    terminal_event_emitted=bool(
                        self._run_token and self._run_token.terminal_event_emitted
                    ),
                    recovery_required=self._status.recovery_required,
                    message="terminal cleanup already completed",
                )
            run, task = self._run_token, self._task
            current_task = asyncio.current_task()
            cleanup_from_run_task = task is not None and task is current_task
            if run is not None:
                run.terminal_cleanup_started = True
                run.cancel_event.set()
                run.pause_requested = False
                run.skip_requested = False
                if run.continue_gate is not None and not run.continue_gate.done():
                    run.continue_gate.cancel()
                if run.resume_gate is not None and not run.resume_gate.done():
                    run.resume_gate.cancel()
                run.continue_gate = None
                run.resume_gate = None
            self._invalidate_dwell_identity(run)
            dwell_cancel_result: dict[str, Any] | None = None
            spray_off_result: dict[str, Any] | None = None
            hold_deactivated = False
            recovery_required = False
            terminal_failure = ""
            if task is not None and not task.done() and not cleanup_from_run_task:
                task.cancel()
                try:
                    await asyncio.wait_for(
                        asyncio.shield(task), timeout=self._DRAIN_TIMEOUT_S
                    )
                except (asyncio.CancelledError, asyncio.TimeoutError, Exception):
                    pass
            if ros_node is not None:
                dwell_cancel_result = await self._cancel_dwell_service(ros_node)
                spray_off_result = await self._force_spray_off_with_result(
                    ros_node, require_confirm=require_spray_confirm, timeout_s=1.0
                )
            preserve_hold = terminal_state == PointMissionState.FAILED_GPS_SAFETY
            if hold_owner is not None and reason != "normal_completion" and not preserve_hold:
                hold_owner.deactivate(ros_node)
                hold_deactivated = True
            if dwell_cancel_result is not None and not dwell_cancel_result.get(
                "success", False
            ):
                recovery_required = True
                terminal_failure = dwell_cancel_result.get("message") or "dwell cancel failed"
            if spray_off_result is not None and (
                spray_off_result.get("recovery_required")
                or not spray_off_result.get("success", False)
            ):
                recovery_required = True
                terminal_failure = (
                    spray_off_result.get("failure_reason")
                    or spray_off_result.get("message")
                    or terminal_failure
                    or "spray OFF not confirmed"
                )
            last_error = ""
            if reason != "normal_completion":
                last_error = terminal_failure or reason
            if run is not None:
                self._write(
                    run,
                    state=terminal_state,
                    run_active=False,
                    waiting_for_continue=False,
                    resume_available=False,
                    active_dwell=False,
                    dwell_remaining_s=0.0,
                    active_dwell_command_id=None,
                    terminal_failure_reason=terminal_failure or reason,
                    last_transition=reason,
                    last_error=last_error,
                    dwell_cancel_result=dwell_cancel_result,
                    spray_off_result=spray_off_result,
                    recovery_required=recovery_required,
                    skip_pending=False,
                )
            else:
                self._status.state = terminal_state
                self._status.run_active = False
                self._status.waiting_for_continue = False
                self._status.resume_available = False
                self._status.active_dwell = False
                self._status.dwell_remaining_s = 0.0
                self._status.active_dwell_command_id = None
                self._status.terminal_failure_reason = terminal_failure or reason
                self._status.last_transition = reason
                self._status.last_error = last_error
                self._status.dwell_cancel_result = dwell_cancel_result
                self._status.spray_off_result = spray_off_result
                self._status.recovery_required = recovery_required
                self._status.skip_pending = False
            self._run_token = None
            self._terminal_cleanup_reason = reason
            event_map = {
                "normal_completion": "point_completed",
                "operator_stop": "point_aborted",
                "operator_abort": "point_aborted",
                "emergency_stop": "point_aborted",
                "restart_stop_first": "point_aborted",
                "start_failure": "point_failed",
                "completion_degraded": "point_failed",
                "dwell_fault": "point_failed",
            }
            event_type = event_map.get(reason, "point_aborted")
            terminal_event_emitted = False
            if run is None or not run.terminal_event_emitted:
                self._emit_point_event(
                    run,
                    event_type,
                    terminal=True,
                    reason=reason,
                )
                terminal_event_emitted = True
                if run is not None:
                    run.terminal_event_emitted = True
            return PointTerminalCleanupResult(
                success=not recovery_required,
                idempotent=False,
                reason=reason,
                point_mission_state=terminal_state.value,
                dwell_cancel_result=dwell_cancel_result,
                spray_off_result=spray_off_result,
                hold_deactivated=hold_deactivated,
                terminal_event_emitted=terminal_event_emitted,
                recovery_required=recovery_required,
                message="terminal cleanup completed"
                if not recovery_required
                else terminal_failure or "terminal cleanup degraded",
            )

    def reset_for_restart(self, expected_mission_id: str) -> int:
        if self._status.mission_id and self._status.mission_id != expected_mission_id:
            raise ValueError("restart rejected: mission identity mismatch")
        self._generation += 1
        self._terminal_cleanup_reason = None
        self._resolved_points = []
        self._spray_ever_on = False
        self._run_token = None
        self._task = None
        self._status = PointMissionStatus(
            state=PointMissionState.IDLE,
            mission_id=expected_mission_id or self._status.mission_id,
            generation=self._generation,
            total_points=len(self._points),
            ready=bool(self._points),
            last_transition="restart_reset",
            source_frame=self._source_frame,
            point_execution_mode=self._execution_mode.value,
            parent_mission_id=expected_mission_id or self._status.mission_id,
            point_mission_generation=self._generation,
        )
        return self._generation

    async def skip_point(
        self,
        ros_node,
        hold_owner,
        *,
        point_index: int,
        expected_generation: int | None,
        reason: str,
        operation_token: MissionOperationToken,
    ) -> tuple[bool, str, int]:
        coordinator = self._operation_coordinator()
        if coordinator is not None:
            try:
                operation_token.raise_if_stale(coordinator.current_generation())
            except Exception:
                return False, "mission operation token is stale", 409
            if operation_token.is_preempted():
                return False, "skip preempted by higher-priority operation", 409
        reject = self._terminal_reject()
        if reject is not None:
            return reject
        if self._config is None or not self._points:
            return False, "no point mission loaded", 409
        if not self.is_active():
            return False, "point mission is not active", 409
        if expected_generation is not None and expected_generation != self._generation:
            return False, "stale point mission generation", 409
        if point_index != self._status.current_point_index:
            return False, "skip point_index does not match active point", 409
        if self._status.state == PointMissionState.WAITING_FOR_CONTINUE:
            return False, "point already completed; use continue", 409
        if self._status.state == PointMissionState.COMPLETED:
            return False, "point mission already completed", 409
        if self._status.state == PointMissionState.PAUSED_GPS_SAFETY:
            return False, "GPS safety blocks skip until recovery", 409
        if self._status.state not in _SKIP_ACCEPTED_STATES:
            return (
                False,
                f"point mission is {self._status.state.value}, skip not accepted",
                409,
            )
        if self._status.skip_pending:
            return False, "skip already pending", 409
        if self._status.state in {
            PointMissionState.PAUSED_OBSTACLE,
            PointMissionState.OBSTACLE_DURING_DWELL,
        }:
            blocked, obstacle_state = self._write_obstacle_status(self._run_token)
            if blocked:
                return False, f"obstacle {obstacle_state} - cannot skip while blocked", 409
        run = self._run_token
        if run is None:
            return False, "point mission is not active", 409
        if coordinator is not None:
            run.operation_generation = coordinator.current_generation()
        async with self._command_lock:
            run.skip_requested = True
            run.skip_request_id = point_index
            run.skip_reason = reason
            self._write(run, skip_pending=True)
            if self._status.state in PAUSED_STATES:
                is_last = self._status.current_point_index >= len(self._resolved_points) - 1
                ok, msg = await self._skip_cycle(
                    run, ros_node, hold_owner, self._status.current_point_index, is_last
                )
                if not ok:
                    return False, msg, 503
        return True, "skip accepted", 200


    async def pause_mission(self, ros_node, hold_owner) -> tuple[bool, str, int]:
        reject = self._terminal_reject()
        if reject is not None:
            return reject
        if self._config is None or not self._points:
            return False, "no point mission loaded", 409
        if self._status.state in {PointMissionState.COMPLETED, PointMissionState.FAILED, PointMissionState.ABORTING}:
            return False, f"point mission is terminal: {self._status.state.value}", 409
        if self.is_paused():
            return False, "point mission already paused", 409
        if not self.is_active():
            return False, "point mission is not active", 409
        run = self._run_token
        if run is None:
            return False, "point mission is not active", 409
        run.pause_requested = True
        await asyncio.sleep(0)
        return True, "pause requested", 200

    async def resume_mission(
        self,
        ros_node,
        hold_owner,
        *,
        expected_generation: int | None = None,
    ) -> tuple[bool, str, int]:
        reject = self._terminal_reject()
        if reject is not None:
            return reject
        if self._config is None or not self._points:
            return False, "no point mission loaded", 409
        if expected_generation is not None and expected_generation != self._generation:
            return False, "stale point mission generation", 409
        if not self.is_paused():
            return False, f"point mission is {self._status.state.value}, not paused", 409
        obstacle_blocked, obstacle_state = self._write_obstacle_status(self._run_token)
        if obstacle_blocked:
            return False, f"obstacle {obstacle_state} — cannot resume", 409
        run = self._run_token
        if self._status.state == PointMissionState.PAUSED_GPS_SAFETY:
            verdict = self._evaluate_gps_safety(
                ros_node.get_state(),
                recovery_since=self._gps_recovery_since,
                paused=True,
            )
            self._write_gps_verdict(run, verdict)
            if not verdict.recovery_ready:
                return False, "GPS placement not stable for recovery", 409
            try:
                self._resolved_points = self._resolve_points(ros_node.get_state())
            except PlacementError as exc:
                return False, str(exc), 409
        if hold_owner is None or not hold_owner.active:
            return False, "hold is not active", 409
        if not self._resume_health_ok(ros_node):
            return False, "pose or FCU telemetry not healthy for resume", 409
        if run is None or not self.is_active():
            return False, "point mission is not active", 409
        gate = run.resume_gate
        if gate is None or gate.done():
            return False, "point mission is not awaiting resume", 409
        gate.set_result(True)
        await asyncio.sleep(0)
        return True, "resume accepted", 200

    def _resume_health_ok(self, ros_node) -> bool:
        if ros_node is None:
            return False
        state = ros_node.get_state()
        if not state.get("pose_received", False):
            return False
        if not state.get("connected", False):
            return False
        if self._telemetry_stale(state):
            return False
        return True

    async def continue_point(self, ros_node=None) -> tuple[bool, str, int]:
        """Wake the active manual-continue wait for the current run generation."""
        reject = self._terminal_reject()
        if reject is not None:
            return reject
        if self._config is None or not self._points:
            return False, "no point mission loaded", 409
        if self.is_paused():
            return False, "point mission is paused", 409
        obstacle_blocked, obstacle_state = self._write_obstacle_status(self._run_token)
        if obstacle_blocked:
            return False, f"obstacle {obstacle_state} — cannot continue", 409
        if self._status.state == PointMissionState.COMPLETED:
            return False, "point mission already completed", 409
        if self._status.state in {PointMissionState.FAILED, PointMissionState.ABORTING}:
            return False, f"point mission is terminal: {self._status.state.value}", 409
        if self._status.state != PointMissionState.WAITING_FOR_CONTINUE:
            return False, (
                f"point mission is {self._status.state.value}, not waiting for continue"
            ), 409
        run = self._run_token
        if run is None or not self.is_active():
            return False, "point mission is not active", 409
        if self._gps_applies():
            # Close the race where GPS degrades after WAITING_FOR_CONTINUE is
            # displayed but before the wait loop observes and enters pause.
            if ros_node is None:
                try:
                    from main import ros_node
                except Exception:
                    ros_node = None
            if ros_node is None:
                return False, "GPS safety blocks continue: telemetry unavailable", 409
            verdict = self._evaluate_gps_safety(ros_node.get_state())
            self._write_gps_verdict(run, verdict)
            if not verdict.ok:
                return False, f"GPS safety blocks continue: {verdict.reason}", 409
        gate = run.continue_gate
        if gate is None or gate.done():
            return False, "point mission is not awaiting continue", 409
        gate.set_result(True)
        await asyncio.sleep(0)
        return True, "continue accepted", 200

    async def start(self, ros_node, offboard_ctrl, hold_owner=None) -> tuple[bool, str]:
        if self._config is None or not self._points:
            return False, "point mission not loaded"
        await self.cancel_and_drain(
            ros_node, reason="start_replace", offboard_ctrl=offboard_ctrl
        )
        if not self._resolved_points:
            try:
                self.prepare(ros_node.get_state())
            except PlacementError as exc:
                self._status.state = PointMissionState.FAILED
                self._status.last_error = str(exc)
                self._status.last_failure_reason = str(exc)
                self._status.ready = False
                return False, str(exc)
        parent_id = (
            getattr(offboard_ctrl, "running_mission_id", None)
            or getattr(offboard_ctrl, "loaded_mission_id", None)
            or self._status.mission_id
        )
        coordinator = self._operation_coordinator()
        op_gen = coordinator.current_generation() if coordinator is not None else 0
        run = PointMissionRun(
            self._generation,
            self._status.mission_id,
            asyncio.Event(),
            parent_mission_id=str(parent_id or ""),
            operation_generation=op_gen,
        )
        self._run_token = run
        self._terminal_cleanup_reason = None
        self._spray_ever_on = False
        self._write(
            run,
            state=PointMissionState.PREPARING_LEG,
            current_point_index=0,
            last_error="",
            last_failure_reason="",
            ready=True,
            resolved_runtime_frame=LOCAL_NED,
            run_active=True,
            waiting_for_continue=False,
            last_completed_point_index=None,
            next_point_index=0 if self._resolved_points else None,
            arrival_met=False,
            settle_met=False,
            active_dwell_command_id=None,
            terminal_safety_ok=True,
            terminal_safety_reason="",
            parent_mission_id=run.parent_mission_id,
            point_mission_generation=run.generation,
            recovery_required=False,
            terminal_failure_reason="",
            dwell_cancel_result=None,
            spray_off_result=None,
        )
        self._task = asyncio.create_task(
            self._run(run, ros_node, offboard_ctrl, hold_owner),
            name=f"point-{run.mission_id}-{run.generation}",
        )
        return True, "point mission started"

    def prepare(self, state: dict[str, Any]) -> None:
        """Resolve design coordinates before the controller arms or enters OFFBOARD."""
        if self._gps_applies():
            verdict = self._evaluate_gps_safety(state)
            self._write_gps_verdict(None, verdict)
            if not verdict.ok:
                raise PlacementError(verdict.reason)
        self._resolved_points = self._resolve_points(state)
        self._status.resolved_runtime_frame = LOCAL_NED

    def _resolve_points(self, state: dict[str, Any]) -> list[SprayPoint]:
        coords = [(p.north_m, p.east_m) for p in self._points]
        if self._source_frame == LOCAL_NED:
            resolved = coords
        elif self._source_frame == GPS_SURVEYED:
            resolved, _ = resolve_surveyed_points(
                coords, self._origin_gps, state, safety=self._gps_safety_params()
            )
        else:
            raise PlacementError("Point mission frame is missing or ambiguous")
        return [
            SprayPoint(n, e, p.dwell_s, p.source_index, p.mark)
            for p, (n, e) in zip(self._points, resolved)
        ]


















