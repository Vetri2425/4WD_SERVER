"""Verified GPS mission adapter (thin; inherits TargetMissionCoreMixin)."""

from __future__ import annotations

import asyncio
import sys
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal

_SRC = Path(__file__).resolve().parents[2] / "src"
_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from spray_config import PointSprayParams, SprayConfiguration, SprayMode  # noqa: E402

from logging_setup import get_logger
from mission_placement import GPS_SURVEYED, LOCAL_NED, PlacementError
from models import PointTerminalCleanupResult, VerifiedTargetEvent
from point_mission import PointMissionStatus
from target_mission.commands import TargetMissionCommandsMixin
from target_mission.core import TargetMissionCoreMixin
from target_mission.leg_geometry import build_target_leg
from target_mission.types import (
    TargetExecutionMode,
    TargetMissionRun,
    TargetMissionState,
)
from verified_mission.artifact import VerifiedMissionArtifact
from verified_mission.events import get_target_event_journal, utc_ts
from verified_mission.placement import resolve_verified_targets

log = get_logger("server.verified_mission.adapter")

MissionOutcome = Literal["completed", "failed", "stopped", "aborted"]


@dataclass(frozen=True)
class VerifiedTarget:
    """Runtime target row (NED after prepare); satisfies TargetPoint protocol."""

    north_m: float
    east_m: float
    dwell_s: float | None
    mark: bool
    source_index: int
    lat: float
    lon: float
    alt: float = 0.0


class VerifiedMissionOrchestrator(TargetMissionCommandsMixin, TargetMissionCoreMixin):
    """Verified GPS adapter: WGS84 artifact, GPS convert in prepare(), target_event."""

    mission_kind = "verified_gps"
    _DRAIN_TIMEOUT_S = 6.0

    def __init__(self) -> None:
        self._status = PointMissionStatus()
        self._artifact: VerifiedMissionArtifact | None = None
        self._points: list[VerifiedTarget] = []
        self._resolved_points: list[VerifiedTarget] = []
        self._config: SprayConfiguration | None = None
        self._execution_mode = TargetExecutionMode.AUTO
        self._task: asyncio.Task | None = None
        self._run_token: TargetMissionRun | None = None
        self._generation = 0
        self._command_seq = 0
        self._source_frame = GPS_SURVEYED
        self._origin_gps: tuple[float, float] | None = None
        self._obstacle_clear = True
        self._obstacle_last_recv: float | None = None
        self._spray_ever_on = False
        self._gps_recovery_since: float | None = None
        self._gps_fault_count = 0
        self._gps_last_fault_time: float | None = None
        self._log_cb = None
        self._command_lock = asyncio.Lock()
        self._event_lock = threading.RLock()
        self._terminal_cleanup_reason: str | None = None
        self._terminal_outcome: MissionOutcome | None = None

    @property
    def status(self) -> PointMissionStatus:
        return self._status

    def is_active(self) -> bool:
        return self._task is not None and not self._task.done()

    def load_artifact(self, artifact: VerifiedMissionArtifact) -> None:
        if self.is_active():
            raise RuntimeError("active verified mission must be replaced asynchronously")
        settings = artifact.settings or {}
        point_params = PointSprayParams(
            default_dwell_s=float(settings.get("default_dwell_s", 2.0)),
            max_dwell_s=float(settings.get("max_dwell_s", 60.0)),
            arrival_tolerance_m=float(settings.get("arrival_tolerance_m", 0.05)),
            settle_time_s=float(settings.get("settle_time_s", 0.10)),
            leg_timeout_s=float(settings.get("leg_timeout_s", 120.0)),
            settle_speed_mps=float(settings.get("settle_speed_mps", 0.05)),
            leg_trajectory_mode=str(settings.get("leg_trajectory_mode", "densified")),
            leg_spacing_m=float(settings.get("leg_spacing_m", 0.08)),
        )
        config = SprayConfiguration(
            mode=SprayMode.POINT,
            point=point_params,
            revision=int(settings.get("revision", 1)),
        )
        mode_raw = str(settings.get("mode", settings.get("execution_mode", "auto"))).lower()
        execution_mode = (
            TargetExecutionMode.MANUAL
            if mode_raw in {"manual", "dgps mark"}
            else TargetExecutionMode.AUTO
        )
        points = [
            VerifiedTarget(
                north_m=0.0,
                east_m=0.0,
                dwell_s=row.get("dwell_s"),
                mark=bool(row["mark"]),
                source_index=int(row["index"]),
                lat=float(row["lat"]),
                lon=float(row["lon"]),
                alt=float(row.get("alt", 0.0)),
            )
            for row in artifact.waypoints
        ]
        anchor = artifact.waypoints[0]
        self._install(
            artifact.mission_id,
            artifact,
            points,
            config,
            (float(anchor["lat"]), float(anchor["lon"])),
            execution_mode,
        )

    def _install(
        self,
        mission_id: str,
        artifact: VerifiedMissionArtifact,
        points: list[VerifiedTarget],
        config: SprayConfiguration,
        origin_gps: tuple[float, float],
        execution_mode: TargetExecutionMode,
    ) -> None:
        self._generation += 1
        self._artifact = artifact
        self._points = list(points)
        self._resolved_points = []
        self._config = config
        self._execution_mode = execution_mode
        self._origin_gps = origin_gps
        self._run_token = None
        self._terminal_outcome = None
        self._status = PointMissionStatus(
            state=TargetMissionState.IDLE,
            mission_id=mission_id,
            generation=self._generation,
            total_points=len(points),
            ready=True,
            last_transition="loaded",
            source_frame=GPS_SURVEYED,
            point_execution_mode=execution_mode.value,
        )

    def prepare(self, state: dict[str, Any]) -> None:
        """GPS → anchor-relative NED → resolve_surveyed_points (start boundary only)."""
        if self._artifact is None:
            raise PlacementError("verified mission not loaded")
        verdict = self._evaluate_gps_safety(state)
        self._write_gps_verdict(None, verdict)
        if not verdict.ok:
            raise PlacementError(verdict.reason)
        resolved_rows, anchor = resolve_verified_targets(
            self._artifact.waypoints,
            live_state=state,
            safety=self._gps_safety_params(),
        )
        self._origin_gps = anchor
        self._resolved_points = [
            VerifiedTarget(
                north_m=float(row["north_m"]),
                east_m=float(row["east_m"]),
                dwell_s=float(row["dwell_s"]) if row.get("dwell_s") is not None else None,
                mark=bool(row["mark"]),
                source_index=int(row["index"]),
                lat=float(row["lat"]),
                lon=float(row["lon"]),
                alt=float(row.get("alt", 0.0)),
            )
            for row in resolved_rows
        ]
        self._status.resolved_runtime_frame = LOCAL_NED

    def _target_for_index(self, index: int) -> VerifiedTarget | None:
        if 0 <= index < len(self._resolved_points):
            return self._resolved_points[index]
        return None

    def _build_target_leg(
        self,
        state: dict[str, Any],
        point: VerifiedTarget,
        params: PointSprayParams,
    ) -> tuple[list[tuple[float, float]], dict[str, Any]]:
        return build_target_leg(state, point.north_m, point.east_m, params)

    def _build_event(
        self,
        event_type: str,
        *,
        point_index: int | None = None,
        terminal: bool = False,
        mission_outcome: MissionOutcome | None = None,
        reason: str = "",
        message: str = "",
        lat: float | None = None,
        lon: float | None = None,
    ) -> VerifiedTargetEvent:
        resolved_index = (
            point_index if point_index is not None else self._status.current_point_index
        )
        target = self._target_for_index(resolved_index)
        return VerifiedTargetEvent(
            event_id=0,
            target_index=resolved_index,
            mission_id=self._status.mission_id,
            event_type=event_type,  # type: ignore[arg-type]
            timestamp=utc_ts(),
            terminal=terminal,
            mission_outcome=mission_outcome,
            lat=lat if lat is not None else (target.lat if target else None),
            lon=lon if lon is not None else (target.lon if target else None),
            reason=reason or None,
            message=message or reason,
        )

    def _emit_target_event(
        self,
        run: TargetMissionRun | None,
        event_type: str,
        *,
        terminal: bool = False,
        mission_outcome: MissionOutcome | None = None,
        reason: str = "",
        **kwargs: Any,
    ) -> None:
        if run is not None and run.terminal_event_emitted and terminal:
            return
        with self._event_lock:
            event = self._build_event(
                event_type,
                terminal=terminal,
                mission_outcome=mission_outcome,
                reason=reason,
                message=kwargs.get("message", reason),
                point_index=kwargs.get("point_index"),
                lat=kwargs.get("lat"),
                lon=kwargs.get("lon"),
            )
            get_target_event_journal().append(event)
        if run is not None and terminal:
            run.terminal_event_emitted = True
            if mission_outcome is not None:
                self._terminal_outcome = mission_outcome

    def _emit_lifecycle_event(
        self,
        run: TargetMissionRun | None,
        kind: str,
        **kwargs: Any,
    ) -> None:
        mapping = {
            "leg_started": "target_active",
            "arrived": "target_arrived",
            "dwell_started": "target_marking",
            "marked": "target_completed",
            "skipped": "target_skipped",
        }
        event_type = mapping.get(kind)
        if event_type is None:
            return
        if event_type == "target_completed":
            point_index = int(kwargs.get("point_index", self._status.current_point_index))
            if point_index >= max(0, self._status.total_points - 1):
                return
        self._emit_target_event(run, event_type, reason=kwargs.get("reason", ""), **kwargs)

    def _emit_navigation_only_completion(
        self,
        run: TargetMissionRun | None,
        *,
        point_index: int,
        source_index: int,
    ) -> None:
        """Emit one non-terminal ``target_completed`` for a finished mark=false leg."""
        self._emit_target_event(
            run,
            "target_completed",
            point_index=point_index,
        )

    async def _wait_settled(self, run, ros_node, hold_owner, point, params, started) -> str:
        self._emit_target_event(
            run,
            "target_settling",
            point_index=self._status.current_point_index,
            source_index=point.source_index,
        )
        return await super()._wait_settled(
            run, ros_node, hold_owner, point, params, started
        )

    def _mission_outcome_for_reason(self, reason: str, *, recovery_required: bool) -> MissionOutcome:
        if reason == "normal_completion" and not recovery_required:
            return "completed"
        if reason in {"operator_stop", "restart_stop_first"}:
            return "stopped"
        if reason in {"operator_abort", "emergency_stop"}:
            return "aborted"
        return "failed"

    def _terminal_event_for_reason(self, reason: str) -> str:
        outcome = self._mission_outcome_for_reason(reason, recovery_required=False)
        return {
            "completed": "target_completed",
            "failed": "target_failed",
            "stopped": "target_stopped",
            "aborted": "target_aborted",
        }[outcome]

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
        terminal_state: TargetMissionState,
        operation_token,
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
            preserve_hold = terminal_state == TargetMissionState.FAILED_GPS_SAFETY
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
                self._status.recovery_required = recovery_required
            self._run_token = None
            self._terminal_cleanup_reason = reason
            terminal_event_emitted = False
            if run is None or not run.terminal_event_emitted:
                outcome = self._mission_outcome_for_reason(
                    reason, recovery_required=recovery_required
                )
                event_type = self._terminal_event_for_reason(reason)
                if recovery_required:
                    outcome = "failed"
                    event_type = "target_failed"
                terminal_point_index = (
                    self._status.last_completed_point_index
                    if self._status.last_completed_point_index is not None
                    else max(0, self._status.total_points - 1)
                )
                if (
                    reason == "normal_completion"
                    and not recovery_required
                    and self._status.last_skipped_point_index == terminal_point_index
                ):
                    event_type = "target_skipped"
                self._emit_target_event(
                    run,
                    event_type,
                    terminal=True,
                    mission_outcome=outcome,
                    reason=reason,
                    message=terminal_failure or reason,
                    point_index=terminal_point_index,
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

    async def start(
        self, ros_node, offboard_ctrl, hold_owner=None
    ) -> tuple[bool, str]:
        if self._config is None or not self._points:
            return False, "verified mission not loaded"
        if not self._resolved_points:
            try:
                self.prepare(ros_node.get_state())
            except PlacementError as exc:
                self._status.state = TargetMissionState.FAILED
                self._status.last_error = str(exc)
                self._status.ready = False
                return False, str(exc)
        parent_id = (
            getattr(offboard_ctrl, "running_mission_id", None)
            or getattr(offboard_ctrl, "loaded_mission_id", None)
            or self._status.mission_id
        )
        coordinator = self._operation_coordinator()
        op_gen = coordinator.current_generation() if coordinator is not None else 0
        run = TargetMissionRun(
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
            state=TargetMissionState.PREPARING_LEG,
            current_point_index=0,
            ready=True,
            resolved_runtime_frame=LOCAL_NED,
            run_active=True,
            parent_mission_id=run.parent_mission_id,
            point_mission_generation=run.generation,
        )
        self._task = asyncio.create_task(
            self._run(run, ros_node, offboard_ctrl, hold_owner),
            name=f"verified-mission-{self._status.mission_id}",
        )
        return True, "started"

    def _refresh_resolved_points(self, state: dict[str, Any]) -> None:
        self.prepare(state)

    async def clear_mission(
        self, ros_node, *, reason: str = "cleared", offboard_ctrl=None
    ) -> None:
        await super().clear_mission(
            ros_node, reason=reason, offboard_ctrl=offboard_ctrl
        )
        self._artifact = None
        self._terminal_outcome = None

    async def stop(self, ros_node, offboard_ctrl=None, hold_owner=None) -> None:
        from mission_ops import MissionOperation, MissionOperationCoordinator

        coordinator = self._operation_coordinator() or MissionOperationCoordinator()
        token = await coordinator.begin(MissionOperation.STOP, timeout_s=0.5)
        try:
            await self.terminal_cleanup(
                ros_node,
                hold_owner,
                reason="operator_stop",
                terminal_state=TargetMissionState.ABORTING,
                operation_token=token,
                offboard_ctrl=offboard_ctrl,
                require_spray_confirm=True,
            )
            if offboard_ctrl is not None and hasattr(offboard_ctrl, "stop_async"):
                await offboard_ctrl.stop_async()
        finally:
            await coordinator.finish(token)
