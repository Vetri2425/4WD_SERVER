"""Point-mission orchestrator state machine (async, non-blocking)."""

from __future__ import annotations

import asyncio
import sys
import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, Callable, Literal

_SRC = Path(__file__).resolve().parents[1] / "src"
_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from point_ingest import SprayPoint, points_from_staged_dict  # noqa: E402
from point_leg_trajectory import (  # noqa: E402
    PointLegTrajectoryMode,
    build_point_leg_path,
    leg_length_m,
    predict_rpp_conditioning,
)
from spray_config import (  # noqa: E402
    GpsSurveyedSafetyParams,
    ObstacleSafetyParams,
    PointSprayParams,
    SprayConfiguration,
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
from mission_placement import GPS_SURVEYED, LOCAL_NED, PlacementError, resolve_surveyed_points
from models import MissionState, PointMissionEvent, PointTerminalCleanupResult
from point_events import get_point_event_journal, utc_ts
from spray_safety import force_spray_off_confirmed

try:
    from mission_ops import MissionOperationToken
except ImportError:  # pragma: no cover
    MissionOperationToken = Any  # type: ignore[misc,assignment]

log = get_logger("server.point_mission")


class PointMissionRunFailure(RuntimeError):
    """Run-loop failure that must route through terminal_cleanup()."""

    def __init__(
        self,
        message: str,
        *,
        cleanup_reason: Literal["dwell_fault", "start_failure", "completion_degraded"] = "dwell_fault",
        terminal_state: "PointMissionState | None" = None,
    ) -> None:
        super().__init__(message)
        self.cleanup_reason = cleanup_reason
        self.terminal_state = terminal_state


_PARENT_ABORT_REASONS = frozenset(
    {
        "abort",
        "cancelled",
        "non_point_load",
        "failed",
        "gps_fail",
        "dwell_fault",
        "schema_fault",
        "runtime_restart",
        "completion_degraded",
    }
)

_DWELL_POLL_REQUIRED_FIELDS = (
    "dwell_command_id",
    "dwell_mission_id",
    "dwell_point_index",
    "commanded_on",
    "confirmed_off",
    "off_acknowledged",
    "active_dwell",
    "status_stale",
)


class SprayRuntimeSchemaError(RuntimeError):
    """Raised when spray runtime status is missing required dwell fields."""


class PointMissionState(str, Enum):
    IDLE = "idle"
    PREPARING_LEG = "preparing_leg"
    NAVIGATING = "navigating"
    SETTLING = "settling"
    DWELLING = "dwelling"
    WAITING_FOR_CONTINUE = "waiting_for_continue"
    ADVANCING = "advancing"
    PAUSING = "pausing"
    PAUSED_HOLD = "paused_hold"
    RESUMING = "resuming"
    PAUSED_OBSTACLE = "paused_obstacle"
    OBSTACLE_DURING_DWELL = "obstacle_during_dwell"
    PAUSED_GPS_SAFETY = "paused_gps_safety"
    FAILED_GPS_SAFETY = "failed_gps_safety"
    GPS_DURING_DWELL = "gps_during_dwell"
    COMPLETED = "completed"
    ABORTING = "aborting"
    FAILED = "failed"


OBSTACLE_NOT_CONFIGURED = "not_configured"
OBSTACLE_OK = "ok"
OBSTACLE_MISSING = "missing"
OBSTACLE_STALE = "stale"
OBSTACLE_BLOCKED = "blocked"


PAUSED_STATES = frozenset(
    {
        PointMissionState.PAUSED_HOLD,
        PointMissionState.PAUSED_OBSTACLE,
        PointMissionState.OBSTACLE_DURING_DWELL,
        PointMissionState.PAUSED_GPS_SAFETY,
        PointMissionState.GPS_DURING_DWELL,
    }
)

TERMINAL_POINT_STATES = frozenset(
    {
        PointMissionState.COMPLETED,
        PointMissionState.FAILED,
        PointMissionState.ABORTING,
    }
)

_TERMINAL_REASON_PRIORITY = {
    "emergency_stop": 100,
    "operator_abort": 90,
    "restart_stop_first": 85,
    "operator_stop": 80,
    "dwell_fault": 75,
    "completion_degraded": 70,
    "start_failure": 65,
    "normal_completion": 60,
}

_SKIP_ACCEPTED_STATES = frozenset(
    {
        PointMissionState.PREPARING_LEG,
        PointMissionState.NAVIGATING,
        PointMissionState.SETTLING,
        PointMissionState.DWELLING,
        PointMissionState.PAUSED_HOLD,
        PointMissionState.PAUSED_OBSTACLE,
        PointMissionState.OBSTACLE_DURING_DWELL,
    }
)


class PointExecutionMode(str, Enum):
    AUTO = "auto"
    MANUAL = "manual"

    @classmethod
    def parse(cls, value: Any) -> PointExecutionMode:
        text = str(value or cls.AUTO.value).strip().lower()
        try:
            return cls(text)
        except ValueError as exc:
            raise PlacementError(
                f"unsupported point_execution_mode {value!r}; expected "
                f"{cls.AUTO.value} or {cls.MANUAL.value}"
            ) from exc


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


@dataclass
class PointMissionRun:
    generation: int
    mission_id: str
    cancel_event: asyncio.Event
    parent_mission_id: str = ""
    continue_gate: asyncio.Future | None = None
    resume_gate: asyncio.Future | None = None
    pause_requested: bool = False
    active_dwell_command_id: int | None = None
    active_dwell_command_revision: int | None = None
    active_dwell_configuration_revision: int | None = None
    active_dwell_point_index: int | None = None
    active_dwell_source_index: int | None = None
    dwell_revision_invalid: bool = False
    spray_runtime_fingerprint: tuple[int, int, float] | None = None
    operation_generation: int = 0
    terminal_cleanup_started: bool = False
    terminal_event_emitted: bool = False
    skip_requested: bool = False
    skip_request_id: int | None = None
    skip_reason: str = ""


class PointMissionOrchestrator:
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

    def set_logger(self, cb: Callable[[str, str], None]) -> None:
        self._log_cb = cb

    def _record(self, level: str, message: str) -> None:
        if self._log_cb is not None:
            self._log_cb(level, message)
        getattr(log, level if level in ("info", "warning", "error", "debug") else "info")(message)

    @property
    def status(self) -> PointMissionStatus:
        return self._status

    def is_active(self) -> bool:
        return self._task is not None and not self._task.done()

    def is_paused(self) -> bool:
        return self._status.state in PAUSED_STATES

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
        run: PointMissionRun,
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

    def _invalidate_dwell_identity(self, run: PointMissionRun | None) -> None:
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
        run: PointMissionRun,
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
        run: PointMissionRun | None,
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

    def _write_obstacle_status(self, run: PointMissionRun | None) -> tuple[bool, str]:
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

    def _write_gps_verdict(self, run: PointMissionRun | None, verdict: GpsSafetyVerdict) -> None:
        if not self._gps_applies():
            self._write(run, **local_ned_gps_status())
            return
        self._write(run, **verdict.as_status_dict())

    def _point_params(self) -> PointSprayParams:
        if self._config is not None:
            return self._config.point
        return PointSprayParams()

    def _build_point_leg(
        self,
        state: dict[str, Any],
        point: SprayPoint,
        params: PointSprayParams,
    ) -> tuple[list[tuple[float, float]], dict[str, Any]]:
        start = (float(state["pos_n"]), float(state["pos_e"]))
        end = (point.north_m, point.east_m)
        mode = PointLegTrajectoryMode.parse(params.leg_trajectory_mode)
        published = build_point_leg_path(
            start,
            end,
            mode=mode,
            spacing_m=params.leg_spacing_m,
        )
        profile, conditioned = predict_rpp_conditioning(
            published,
            runtime_entry=True,
            resample_spacing_m=params.leg_spacing_m,
        )
        return published, {
            "point_leg_trajectory_mode": mode.value,
            "point_leg_spacing_m": params.leg_spacing_m,
            "point_leg_published_count": len(published),
            "point_leg_conditioned_count": len(conditioned),
            "active_trajectory_mode": profile,
            "point_leg_length_m": leg_length_m(start, end),
        }

    def _write_leg_diagnostics(
        self, run: PointMissionRun | None, diag: dict[str, Any]
    ) -> None:
        self._write(run, **diag)

    async def _handle_hold_drift(
        self,
        run: PointMissionRun,
        ros_node,
        hold_owner,
        point: SprayPoint,
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
            dwell_cancelled=phase == PointMissionState.DWELLING.value
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
            phase == PointMissionState.DWELLING.value or bool(self._status.active_dwell)
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
        run: PointMissionRun,
        ros_node,
        hold_owner,
        point: SprayPoint,
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

    def _merge_hold_status(self, run: PointMissionRun | None, hold_owner, ros_node) -> None:
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

    def _is_current(self, run: PointMissionRun) -> bool:
        return self._run_token is run and self._generation == run.generation

    def _operation_coordinator(self):
        try:
            from main import operation_coordinator

            return operation_coordinator
        except Exception:
            return None

    def _check_operation_generation(self, run: PointMissionRun) -> None:
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

    def _write(self, run: PointMissionRun | None, **changes: Any) -> None:
        if run is not None and not self._is_current(run):
            return
        for key, value in changes.items():
            setattr(self._status, key, value)

    def _distance_to_point(self, state: dict[str, Any], point: SprayPoint) -> float:
        return (
            (float(state.get("pos_n", 0.0)) - point.north_m) ** 2
            + (float(state.get("pos_e", 0.0)) - point.east_m) ** 2
        ) ** 0.5

    def _update_live_diagnostics(
        self,
        run: PointMissionRun,
        ros_node,
        point: SprayPoint,
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

    async def _skip_cycle(
        self,
        run: PointMissionRun,
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
            PointMissionState.DWELLING,
            PointMissionState.OBSTACLE_DURING_DWELL,
            PointMissionState.GPS_DURING_DWELL,
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
        self._emit_point_event(
            run,
            "point_skipped",
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
            state=PointMissionState.ADVANCING if not is_last else PointMissionState.ADVANCING,
            last_transition=f"skipped:{index}",
            next_point_index=None if is_last else index + 1,
        )
        return True, "skipped"

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

    def _resume_phase_after_pause(self, phase: str) -> str:
        if phase == PointMissionState.WAITING_FOR_CONTINUE.value:
            return "waiting_for_continue"
        return "navigating"

    async def _gps_fail_cycle(
        self,
        run: PointMissionRun,
        ros_node,
        hold_owner,
        point: SprayPoint,
        phase: str,
        verdict: GpsSafetyVerdict,
    ) -> None:
        during_dwell = phase == PointMissionState.DWELLING.value
        dwell_cancelled = during_dwell or bool(self._status.active_dwell)
        self._write(
            run,
            state=PointMissionState.FAILED_GPS_SAFETY,
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
        run: PointMissionRun,
        ros_node,
        hold_owner,
        point: SprayPoint,
        phase: str,
        *,
        pause_reason: str,
    ) -> None:
        during_dwell = phase == PointMissionState.DWELLING.value
        dwell_cancelled = during_dwell or bool(self._status.active_dwell)
        transient = PointMissionState.PAUSING
        if pause_reason == "obstacle" and during_dwell:
            transient = PointMissionState.OBSTACLE_DURING_DWELL
        elif pause_reason == "gps_safety" and during_dwell:
            transient = PointMissionState.GPS_DURING_DWELL
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
            "operator": PointMissionState.PAUSED_HOLD,
            "obstacle": PointMissionState.PAUSED_OBSTACLE,
            "gps_safety": PointMissionState.PAUSED_GPS_SAFETY,
        }[pause_reason]
        run.resume_gate = asyncio.get_running_loop().create_future()
        self._write(
            run,
            state=target,
            resume_available=True,
            last_transition=f"paused:{phase}",
        )
        self._emit_point_event(
            run,
            "point_paused",
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
        self._write(run, state=PointMissionState.RESUMING, last_transition="resuming", resume_available=False)
        self._emit_point_event(
            run,
            "point_resumed",
            point_index=self._status.current_point_index,
            source_index=point.source_index,
            reason=pause_reason,
        )
        if hold_owner is not None:
            hold_owner.deactivate(ros_node)
            self._merge_hold_status(run, hold_owner, ros_node)

    async def _poll_interruptions(
        self,
        run: PointMissionRun,
        ros_node,
        hold_owner,
        point: SprayPoint,
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

    async def _run(self, run: PointMissionRun, ros_node, offboard_ctrl, hold_owner) -> None:
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
                if is_last:
                    break
                if self._execution_mode == PointExecutionMode.MANUAL:
                    await self._wait_for_continue(run, ros_node, hold_owner, point, index)
                else:
                    self._write(
                        run,
                        state=PointMissionState.ADVANCING,
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
                        terminal_state=PointMissionState.FAILED,
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
                    terminal_state=PointMissionState.COMPLETED,
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
                    state=PointMissionState.FAILED,
                    last_error="cancelled",
                    last_failure_reason="cancelled",
                    last_transition="operator_abort",
                    ready=False,
                    run_active=False,
                    waiting_for_continue=False,
                )
            raise
        except PointMissionRunFailure as exc:
            terminal = exc.terminal_state or (
                PointMissionState.FAILED_GPS_SAFETY
                if self._status.state == PointMissionState.FAILED_GPS_SAFETY
                else PointMissionState.FAILED
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
                terminal_state=PointMissionState.FAILED,
                error_message=str(exc),
            )
            self._record("error", f"point mission spray schema fault: {exc}")
        except Exception as exc:
            terminal = (
                PointMissionState.FAILED_GPS_SAFETY
                if self._status.state == PointMissionState.FAILED_GPS_SAFETY
                else PointMissionState.FAILED
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

    def _check_cancel(self, run: PointMissionRun) -> None:
        if (
            run.cancel_event.is_set()
            or run.terminal_cleanup_started
            or not self._is_current(run)
        ):
            raise asyncio.CancelledError()
        self._check_operation_generation(run)

    async def _wait_for_continue(
        self, run: PointMissionRun, ros_node, hold_owner, point: SprayPoint, completed_index: int
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
            state=PointMissionState.WAITING_FOR_CONTINUE,
            waiting_for_continue=True,
            last_transition=f"waiting_for_continue:{completed_index}",
        )
        self._emit_point_event(
            run,
            "point_waiting_for_continue",
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
                    PointMissionState.WAITING_FOR_CONTINUE.value,
                )
                if resume == "waiting_for_continue":
                    break
                drift = await self._poll_hold_drift(
                    run,
                    ros_node,
                    hold_owner,
                    point,
                    params,
                    PointMissionState.WAITING_FOR_CONTINUE.value,
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
            state=PointMissionState.ADVANCING,
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
        published, diag = self._build_point_leg(state, point, params)
        self._write_leg_diagnostics(run, diag)
        ros_node.publish_path(
            published,
            spray_flags=[False] * len(published),
            runtime_entry=True,
        )
        self._emit_point_event(
            run,
            "point_leg_started",
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
                self._write(run, state=PointMissionState.PREPARING_LEG, last_transition=f"preparing_leg:{index}")
                await ros_node.cancel_spray_dwell_async()
                await self._publish_fresh_leg(run, ros_node, point, params)
                started = time.monotonic()
                self._write(run, state=PointMissionState.NAVIGATING, last_transition=f"navigating:{index}")
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
                self._write(run, state=PointMissionState.SETTLING, last_transition=f"settling:{index}")
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
                self._write(run, state=PointMissionState.DWELLING, last_transition=f"dwelling:{index}")
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
                self._emit_point_event(
                    run,
                    "point_dwell_started",
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
                run, ros_node, hold_owner, point, PointMissionState.NAVIGATING.value
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
                self._emit_point_event(
                    run,
                    "point_arrived",
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
                run, ros_node, hold_owner, point, PointMissionState.SETTLING.value
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
        run: PointMissionRun,
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
        run: PointMissionRun,
        ros_node,
        offboard_ctrl,
        *,
        reason: str,
    ) -> None:
        raise PointMissionRunFailure(reason, cleanup_reason="dwell_fault")

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
                run, ros_node, hold_owner, point, PointMissionState.DWELLING.value
            )
            if resume is not None:
                return resume
            drift = await self._poll_hold_drift(
                run,
                ros_node,
                hold_owner,
                point,
                params,
                PointMissionState.DWELLING.value,
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
                    self._emit_point_event(
                        run,
                        "point_marked",
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
