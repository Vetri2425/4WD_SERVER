"""Shared target-mission types and constants."""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from enum import Enum
from typing import Literal

from mission_placement import PlacementError


class TargetMissionRunFailure(RuntimeError):
    """Run-loop failure that must route through terminal_cleanup()."""

    def __init__(
        self,
        message: str,
        *,
        cleanup_reason: Literal[
            "dwell_fault", "start_failure", "completion_degraded"
        ] = "dwell_fault",
        terminal_state: "TargetMissionState | None" = None,
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


class TargetMissionState(str, Enum):
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
        TargetMissionState.PAUSED_HOLD,
        TargetMissionState.PAUSED_OBSTACLE,
        TargetMissionState.OBSTACLE_DURING_DWELL,
        TargetMissionState.PAUSED_GPS_SAFETY,
        TargetMissionState.GPS_DURING_DWELL,
    }
)

TERMINAL_POINT_STATES = frozenset(
    {
        TargetMissionState.COMPLETED,
        TargetMissionState.FAILED,
        TargetMissionState.ABORTING,
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
        TargetMissionState.PREPARING_LEG,
        TargetMissionState.NAVIGATING,
        TargetMissionState.SETTLING,
        TargetMissionState.DWELLING,
        TargetMissionState.PAUSED_HOLD,
        TargetMissionState.PAUSED_OBSTACLE,
        TargetMissionState.OBSTACLE_DURING_DWELL,
    }
)


class TargetExecutionMode(str, Enum):
    AUTO = "auto"
    MANUAL = "manual"

    @classmethod
    def parse(cls, value: Any) -> TargetExecutionMode:
        text = str(value or cls.AUTO.value).strip().lower()
        try:
            return cls(text)
        except ValueError as exc:
            raise PlacementError(
                f"unsupported point_execution_mode {value!r}; expected "
                f"{cls.AUTO.value} or {cls.MANUAL.value}"
            ) from exc


@dataclass
class TargetMissionRun:
    generation: int
    mission_id: str
    cancel_event: asyncio.Event
    parent_mission_id: str = ""
    continue_gate: Any | None = None
    resume_gate: Any | None = None
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