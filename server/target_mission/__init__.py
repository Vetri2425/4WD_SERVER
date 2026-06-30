"""Shared target-mission state machine for point and future target missions."""

from target_mission.core import TargetMissionCoreMixin, TargetPoint
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
)

__all__ = [
    "OBSTACLE_BLOCKED",
    "OBSTACLE_MISSING",
    "OBSTACLE_NOT_CONFIGURED",
    "OBSTACLE_OK",
    "OBSTACLE_STALE",
    "PAUSED_STATES",
    "TERMINAL_POINT_STATES",
    "SprayRuntimeSchemaError",
    "TargetExecutionMode",
    "TargetMissionCoreMixin",
    "TargetMissionRun",
    "TargetMissionRunFailure",
    "TargetMissionState",
    "TargetPoint",
    "build_target_leg",
]