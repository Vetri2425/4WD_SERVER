"""Mission-kind routing helpers for shared lifecycle services."""

from __future__ import annotations

from typing import Any, Literal

MissionKind = Literal["none", "path", "point_staged", "verified_gps", "legacy"]


def loaded_mission_kind(offboard_ctrl: Any | None) -> MissionKind:
    if offboard_ctrl is None:
        return "none"
    kind = getattr(offboard_ctrl, "loaded_mission_kind", None)
    if kind in {"none", "path", "point_staged", "verified_gps", "legacy"}:
        return kind  # type: ignore[return-value]
    return "none"


def active_target_orchestrator(
    *,
    offboard_ctrl: Any | None,
    point_mission: Any | None,
    verified_mission: Any | None,
) -> Any | None:
    """Return the target-mission orchestrator for the resident mission kind."""
    if loaded_mission_kind(offboard_ctrl) == "verified_gps":
        return verified_mission
    if offboard_ctrl is not None and getattr(offboard_ctrl, "spray_mode", "") == "point":
        return point_mission
    return point_mission


def is_verified_prefix(mission_id: str | None, *, prefix: str) -> bool:
    return bool(mission_id and str(mission_id).startswith(prefix))