"""Load verified GPS artifacts into the offboard controller."""

from __future__ import annotations

from typing import Any

from logging_setup import get_logger
from models import MissionState
from verified_mission.artifact import VerifiedMissionArtifact
from verified_mission.store import VerifiedMissionStore

log = get_logger("server.verified_mission.loader")


def derive_verified_mission_state(
    artifact: VerifiedMissionArtifact,
    *,
    offboard_ctrl: Any | None,
    orchestrator: Any | None = None,
) -> str:
    """Honest mission state from live controller + orchestrator, not file alone."""
    if offboard_ctrl is None:
        return "stored"

    loaded_id = getattr(offboard_ctrl, "loaded_mission_id", None)
    running_id = getattr(offboard_ctrl, "running_mission_id", None)
    loaded_kind = getattr(offboard_ctrl, "loaded_mission_kind", "none")
    parent_state = getattr(offboard_ctrl, "state", None)

    if loaded_id != artifact.mission_id or loaded_kind != "verified_gps":
        return "stored"

    if orchestrator is not None and getattr(orchestrator, "is_active", lambda: False)():
        return "running"

    if parent_state == MissionState.RUNNING:
        return "running"

    if parent_state == MissionState.COMPLETED:
        terminal = None
        if orchestrator is not None:
            terminal = getattr(orchestrator, "_terminal_outcome", None)
        return "completed" if terminal == "completed" else "failed"

    if parent_state in {MissionState.ABORTED, MissionState.ERROR}:
        return "failed"

    if loaded_id == artifact.mission_id:
        return "loaded"

    return "stored"


def load_verified_artifact_into_controller(
    artifact: VerifiedMissionArtifact,
    *,
    offboard_ctrl: Any,
    orchestrator: Any | None = None,
    store: VerifiedMissionStore | None = None,
) -> dict[str, Any]:
    """Stage verified artifact in controller without GPS conversion (prepare at start)."""
    if offboard_ctrl is None:
        raise RuntimeError("offboard controller unavailable")

    load_path = getattr(offboard_ctrl, "load_path", None)
    if load_path is None:
        raise RuntimeError("offboard controller missing load_path")

    # Shell path: single hold point; real legs published per-target at runtime.
    shell = [(0.0, 0.0)]
    anchor = artifact.waypoints[0]
    load_path(
        shell,
        spray_flags=[False],
        mission_id=artifact.mission_id,
        placement_mode="GPS_SURVEYED",
        origin_gps=(float(anchor["lat"]), float(anchor["lon"])),
        is_staged=True,
        allow_replace_protected=True,
        mission_kind="verified_gps",
        spray_mode="point",
        metadata={
            "mission_name": artifact.mission_name,
            "total_targets": artifact.total_targets,
            "verified_waypoints": artifact.waypoints,
            "verified_settings": artifact.settings,
        },
    )

    if orchestrator is not None and hasattr(orchestrator, "load_artifact"):
        orchestrator.load_artifact(artifact)

    log.info("loaded verified mission %s (%d targets)", artifact.mission_id, artifact.total_targets)
    return {
        "success": True,
        "mission_id": artifact.mission_id,
        "total_targets": artifact.total_targets,
        "state": derive_verified_mission_state(artifact, offboard_ctrl=offboard_ctrl, orchestrator=orchestrator),
    }


def ensure_verified_mission_loaded(
    mission_id: str,
    *,
    offboard_ctrl: Any,
    orchestrator: Any | None = None,
    store: VerifiedMissionStore | None = None,
) -> VerifiedMissionArtifact:
    """Load from store when mission_id is not already resident."""
    mission_store = store or VerifiedMissionStore()
    loaded_id = getattr(offboard_ctrl, "loaded_mission_id", None)
    loaded_kind = getattr(offboard_ctrl, "loaded_mission_kind", "none")
    if loaded_id == mission_id and loaded_kind == "verified_gps":
        return mission_store.load(mission_id)

    artifact = mission_store.load(mission_id)
    load_verified_artifact_into_controller(
        artifact,
        offboard_ctrl=offboard_ctrl,
        orchestrator=orchestrator,
        store=mission_store,
    )
    return artifact