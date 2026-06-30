"""Verified GPS mission upload and status endpoints."""

from __future__ import annotations

from fastapi import APIRouter, Depends, HTTPException

from auth import require_token
from config import VERIFIED_MISSION_ID_PREFIX
from logging_setup import get_logger
from models import (
    GetVerifiedMissionResponse,
    UploadVerifiedMissionRequest,
    UploadVerifiedMissionResponse,
)
from verified_mission.ingest import ingest_verified_waypoints, validated_waypoints_as_dict
from verified_mission.loader import derive_verified_mission_state
from verified_mission.store import VerifiedMissionStore

log = get_logger("server.routes.verified_mission")

router = APIRouter(prefix="/mission", tags=["mission"])


def _store() -> VerifiedMissionStore:
    from main import verified_mission_store

    return verified_mission_store or VerifiedMissionStore()


def _is_resident(mission_id: str) -> bool:
    from main import offboard_ctrl, verified_mission

    if offboard_ctrl is None:
        return False
    loaded_id = getattr(offboard_ctrl, "loaded_mission_id", None)
    running_id = getattr(offboard_ctrl, "running_mission_id", None)
    loaded_kind = getattr(offboard_ctrl, "loaded_mission_kind", "none")
    if loaded_kind != "verified_gps":
        return False
    if mission_id in {loaded_id, running_id}:
        return True
    if verified_mission is not None and verified_mission.is_active():
        if getattr(verified_mission.status, "mission_id", None) == mission_id:
            return True
    return False


@router.post(
    "/verified-waypoints",
    response_model=UploadVerifiedMissionResponse,
    dependencies=[Depends(require_token)],
)
async def upload_verified_waypoints(req: UploadVerifiedMissionRequest):
    """Upload ordered WGS84 targets; reject atomically on any validation error."""
    try:
        validated = ingest_verified_waypoints(
            [wp.model_dump() for wp in req.waypoints],
            settings=req.settings,
        )
    except ValueError as exc:
        raise HTTPException(422, str(exc)) from exc

    store = _store()
    artifact = store.save(
        mission_name=req.mission_name,
        waypoints=validated_waypoints_as_dict(validated),
        settings=req.settings,
    )
    store.prune_expired(is_resident=_is_resident)
    log.info(
        "verified mission uploaded id=%s targets=%d",
        artifact.mission_id,
        artifact.total_targets,
    )
    return UploadVerifiedMissionResponse(
        success=True,
        mission_id=artifact.mission_id,
        total_targets=artifact.total_targets,
        mission_name=artifact.mission_name,
        message="verified mission stored",
    )


@router.get(
    "/verified/{mission_id}",
    response_model=GetVerifiedMissionResponse,
    dependencies=[Depends(require_token)],
)
async def get_verified_mission(mission_id: str):
    """Round-trip WGS84 artifact with honest live state (not file existence alone)."""
    if not mission_id.startswith(VERIFIED_MISSION_ID_PREFIX):
        raise HTTPException(400, f"mission_id must start with {VERIFIED_MISSION_ID_PREFIX!r}")

    store = _store()
    try:
        artifact = store.load(mission_id)
    except FileNotFoundError as exc:
        raise HTTPException(404, str(exc)) from exc

    from main import offboard_ctrl, verified_mission

    state = derive_verified_mission_state(
        artifact,
        offboard_ctrl=offboard_ctrl,
        orchestrator=verified_mission,
    )
    return GetVerifiedMissionResponse(
        mission_id=artifact.mission_id,
        mission_name=artifact.mission_name,
        total_targets=artifact.total_targets,
        waypoints=artifact.waypoints,
        created_at=artifact.created_at,
        state=state,  # type: ignore[arg-type]
    )