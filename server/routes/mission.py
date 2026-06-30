"""Mission endpoints (auth-protected).

POST /api/mission/load    — load path by name or file
POST /api/mission/start   — arm → OFFBOARD → publish path
POST /api/mission/stop    — publish stop-path (stay armed)
POST /api/mission/abort   — hard abort (stop-path + MANUAL + disarm)
POST /api/mission/clear   — clear the resident mission (in-memory only)
POST /api/mission/pause   — resumable OFFBOARD hold (point missions)
POST /api/mission/resume  — resume paused point mission from live pose
POST /api/mission/obstacle — set obstacle clear/blocked hook state
POST /api/mission/point/continue — advance manual point mission after operator approval
POST /api/mission/point/skip     — skip the active point leg
POST /api/mission/restart        — reset resident mission (optional stop-first / start)
GET  /api/mission/point/status   — Point Mode runtime diagnostics
GET  /api/mission/point/events   — bounded Point event journal
GET  /api/mission/status  — current state + RPP snapshot
"""
from __future__ import annotations

from fastapi import APIRouter, Depends, HTTPException, Query

from auth import require_operator_or_machine, require_token
from config import RPP_STALE, RPP_STATE_NAMES
from mission_loading import (
    MissionLoadConflict,
    load_path_for_controller,
    start_mission_for_controller,
)
from mission_placement import PlacementError
from mission_services import (
    MissionServiceError,
    build_service_context,
    continue_point_service,
    pause_point_service,
    restart_mission_service,
    resume_point_service,
    set_point_obstacle_service,
    skip_point_service,
    abort_mission_service,
    stop_mission_service,
)
from models import (
    LoadedPathResponse,
    MissionClearResponse,
    MissionLoadRequest,
    MissionRestartRequest,
    MissionRestartResponse,
    MissionResumeRequest,
    MissionStartRequest,
    MissionStatus,
    ObstacleStatusRequest,
    ObstacleStatusResponse,
    PointContinueResponse,
    PointEventHistoryResponse,
    PointMissionEvent,
    PointMissionStatusResponse,
    PointPauseResponse,
    PointResumeResponse,
    PointSkipRequest,
    PointSkipResponse,
)

router = APIRouter(prefix="/mission", tags=["mission"])


def _service_context(transport: str = "rest"):
    from main import (
        hold_owner,
        mission_capture,
        offboard_ctrl,
        operation_coordinator,
        path_mgr,
        point_mission,
        ros_node,
    )
    from mission_ops import MissionOperationCoordinator

    coordinator = operation_coordinator or MissionOperationCoordinator()
    return build_service_context(
        offboard_ctrl=offboard_ctrl,
        point_mission=point_mission,
        ros_node=ros_node,
        hold_owner=hold_owner,
        path_mgr=path_mgr,
        mission_capture=mission_capture,
        transport=transport,  # type: ignore[arg-type]
        operation_coordinator=coordinator,
    )


def _http_from_service_error(exc: MissionServiceError) -> HTTPException:
    return HTTPException(exc.status_code, exc.message)


@router.get(
    "/loaded-path",
    response_model=LoadedPathResponse,
    dependencies=[Depends(require_operator_or_machine("mission:loaded-path"))],
)
async def loaded_path():
    """Stage 10 — confirm the coordinates currently resident in the controller."""
    from main import offboard_ctrl
    if offboard_ctrl is None:
        return LoadedPathResponse(loaded=False, state="idle")
    return LoadedPathResponse(**offboard_ctrl.loaded_path_summary())


def _merge_point_status() -> dict:
    from main import hold_owner, point_mission, ros_node

    if point_mission is None:
        return {}
    payload = point_mission.status.as_dict()
    if hold_owner is not None:
        hold = hold_owner.as_dict(ros_node)
        payload.update(
            {
                "setpoint_source": hold["setpoint_source"],
                "hold_active": hold["hold_active"],
                "hold_north_m": hold["hold_north_m"],
                "hold_east_m": hold["hold_east_m"],
                "hold_heading_ned_rad": hold["hold_heading_ned_rad"],
                "hold_error_m": hold["hold_error_m"],
            }
        )
    return payload


@router.post("/clear", response_model=MissionClearResponse, dependencies=[Depends(require_token)])
async def clear_mission():
    """Clear an idle/completed resident mission without deleting artifacts."""
    from main import hold_owner, offboard_ctrl, point_mission, ros_node
    from offboard_controller import MissionClearConflict

    if offboard_ctrl is None:
        raise HTTPException(503, "Controller not ready")
    if hold_owner is not None:
        hold_owner.deactivate(ros_node)
    if point_mission is not None:
        await point_mission.clear_mission(
            ros_node, reason="cleared", offboard_ctrl=offboard_ctrl
        )
    try:
        status = await offboard_ctrl.clear_mission_async()
    except MissionClearConflict as exc:
        raise HTTPException(409, str(exc))
    return MissionClearResponse(
        cleared=True,
        status=LoadedPathResponse(**status),
    )


@router.post("/load", dependencies=[Depends(require_token)])
async def load_mission(req: MissionLoadRequest):
    from main import offboard_ctrl, path_mgr
    if offboard_ctrl is None:
        raise HTTPException(503, "Controller not ready")
    name = req.path_name or req.mission_file
    if not name:
        raise HTTPException(400, "Provide path_name or mission_file")
    try:
        pts = await load_path_for_controller(offboard_ctrl, path_mgr, name)
    except FileNotFoundError as exc:
        raise HTTPException(404, str(exc))
    except MissionLoadConflict as exc:
        raise HTTPException(409, str(exc))
    except PlacementError as exc:
        raise HTTPException(422, str(exc))
    except Exception as exc:
        raise HTTPException(400, f"Load failed: {exc}")
    return {
        "loaded": name,
        "mission_id": offboard_ctrl.loaded_mission_id,
        "num_points": len(pts),
    }


@router.post("/start", dependencies=[Depends(require_token)])
async def start_mission(req: MissionStartRequest | None = None):
    from main import mission_capture, offboard_ctrl, path_mgr, ros_node
    from mission_debug_capture import CaptureUnavailable
    if offboard_ctrl is None:
        raise HTTPException(503, "Controller not ready")

    auto_origin = req.auto_origin if req else False
    name = (req.path_name or req.mission_file) if req else None
    mission_id = req.mission_id if req else None
    try:
        ok, msg = await start_mission_for_controller(
            offboard_ctrl,
            path_mgr,
            ros_node,
            name=name,
            mission_id=mission_id,
            auto_origin=auto_origin,
            capture_coordinator=mission_capture,
            transport="rest",
            start_request={
                "path_name": req.path_name if req else None,
                "mission_file": req.mission_file if req else None,
                "mission_id": mission_id,
                "auto_origin": auto_origin,
            },
        )
    except CaptureUnavailable as exc:
        raise HTTPException(503, f"Mission capture unavailable: {exc}")
    except FileNotFoundError as exc:
        raise HTTPException(404, str(exc))
    except MissionLoadConflict as exc:
        raise HTTPException(409, str(exc))
    except PlacementError as exc:
        raise HTTPException(422, str(exc))
    except Exception as exc:
        raise HTTPException(400, f"Path load failed: {exc}")
    if not ok:
        raise HTTPException(409, f"Mission start failed: {msg}")
    return {"state": offboard_ctrl.state.value, "message": msg}


@router.post("/stop", dependencies=[Depends(require_token)])
async def stop_mission():
    try:
        return await stop_mission_service(_service_context("rest"))
    except MissionServiceError as exc:
        raise _http_from_service_error(exc)


@router.post("/abort", dependencies=[Depends(require_token)])
async def abort_mission():
    try:
        return await abort_mission_service(_service_context("rest"))
    except MissionServiceError as exc:
        raise _http_from_service_error(exc)


def _point_status_payload() -> PointMissionStatusResponse | None:
    from main import point_mission

    if point_mission is None:
        return None
    return PointMissionStatusResponse(**_merge_point_status())


@router.get("/point/status", response_model=PointMissionStatusResponse, dependencies=[Depends(require_token)])
async def point_mission_status():
    """Point Mode runtime diagnostics for the loaded/active point mission."""
    from main import point_mission

    if point_mission is None:
        raise HTTPException(503, "Point mission orchestrator unavailable")
    return PointMissionStatusResponse(**_merge_point_status())


@router.get(
    "/point/events",
    response_model=PointEventHistoryResponse,
    dependencies=[Depends(require_token)],
)
async def point_mission_events(
    since_event_id: int | None = Query(default=None),
):
    from point_events import get_point_event_journal

    payload = get_point_event_journal().history(since_event_id)
    return PointEventHistoryResponse(
        events=payload["events"],
        latest_event_id=payload["latest_event_id"],
        last_event_id=payload["latest_event_id"],
        history_evicted=payload["history_evicted"],
        oldest_available_event_id=payload["oldest_available_event_id"],
    )


@router.post("/pause", response_model=PointPauseResponse, dependencies=[Depends(require_token)])
async def pause_mission():
    try:
        return await pause_point_service(_service_context("rest"))
    except MissionServiceError as exc:
        raise _http_from_service_error(exc)


@router.post("/resume", response_model=PointResumeResponse, dependencies=[Depends(require_token)])
async def resume_mission(req: MissionResumeRequest | None = None):
    try:
        return await resume_point_service(_service_context("rest"), req)
    except MissionServiceError as exc:
        raise _http_from_service_error(exc)


@router.post("/obstacle", response_model=ObstacleStatusResponse, dependencies=[Depends(require_token)])
async def set_obstacle_status(req: ObstacleStatusRequest):
    try:
        return await set_point_obstacle_service(_service_context("rest"), req)
    except MissionServiceError as exc:
        raise _http_from_service_error(exc)


@router.post("/point/continue", response_model=PointContinueResponse, dependencies=[Depends(require_token)])
async def point_mission_continue():
    try:
        return await continue_point_service(_service_context("rest"))
    except MissionServiceError as exc:
        raise _http_from_service_error(exc)


@router.post("/point/skip", response_model=PointSkipResponse, dependencies=[Depends(require_token)])
async def point_mission_skip(req: PointSkipRequest):
    try:
        return await skip_point_service(_service_context("rest"), req)
    except MissionServiceError as exc:
        raise _http_from_service_error(exc)


@router.post("/restart", response_model=MissionRestartResponse, dependencies=[Depends(require_token)])
async def mission_restart(req: MissionRestartRequest):
    try:
        return await restart_mission_service(_service_context("rest"), req)
    except MissionServiceError as exc:
        raise _http_from_service_error(exc)


@router.get("/debug-capture/status", dependencies=[Depends(require_token)])
async def debug_capture_status():
    from main import mission_capture
    if mission_capture is None:
        return {"state": "unavailable", "required": None}
    return mission_capture.get_status()


@router.get(
    "/status",
    response_model=MissionStatus,
    dependencies=[Depends(require_operator_or_machine("mission:status"))],
)
async def mission_status():
    from main import offboard_ctrl, ros_node
    state = offboard_ctrl.state if offboard_ctrl else "idle"
    last_path_loaded = offboard_ctrl.loaded_path_name if offboard_ctrl else None
    loaded_mission_id = offboard_ctrl.loaded_mission_id if offboard_ctrl else None
    running_mission_id = offboard_ctrl.running_mission_id if offboard_ctrl else None
    s = {}
    if ros_node is not None:
        try:
            s = ros_node.get_state()
        except Exception:
            s = {}

    code = RPP_STALE
    dist_to_goal = None
    speed = None
    xtrack = None
    pose_age_ms = s.get("pose_age_ms")
    rpp_debug_age_ms = s.get("rpp_debug_age_ms")
    rpp_debug_fresh = s.get("rpp_debug_fresh")
    measured_speed_m_s = s.get("measured_speed_m_s")
    if ros_node is not None:
        try:
            monitor = ros_node.get_rpp_monitor()
            if monitor.has_snapshot(fresh=True):
                rpp = monitor.get_snapshot()
                code = rpp.state_code
                dist_to_goal = rpp.dist_to_goal_m
                speed = rpp.speed_m_s
                xtrack = rpp.xtrack_m
                pose_age_ms = rpp.pose_age_ms
                age_s = monitor.snapshot_age_s()
                rpp_debug_age_ms = age_s * 1000.0 if age_s is not None else None
                rpp_debug_fresh = True
            elif monitor.has_snapshot():
                age_s = monitor.snapshot_age_s()
                rpp_debug_age_ms = age_s * 1000.0 if age_s is not None else None
                rpp_debug_fresh = False
        except Exception:
            code = s.get("rpp_state", RPP_STALE)
            dist_to_goal = s.get("dist_to_goal_m")
            speed = s.get("speed_m_s")
            xtrack = s.get("xtrack_m")

    point_payload = _point_status_payload()
    return MissionStatus(
        state          = state,
        rpp_state      = code,
        rpp_state_name = RPP_STATE_NAMES.get(code, "UNKNOWN"),
        dist_to_goal   = dist_to_goal,
        speed          = speed,
        xtrack         = xtrack,
        pose_age_ms    = pose_age_ms,
        rpp_debug_age_ms = rpp_debug_age_ms,
        rpp_debug_fresh = rpp_debug_fresh,
        measured_speed_m_s = measured_speed_m_s,
        fcu_connected  = s.get("connected"),
        last_path_loaded = last_path_loaded,
        loaded_mission_id = loaded_mission_id,
        running_mission_id = running_mission_id,
        point = point_payload,
    )