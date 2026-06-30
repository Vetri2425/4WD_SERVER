"""Shared mission path loading guards."""

from __future__ import annotations

import asyncio
from typing import Optional

from config import POSE_STALE_MS, SPRAY_DEFAULT_ON, VERIFIED_MISSION_ID_PREFIX
from logging_setup import get_logger
from mission_placement import PlacementError
from models import MissionState
from spray_safety import cleanup_mission_start_failure

log = get_logger("server.mission_loading")


def _spray_startup_block_reason() -> str | None:
    try:
        from main import spray_startup_reconciliation
    except Exception:
        return None
    if spray_startup_reconciliation is None:
        return None
    return spray_startup_reconciliation.block_reason()


MIN_MISSION_POINTS = 2
LOAD_ALLOWED_STATES = {
    MissionState.IDLE,
    MissionState.COMPLETED,
    MissionState.ABORTED,
    MissionState.ERROR,
}

_load_lock = asyncio.Lock()


class MissionLoadConflict(Exception):
    """Raised when lifecycle state makes mission loading unsafe."""


def load_block_reason(state: MissionState) -> Optional[str]:
    if state in LOAD_ALLOWED_STATES:
        return None
    return f"Cannot load mission while controller state is {state.value}"


def validate_point_count(points: list[tuple[float, float]]) -> None:
    if len(points) < MIN_MISSION_POINTS:
        raise ValueError(
            f"Mission path must contain at least {MIN_MISSION_POINTS} points "
            f"(got {len(points)})"
        )


def spray_flags_for_path(path_mgr, name: str, points_len: int) -> list[bool]:
    """Return per-point spray flags for a loaded path, or a configured legacy default."""
    try:
        preview = path_mgr.preview_path(name)
        flags = [bool(wp.spray) for wp in preview.waypoints]
    except Exception:
        flags = [SPRAY_DEFAULT_ON] * points_len
    if len(flags) != points_len:
        flags = [SPRAY_DEFAULT_ON] * points_len
    return flags


def pose_origin_or_error(state: dict) -> tuple[float, float] | str:
    if not state.get("pose_received", False):
        return "auto_origin requested but no local pose received yet"
    pose_age_raw = state.get("pose_age_ms")
    if pose_age_raw is None:
        pose_age = POSE_STALE_MS + 1.0
    else:
        pose_age = float(pose_age_raw)
    if pose_age > POSE_STALE_MS:
        return (
            f"auto_origin requested but local pose is stale "
            f"({pose_age:.0f} ms > {POSE_STALE_MS:.0f} ms)"
        )
    return (float(state.get("pos_n", 0.0)), float(state.get("pos_e", 0.0)))


async def load_path_for_controller(
    offboard_ctrl,
    path_mgr,
    name: str,
    *,
    origin: tuple[float, float] = (0.0, 0.0),
    start_position: tuple[float, float] | None = None,
    auto_origin: bool = False,
) -> list[tuple[float, float]]:
    """Load and validate a path without blocking the FastAPI event loop."""
    async with _load_lock:
        startup_block = _spray_startup_block_reason()
        if startup_block:
            raise MissionLoadConflict(startup_block)
        if offboard_ctrl.has_protected_mission:
            raise MissionLoadConflict(
                f"Loaded mission {offboard_ctrl.loaded_mission_id!r} is staged/surveyed; "
                "legacy load cannot replace it"
            )
        prior_state = offboard_ctrl.state
        reason = load_block_reason(prior_state)
        if reason:
            raise MissionLoadConflict(reason)

        offboard_ctrl.state = MissionState.LOADING
        try:
            points = await asyncio.to_thread(
                path_mgr.load_path,
                name,
                origin=origin,
                start_position=start_position,
                auto_origin=auto_origin,
            )
            validate_point_count(points)
            spray_flags = spray_flags_for_path(path_mgr, name, len(points))
        except Exception:
            offboard_ctrl.state = prior_state
            raise

        offboard_ctrl.state = prior_state
        offboard_ctrl.load_path(points, name=name, spray_flags=spray_flags)
        return points


def _verified_identity_conflict(
    offboard_ctrl,
    mission_id: str | None,
) -> str | None:
    """Return error message when request id disagrees with resident metadata."""
    if not mission_id:
        return None
    loaded_id = getattr(offboard_ctrl, "loaded_mission_id", None)
    loaded_kind = getattr(offboard_ctrl, "loaded_mission_kind", "none")
    looks_verified = mission_id.startswith(VERIFIED_MISSION_ID_PREFIX)
    if looks_verified and loaded_kind not in {"none", "verified_gps"}:
        return (
            f"mission_id {mission_id!r} looks verified but loaded kind is "
            f"{loaded_kind!r}"
        )
    if loaded_kind == "verified_gps" and loaded_id and mission_id != loaded_id:
        return (
            f"mission_id {mission_id!r} does not match loaded mission "
            f"{loaded_id!r}"
        )
    if looks_verified and loaded_kind == "verified_gps" and loaded_id != mission_id:
        return (
            f"mission_id {mission_id!r} does not match loaded mission "
            f"{loaded_id!r}"
        )
    return None


async def start_verified_gps_mission(
    offboard_ctrl,
    ros_node,
    *,
    mission_id: str | None,
    capture_coordinator=None,
    transport: str = "internal",
    start_request: dict | None = None,
) -> tuple[bool, str]:
    """Start a verified GPS mission using resident metadata routing."""
    from main import hold_owner, verified_mission, verified_mission_store
    from mission_ops import MissionOperation, MissionOperationCoordinator
    from target_mission.types import TargetMissionState
    from verified_mission.loader import ensure_verified_mission_loaded

    if verified_mission is None:
        raise MissionLoadConflict("verified mission orchestrator not ready")
    if ros_node is None:
        raise MissionLoadConflict("ROS node not ready")
    if not mission_id:
        mission_id = getattr(offboard_ctrl, "loaded_mission_id", None)
    if not mission_id:
        raise MissionLoadConflict("mission_id is required for verified GPS start")

    conflict = _verified_identity_conflict(offboard_ctrl, mission_id)
    if conflict:
        raise MissionLoadConflict(conflict)

    ensure_verified_mission_loaded(
        mission_id,
        offboard_ctrl=offboard_ctrl,
        orchestrator=verified_mission,
        store=verified_mission_store,
    )

    capture_id = None
    if capture_coordinator is not None:
        capture_id = await capture_coordinator.begin_capture(
            offboard_ctrl,
            start_request=start_request or {"mission_id": mission_id},
            transport=transport,
        )

    needs_cleanup = False
    try:
        try:
            verified_mission.prepare(ros_node.get_state())
        except PlacementError as exc:
            raise MissionLoadConflict(str(exc)) from exc

        ok, message = await offboard_ctrl.start_async(
            expected_mission_id=mission_id,
        )
        if ok:
            started, why = await verified_mission.start(
                ros_node, offboard_ctrl, hold_owner
            )
            if not started:
                needs_cleanup = True
                ok, message = False, why
    except Exception as exc:
        if capture_coordinator is not None:
            capture_coordinator.record_start_result(
                capture_id,
                success=False,
                state=offboard_ctrl.state.value,
                message=str(exc),
            )
        raise
    finally:
        if needs_cleanup and ros_node is not None:
            try:
                from main import operation_coordinator

                coordinator = operation_coordinator or MissionOperationCoordinator()
                token = await coordinator.begin(MissionOperation.ABORT, timeout_s=0.25)
                try:
                    await verified_mission.terminal_cleanup(
                        ros_node,
                        hold_owner,
                        reason="start_failure",
                        terminal_state=TargetMissionState.FAILED,
                        operation_token=token,
                        offboard_ctrl=offboard_ctrl,
                        require_spray_confirm=True,
                    )
                finally:
                    await coordinator.finish(token)
                spray_off_result = await cleanup_mission_start_failure(
                    ros_node, offboard_ctrl
                )
                if (
                    spray_off_result.get("attempted")
                    and spray_off_result.get("live")
                    and not spray_off_result.get("success")
                ):
                    log.warning(
                        "verified mission start cleanup: spray OFF not confirmed: %s",
                        spray_off_result,
                    )
            except Exception:
                log.exception("verified mission start cleanup failed")

    if capture_coordinator is not None:
        capture_coordinator.record_start_result(
            capture_id,
            success=ok,
            state=offboard_ctrl.state.value,
            message=message,
        )
    return ok, message


async def start_mission_for_controller(
    offboard_ctrl,
    path_mgr,
    ros_node,
    *,
    name: str | None = None,
    mission_id: str | None = None,
    auto_origin: bool = False,
    capture_coordinator=None,
    transport: str = "internal",
    start_request: dict | None = None,
) -> tuple[bool, str]:
    """Start the resident mission, preserving legacy load-and-start for local paths."""
    startup_block = _spray_startup_block_reason()
    if startup_block:
        raise MissionLoadConflict(startup_block)

    loaded_kind = getattr(offboard_ctrl, "loaded_mission_kind", "none")
    if loaded_kind == "verified_gps":
        return await start_verified_gps_mission(
            offboard_ctrl,
            ros_node,
            mission_id=mission_id,
            capture_coordinator=capture_coordinator,
            transport=transport,
            start_request=start_request,
        )
    if mission_id and mission_id.startswith(VERIFIED_MISSION_ID_PREFIX):
        from verified_mission.loader import ensure_verified_mission_loaded
        from main import verified_mission, verified_mission_store

        if loaded_kind not in {"none", "verified_gps"}:
            raise MissionLoadConflict(
                f"mission_id {mission_id!r} looks verified but loaded kind is "
                f"{loaded_kind!r}"
            )
        ensure_verified_mission_loaded(
            mission_id,
            offboard_ctrl=offboard_ctrl,
            orchestrator=verified_mission,
            store=verified_mission_store,
        )
        return await start_verified_gps_mission(
            offboard_ctrl,
            ros_node,
            mission_id=mission_id,
            capture_coordinator=capture_coordinator,
            transport=transport,
            start_request=start_request,
        )

    if offboard_ctrl.has_protected_mission and name:
        raise MissionLoadConflict(
            f"Loaded mission {offboard_ctrl.loaded_mission_id!r} is staged/surveyed; "
            "path_name cannot replace it during start"
        )

    origin = (0.0, 0.0)
    start_position = None
    origin_pre_applied = False

    if auto_origin and not offboard_ctrl.has_protected_mission:
        if ros_node is None:
            raise MissionLoadConflict("ROS node not ready")
        pose_origin = pose_origin_or_error(ros_node.get_state())
        if isinstance(pose_origin, str):
            raise MissionLoadConflict(pose_origin)
        origin = pose_origin
        start_position = origin
        if not name:
            loaded = offboard_ctrl.loaded_path_name
            if loaded and loaded != "unknown":
                name = loaded

    if name:
        await load_path_for_controller(
            offboard_ctrl,
            path_mgr,
            name,
            origin=origin,
            start_position=start_position,
            auto_origin=auto_origin,
        )
        origin_pre_applied = auto_origin

    capture_id = None
    if capture_coordinator is not None:
        capture_id = await capture_coordinator.begin_capture(
            offboard_ctrl,
            start_request=start_request or {
                "path_name": name,
                "mission_id": mission_id,
                "auto_origin": auto_origin,
            },
            transport=transport,
        )

    def _placement_hook(payload: dict) -> None:
        if capture_coordinator is not None:
            capture_coordinator.record_placement(capture_id, payload)

    needs_point_cleanup = False
    try:
        if (
            offboard_ctrl.spray_mode == "point"
            and loaded_kind != "verified_gps"
        ):
            from main import point_mission

            if point_mission is None:
                raise MissionLoadConflict("point mission orchestrator not ready")
            if ros_node is None:
                raise MissionLoadConflict("ROS node not ready")
            point_mission.prepare(ros_node.get_state())
        ok, message = await offboard_ctrl.start_async(
            auto_origin=auto_origin and not origin_pre_applied,
            expected_mission_id=mission_id,
            pre_publish_hook=_placement_hook if capture_id else None,
        )
        if (
            ok
            and offboard_ctrl.spray_mode == "point"
            and loaded_kind != "verified_gps"
        ):
            from main import hold_owner

            try:
                started, why = await point_mission.start(
                    ros_node, offboard_ctrl, hold_owner
                )
            except Exception:
                needs_point_cleanup = True
                raise
            if not started:
                needs_point_cleanup = True
                ok, message = False, why
    except Exception as exc:
        if capture_coordinator is not None:
            capture_coordinator.record_start_result(
                capture_id,
                success=False,
                state=offboard_ctrl.state.value,
                message=str(exc),
            )
        raise
    finally:
        if needs_point_cleanup and ros_node is not None:
            try:
                from main import hold_owner, operation_coordinator, point_mission
                from mission_ops import MissionOperation, MissionOperationCoordinator
                from point_mission import PointMissionState

                coordinator = operation_coordinator or MissionOperationCoordinator()
                token = await coordinator.begin(MissionOperation.ABORT, timeout_s=0.25)
                try:
                    cleanup_fn = (
                        getattr(point_mission, "terminal_cleanup", None)
                        if point_mission is not None
                        else None
                    )
                    if callable(cleanup_fn):
                        cleanup_result = cleanup_fn(
                            ros_node,
                            hold_owner,
                            reason="start_failure",
                            terminal_state=PointMissionState.FAILED,
                            operation_token=token,
                            offboard_ctrl=offboard_ctrl,
                            require_spray_confirm=True,
                        )
                        if asyncio.iscoroutine(cleanup_result):
                            await cleanup_result
                finally:
                    await coordinator.finish(token)
                spray_off_result = await cleanup_mission_start_failure(
                    ros_node, offboard_ctrl
                )
                if (
                    spray_off_result.get("attempted")
                    and spray_off_result.get("live")
                    and not spray_off_result.get("success")
                ):
                    log.warning(
                        "point mission start cleanup: spray OFF not confirmed: %s",
                        spray_off_result,
                    )
            except Exception:
                log.exception("point mission start cleanup failed")
    if capture_coordinator is not None:
        capture_coordinator.record_start_result(
            capture_id,
            success=ok,
            state=offboard_ctrl.state.value,
            message=message,
        )
    return ok, message
