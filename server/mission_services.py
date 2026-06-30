"""Shared mission control services for REST and Socket.IO parity."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Literal

from control_arbiter import ControlArbiterError, get_control_arbiter
from mission_loading import start_mission_for_controller
from mission_ops import MissionOperation, MissionOperationConflict, MissionOperationCoordinator
from mission_stop import stop_active_mission
from models import (
    MissionRestartRequest,
    MissionRestartResponse,
    MissionResumeRequest,
    MissionState,
    ObstacleStatusRequest,
    ObstacleStatusResponse,
    PointContinueResponse,
    PointMissionStatusResponse,
    PointPauseResponse,
    PointResumeResponse,
    PointSkipRequest,
    PointSkipResponse,
)
from mission_kind_dispatch import active_target_orchestrator, loaded_mission_kind
from point_mission import PointMissionState


class MissionServiceError(Exception):
    def __init__(self, status_code: int, message: str, code: str = "") -> None:
        self.status_code = status_code
        self.message = message
        self.code = code
        super().__init__(message)


@dataclass
class MissionServiceContext:
    offboard_ctrl: Any
    point_mission: Any
    verified_mission: Any
    ros_node: Any
    hold_owner: Any
    path_mgr: Any
    mission_capture: Any
    transport: Literal["rest", "socketio", "socket"]
    operation_coordinator: MissionOperationCoordinator


def _target_orchestrator(ctx: MissionServiceContext) -> Any | None:
    return active_target_orchestrator(
        offboard_ctrl=ctx.offboard_ctrl,
        point_mission=ctx.point_mission,
        verified_mission=ctx.verified_mission,
    )


def _merge_point_status(ctx: MissionServiceContext) -> dict[str, Any]:
    orchestrator = _target_orchestrator(ctx)
    if orchestrator is None:
        return {}
    payload = orchestrator.status.as_dict()
    if ctx.hold_owner is not None:
        hold = ctx.hold_owner.as_dict(ctx.ros_node)
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


def _point_status(ctx: MissionServiceContext) -> PointMissionStatusResponse:
    return PointMissionStatusResponse(**_merge_point_status(ctx))


def _require_point_mode(offboard_ctrl) -> None:
    if offboard_ctrl.spray_mode != "point":
        raise MissionServiceError(409, "loaded mission is not in point spray mode", "not_point_mode")


def _require_point_orchestrator(ctx: MissionServiceContext) -> None:
    if _target_orchestrator(ctx) is None:
        raise MissionServiceError(503, "Point mission orchestrator unavailable", "unavailable")
    if ctx.offboard_ctrl is None:
        raise MissionServiceError(503, "Controller not ready", "unavailable")


async def _begin_or_conflict(
    coordinator: MissionOperationCoordinator, operation: MissionOperation, timeout_s: float
):
    try:
        return await coordinator.begin(operation, timeout_s=timeout_s)
    except MissionOperationConflict as exc:
        raise MissionServiceError(409, exc.message, "operation_conflict") from exc


async def pause_point_service(ctx: MissionServiceContext) -> PointPauseResponse:
    _require_point_orchestrator(ctx)
    _require_point_mode(ctx.offboard_ctrl)
    token = await _begin_or_conflict(ctx.operation_coordinator, MissionOperation.PAUSE, 0.5)
    try:
        ok, message, status_code = await _target_orchestrator(ctx).pause_mission(
            ctx.ros_node, ctx.hold_owner
        )
        if not ok:
            raise MissionServiceError(status_code, message, "pause_rejected")
        return PointPauseResponse(
            paused=True, message=message, status=_point_status(ctx)
        )
    finally:
        await ctx.operation_coordinator.finish(token)


async def resume_point_service(
    ctx: MissionServiceContext, req: MissionResumeRequest | None = None
) -> PointResumeResponse:
    _require_point_orchestrator(ctx)
    _require_point_mode(ctx.offboard_ctrl)
    token = await _begin_or_conflict(ctx.operation_coordinator, MissionOperation.RESUME, 0.5)
    try:
        try:
            await get_control_arbiter().ensure_mission_motion_allowed(ctx.offboard_ctrl)
        except ControlArbiterError as exc:
            raise MissionServiceError(409, exc.message, "motion_not_allowed") from exc
        expected_generation = req.expected_generation if req else None
        ok, message, status_code = await _target_orchestrator(ctx).resume_mission(
            ctx.ros_node,
            ctx.hold_owner,
            expected_generation=expected_generation,
        )
        if not ok:
            raise MissionServiceError(status_code, message, "resume_rejected")
        return PointResumeResponse(
            resumed=True, message=message, status=_point_status(ctx)
        )
    finally:
        await ctx.operation_coordinator.finish(token)


async def continue_point_service(ctx: MissionServiceContext) -> PointContinueResponse:
    _require_point_orchestrator(ctx)
    _require_point_mode(ctx.offboard_ctrl)
    token = await _begin_or_conflict(
        ctx.operation_coordinator, MissionOperation.CONTINUE, 0.5
    )
    try:
        try:
            await get_control_arbiter().ensure_mission_motion_allowed(ctx.offboard_ctrl)
        except ControlArbiterError as exc:
            raise MissionServiceError(409, exc.message, "motion_not_allowed") from exc
        ok, message, status_code = await _target_orchestrator(ctx).continue_point(ctx.ros_node)
        if not ok:
            raise MissionServiceError(status_code, message, "continue_rejected")
        return PointContinueResponse(
            continued=True, message=message, status=_point_status(ctx)
        )
    finally:
        await ctx.operation_coordinator.finish(token)


async def set_point_obstacle_service(
    ctx: MissionServiceContext, req: ObstacleStatusRequest
) -> ObstacleStatusResponse:
    _require_point_orchestrator(ctx)
    _target_orchestrator(ctx).set_obstacle_clear(req.clear)
    return ObstacleStatusResponse(obstacle_clear=req.clear, status=_point_status(ctx))


async def skip_point_service(
    ctx: MissionServiceContext, req: PointSkipRequest
) -> PointSkipResponse:
    _require_point_orchestrator(ctx)
    _require_point_mode(ctx.offboard_ctrl)
    token = await _begin_or_conflict(ctx.operation_coordinator, MissionOperation.SKIP, 0.5)
    try:
        try:
            await get_control_arbiter().ensure_mission_motion_allowed(ctx.offboard_ctrl)
        except ControlArbiterError as exc:
            raise MissionServiceError(409, exc.message, "motion_not_allowed") from exc
        ok, message, status_code = await _target_orchestrator(ctx).skip_point(
            ctx.ros_node,
            ctx.hold_owner,
            point_index=req.point_index,
            expected_generation=req.expected_generation,
            reason=req.reason,
            operation_token=token,
        )
        if not ok:
            code = "spray_off_failed" if status_code == 503 else "skip_rejected"
            raise MissionServiceError(status_code, message, code)
        return PointSkipResponse(
            skipped=True, message=message, status=_point_status(ctx)
        )
    finally:
        await ctx.operation_coordinator.finish(token)


_ACTIVE_PARENT_STATES = frozenset(
    {
        MissionState.RUNNING,
        MissionState.ARMING,
        MissionState.SWITCHING_OFFBOARD,
        MissionState.STOPPING,
        MissionState.DISARMING,
    }
)


async def abort_mission_service(ctx: MissionServiceContext) -> dict[str, Any]:
    if ctx.offboard_ctrl is None:
        raise MissionServiceError(503, "Controller not ready", "unavailable")
    token = await _begin_or_conflict(ctx.operation_coordinator, MissionOperation.ABORT, 0.25)
    try:
        if ctx.hold_owner is not None:
            ctx.hold_owner.deactivate(ctx.ros_node)
        orchestrator = _target_orchestrator(ctx)
        if (
            orchestrator is not None
            and ctx.offboard_ctrl.spray_mode == "point"
            and (
                orchestrator.is_active()
                or orchestrator.is_paused()
                or orchestrator.status.state not in {
                    PointMissionState.IDLE,
                    PointMissionState.COMPLETED,
                }
            )
        ):
            await orchestrator.terminal_cleanup(
                ctx.ros_node,
                ctx.hold_owner,
                reason="operator_abort",
                terminal_state=PointMissionState.ABORTING,
                operation_token=token,
                offboard_ctrl=ctx.offboard_ctrl,
                require_spray_confirm=True,
            )
        result = await ctx.offboard_ctrl.abort_async()
        if ctx.mission_capture is not None:
            ctx.mission_capture.record_terminal(
                None,
                "operator_abort",
                state=ctx.offboard_ctrl.state.value,
                details={**result, "transport": ctx.transport},
            )
        return result
    finally:
        await ctx.operation_coordinator.finish(token)


async def stop_mission_service(ctx: MissionServiceContext) -> dict[str, Any]:
    if ctx.offboard_ctrl is None:
        raise MissionServiceError(503, "Controller not ready", "unavailable")
    return await stop_active_mission(
        ctx.offboard_ctrl,
        _target_orchestrator(ctx),
        ctx.ros_node,
        ctx.hold_owner,
        mission_capture=ctx.mission_capture,
        transport=ctx.transport,
        operation_coordinator=ctx.operation_coordinator,
    )


async def restart_mission_service(
    ctx: MissionServiceContext, req: MissionRestartRequest
) -> MissionRestartResponse:
    if ctx.offboard_ctrl is None:
        raise MissionServiceError(503, "Controller not ready", "unavailable")
    if not req.mission_id:
        raise MissionServiceError(422, "mission_id is required", "missing_mission_id")
    if not ctx.offboard_ctrl.loaded_mission_id:
        raise MissionServiceError(409, "restart rejected: no mission loaded", "no_mission")
    if req.mission_id != ctx.offboard_ctrl.loaded_mission_id:
        raise MissionServiceError(
            409, "restart rejected: mission identity mismatch", "identity_mismatch"
        )
    token = await _begin_or_conflict(ctx.operation_coordinator, MissionOperation.RESTART, 0.5)
    stop_result: dict[str, Any] | None = None
    start_result: dict[str, Any] | None = None
    started = False
    try:
        active = ctx.offboard_ctrl.state in _ACTIVE_PARENT_STATES
        if active and not req.stop_first:
            raise MissionServiceError(
                409,
                "restart rejected: mission active; set stop_first=true",
                "active_without_stop_first",
            )
        if active:
            orchestrator = _target_orchestrator(ctx)
            if orchestrator is not None and (
                orchestrator.is_active() or orchestrator.is_paused()
            ):
                await orchestrator.terminal_cleanup(
                    ctx.ros_node,
                    ctx.hold_owner,
                    reason="restart_stop_first",
                    terminal_state=PointMissionState.ABORTING,
                    operation_token=token,
                    require_spray_confirm=True,
                )
            if ctx.hold_owner is not None:
                ctx.hold_owner.deactivate(ctx.ros_node)
            from mission_stop import _force_spray_off_confirmed

            spray_attempted, spray_confirmed, spray_off_result = await _force_spray_off_confirmed(
                ctx.ros_node
            )
            stop_result = await ctx.offboard_ctrl.stop_async()
            stop_result["spray_off_attempted"] = spray_attempted
            stop_result["spray_confirmed_off"] = spray_confirmed
            stop_result["spray_off_result"] = spray_off_result
            if spray_attempted and not spray_confirmed:
                stop_result["spray_off_degraded"] = True
                stop_result["success"] = False
                raise MissionServiceError(
                    503,
                    "restart rejected: stop_first failed: spray OFF not confirmed",
                    "stop_first_spray_off",
                )
            if not stop_result.get("success"):
                raise MissionServiceError(
                    409,
                    f"restart rejected: stop_first failed: {stop_result.get('message', 'stop failed')}",
                    "stop_first_failed",
                )
        point_generation: int | None = None
        if ctx.offboard_ctrl.spray_mode == "point":
            orchestrator = _target_orchestrator(ctx)
            if orchestrator is None:
                raise MissionServiceError(503, "Point mission orchestrator unavailable")
            if not hasattr(orchestrator, "reset_for_restart"):
                raise MissionServiceError(503, "Target orchestrator cannot restart", "unavailable")
            point_generation = orchestrator.reset_for_restart(req.mission_id)
        else:
            ctx.offboard_ctrl.reset_progress_for_restart(req.mission_id)
        if req.start_after_reset:
            try:
                await get_control_arbiter().ensure_mission_motion_allowed(ctx.offboard_ctrl)
            except ControlArbiterError as exc:
                raise MissionServiceError(409, exc.message, "motion_not_allowed") from exc
            ok, msg = await start_mission_for_controller(
                ctx.offboard_ctrl,
                ctx.path_mgr,
                ctx.ros_node,
                mission_id=req.mission_id,
                auto_origin=req.auto_origin,
                capture_coordinator=ctx.mission_capture,
                transport=ctx.transport,
            )
            start_result = {"success": ok, "message": msg}
            started = ok
            if not ok:
                raise MissionServiceError(409, f"restart start failed: {msg}", "start_failed")
        return MissionRestartResponse(
            success=True,
            restarted=True,
            reset=True,
            started=started,
            state=ctx.offboard_ctrl.state.value,
            mission_id=req.mission_id,
            point_mission_generation=point_generation,
            message="mission restarted",
            stop_result=stop_result,
            start_result=start_result,
        )
    finally:
        await ctx.operation_coordinator.finish(token)


def build_service_context(
    *,
    offboard_ctrl,
    point_mission,
    verified_mission=None,
    ros_node,
    hold_owner,
    path_mgr,
    mission_capture,
    transport: Literal["rest", "socketio", "socket"],
    operation_coordinator: MissionOperationCoordinator,
) -> MissionServiceContext:
    return MissionServiceContext(
        offboard_ctrl=offboard_ctrl,
        point_mission=point_mission,
        verified_mission=verified_mission,
        ros_node=ros_node,
        hold_owner=hold_owner,
        path_mgr=path_mgr,
        mission_capture=mission_capture,
        transport=transport,
        operation_coordinator=operation_coordinator,
    )