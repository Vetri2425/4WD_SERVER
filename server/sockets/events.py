"""Socket.IO event handlers — client → server commands.

Socket.IO authenticates once at connect with the operator session token. After
that, control events trust the authenticated SID state rather than passwords or
per-event secrets.
"""
from __future__ import annotations

import datetime

from auth import bind_socket_sid, socket_authenticated, unbind_socket_sid
from control_arbiter import ControlArbiterError
from joystick_controller import JoystickError
from logging_setup import get_logger

log = get_logger("server.socket")


def _now() -> str:
    return datetime.datetime.utcnow().isoformat(timespec="seconds") + "Z"


def _auth_ok(sid: str) -> bool:
    return socket_authenticated(sid)


async def _emit_unauth(sio, sid):
    await sio.emit("socket_error", {"reason": "unauthorised"}, to=sid)


async def _emit_joystick_error(sio, sid, exc: Exception):
    code = getattr(exc, "code", "error")
    message = getattr(exc, "message", str(exc))
    # Rejections were previously emitted only to the client and never logged,
    # leaving no server-side trace when a joystick command stream is refused.
    log.warning("joystick_error sid=%s code=%s msg=%s", sid, code, message)
    await sio.emit("joystick_error", {"type": "joystick_error", "code": code, "message": message}, to=sid)


def register_handlers(sio) -> None:
    """Attach all client → server event handlers to the given AsyncServer."""

    @sio.event
    async def connect(sid, environ, auth=None):
        from main import activity_log
        token = None
        if isinstance(auth, dict):
            token = auth.get("token") or auth.get("auth")
        elif isinstance(auth, str):
            token = auth
        if bind_socket_sid(sid, token) is None:
            raise ConnectionRefusedError("unauthorised")
        activity_log.append({"timestamp": _now(), "level": "info",
                              "message": f"Socket connected: {sid}"})

    @sio.event
    async def disconnect(sid):
        from main import activity_log, joystick_ctrl
        unbind_socket_sid(sid)
        activity_log.append({"timestamp": _now(), "level": "info",
                              "message": f"Socket disconnected: {sid}"})
        if joystick_ctrl is not None and joystick_ctrl.owner_sid == sid:
            try:
                result = await joystick_ctrl.release(sid, reason="disconnect")
                from main import _emit_authenticated
                await _emit_authenticated("joystick_released", result)
            except Exception as exc:
                log.warning("joystick release on disconnect failed: %s", exc)

    # ── Vehicle control ───────────────────────────────────────────────────────

    @sio.on("arm")
    async def on_arm(sid, data):
        from main import ros_node, activity_log
        from spray_safety import disarm_with_spray_safety
        if not _auth_ok(sid):
            return await _emit_unauth(sio, sid)
        if ros_node is None:
            return
        arm_val = data.get("arm", True) if isinstance(data, dict) else bool(data)
        if arm_val:
            ok, why = await ros_node.arm_async(True)
            activity_log.append({"timestamp": _now(),
                                  "level": "info" if ok else "error",
                                  "message": f"Armed via socket: "
                                             f"{'OK' if ok else f'FAILED ({why})'}"})
            await sio.emit(
                "arm_result",
                {"success": ok, "arm": True, "message": why},
                to=sid,
            )
            return
        result = await disarm_with_spray_safety(ros_node)
        activity_log.append(
            {
                "timestamp": _now(),
                "level": "info" if result.success else "warning",
                "message": f"Disarmed via socket: {result.message}",
            }
        )
        payload = result.as_socket_payload(transition_key="arm", transition_value=False)
        payload["arm"] = False
        await sio.emit("arm_result", payload, to=sid)

    @sio.on("set_mode")
    async def on_set_mode(sid, data):
        from main import ros_node, activity_log
        from spray_safety import set_mode_with_spray_safety
        if not _auth_ok(sid):
            return await _emit_unauth(sio, sid)
        if ros_node is None:
            return
        mode = data.get("mode", "MANUAL") if isinstance(data, dict) else str(data)
        if str(mode).upper() == "OFFBOARD":
            await sio.emit(
                "mode_result",
                {
                    "success": False,
                    "mode": mode,
                    "message": "OFFBOARD transitions must use mission_start",
                },
                to=sid,
            )
            return
        state = ros_node.get_state()
        current_mode = str(state.get("mode", "UNKNOWN"))
        result = await set_mode_with_spray_safety(
            ros_node,
            target_mode=str(mode),
            current_mode=current_mode,
        )
        activity_log.append(
            {
                "timestamp": _now(),
                "level": "info" if result.success else "warning",
                "message": f"set_mode {mode}: {result.message}",
            }
        )
        payload = result.as_socket_payload(transition_key="mode", transition_value=mode)
        payload["mode"] = mode
        await sio.emit("mode_result", payload, to=sid)

    @sio.on("emergency_stop")
    async def on_estop(sid, data=None):
        from main import emergency_handler
        if not _auth_ok(sid):
            return await _emit_unauth(sio, sid)
        if emergency_handler is None:
            return
        result = await emergency_handler.estop_async()
        await sio.emit("estop_result", result, to=sid)

    # ── Virtual joystick V2 ──────────────────────────────────────────────────

    @sio.on("joystick_acquire")
    async def on_joystick_acquire(sid, data):
        from main import joystick_ctrl
        if not _auth_ok(sid):
            return await _emit_unauth(sio, sid)
        if joystick_ctrl is None:
            return await _emit_joystick_error(
                sio, sid, JoystickError("unavailable", "joystick controller unavailable")
            )
        try:
            result = await joystick_ctrl.acquire(sid, data if isinstance(data, dict) else {})
        except (JoystickError, ControlArbiterError) as exc:
            return await _emit_joystick_error(sio, sid, exc)
        await sio.emit("joystick_acquired", result, to=sid)

    @sio.on("joystick_command")
    async def on_joystick_command(sid, data):
        from main import joystick_ctrl
        if not _auth_ok(sid):
            return await _emit_unauth(sio, sid)
        if joystick_ctrl is None:
            return await _emit_joystick_error(
                sio, sid, JoystickError("unavailable", "joystick controller unavailable")
            )
        try:
            joystick_ctrl.handle_command(sid, data if isinstance(data, dict) else {})
        except (JoystickError, ControlArbiterError) as exc:
            return await _emit_joystick_error(sio, sid, exc)

    @sio.on("joystick_release")
    async def on_joystick_release(sid, data):
        from main import joystick_ctrl
        if not _auth_ok(sid):
            return await _emit_unauth(sio, sid)
        if joystick_ctrl is None:
            return await _emit_joystick_error(
                sio, sid, JoystickError("unavailable", "joystick controller unavailable")
            )
        payload = data if isinstance(data, dict) else {}
        try:
            result = await joystick_ctrl.release(
                sid,
                session_id=payload.get("session_id"),
                lease_id=payload.get("lease_id"),
                reason="explicit",
            )
        except (JoystickError, ControlArbiterError) as exc:
            return await _emit_joystick_error(sio, sid, exc)
        await sio.emit("joystick_released", result, to=sid)

    # ── Mission control ───────────────────────────────────────────────────────

    @sio.on("mission_load")
    async def on_mission_load(sid, data):
        from main import offboard_ctrl, path_mgr
        from mission_loading import MissionLoadConflict, load_path_for_controller
        if not _auth_ok(sid):
            return await _emit_unauth(sio, sid)
        name = (data.get("path_name") or data.get("mission_file")
                if isinstance(data, dict) else None)
        if not name:
            await sio.emit("mission_error",
                           {"message": "No path name provided"}, to=sid)
            return
        try:
            pts = await load_path_for_controller(offboard_ctrl, path_mgr, name)
            await sio.emit("mission_loaded",
                           {"name": name,
                            "mission_id": offboard_ctrl.loaded_mission_id,
                            "num_points": len(pts)}, to=sid)
        except MissionLoadConflict as exc:
            await sio.emit("mission_error",
                           {"message": str(exc), "status": 409}, to=sid)
        except Exception as exc:
            await sio.emit("mission_error", {"message": str(exc)}, to=sid)

    @sio.on("mission_start")
    async def on_mission_start(sid, data=None):
        from main import mission_capture, offboard_ctrl, path_mgr, ros_node
        from mission_debug_capture import CaptureUnavailable
        from mission_loading import MissionLoadConflict, start_mission_for_controller
        from mission_placement import PlacementError
        if not _auth_ok(sid):
            return await _emit_unauth(sio, sid)
        payload = data if isinstance(data, dict) else {}
        name = payload.get("path_name") or payload.get("mission_file")
        try:
            ok, msg = await start_mission_for_controller(
                offboard_ctrl,
                path_mgr,
                ros_node,
                name=name,
                mission_id=payload.get("mission_id"),
                auto_origin=bool(payload.get("auto_origin", False)),
                capture_coordinator=mission_capture,
                transport="socketio",
                start_request={
                    "path_name": payload.get("path_name"),
                    "mission_file": payload.get("mission_file"),
                    "mission_id": payload.get("mission_id"),
                    "auto_origin": bool(payload.get("auto_origin", False)),
                },
            )
        except CaptureUnavailable as exc:
            ok, msg, status = False, f"Mission capture unavailable: {exc}", 503
        except MissionLoadConflict as exc:
            ok, msg, status = False, str(exc), 409
        except PlacementError as exc:
            ok, msg, status = False, str(exc), 422
        except Exception as exc:
            ok, msg, status = False, str(exc), 409
        else:
            status = 200 if ok else 409
        await sio.emit("mission_status_update",
                       {"state":   offboard_ctrl.state.value,
                        "success": ok,
                        "message": msg,
                        "status": status}, to=sid)

    def _socket_service_context():
        from main import (
            hold_owner,
            mission_capture,
            offboard_ctrl,
            operation_coordinator,
            path_mgr,
            point_mission,
            ros_node,
            verified_mission,
        )
        from mission_ops import MissionOperationCoordinator
        from mission_services import build_service_context

        coordinator = operation_coordinator or MissionOperationCoordinator()
        return build_service_context(
            offboard_ctrl=offboard_ctrl,
            point_mission=point_mission,
            verified_mission=verified_mission,
            ros_node=ros_node,
            hold_owner=hold_owner,
            path_mgr=path_mgr,
            mission_capture=mission_capture,
            transport="socketio",
            operation_coordinator=coordinator,
        )

    def _command_error_payload(exc):
        from mission_services import MissionServiceError

        if isinstance(exc, MissionServiceError):
            return {
                "success": False,
                "status": exc.status_code,
                "code": exc.code or "service_error",
                "message": exc.message,
            }
        return {"success": False, "status": 500, "code": "error", "message": str(exc)}

    async def _emit_service_error(sid, exc):
        payload = _command_error_payload(exc)
        await sio.emit("mission_error", payload, to=sid)
        return payload

    @sio.on("mission_stop")
    async def on_mission_stop(sid, data=None):
        from mission_services import stop_mission_service
        if not _auth_ok(sid):
            return await _emit_unauth(sio, sid)
        try:
            result = await stop_mission_service(_socket_service_context())
        except Exception as exc:
            payload = await _emit_service_error(sid, exc)
            await sio.emit("mission_stop_result", payload, to=sid)
            return
        await sio.emit("mission_status_update", result, to=sid)
        await sio.emit("mission_stop_result", {"status": 200, **result}, to=sid)

    @sio.on("mission_abort")
    async def on_mission_abort(sid, data=None):
        from mission_services import abort_mission_service
        if not _auth_ok(sid):
            return await _emit_unauth(sio, sid)
        try:
            result = await abort_mission_service(_socket_service_context())
        except Exception as exc:
            payload = await _emit_service_error(sid, exc)
            await sio.emit("mission_abort_result", payload, to=sid)
            return
        await sio.emit("mission_status_update", result, to=sid)
        await sio.emit("mission_abort_result", {"status": 200, **result}, to=sid)

    @sio.on("mission_pause")
    async def on_mission_pause(sid, data=None):
        from mission_services import pause_point_service
        if not _auth_ok(sid):
            return await _emit_unauth(sio, sid)
        try:
            result = await pause_point_service(_socket_service_context())
            await sio.emit(
                "mission_pause_result",
                {"success": True, "status": 200, **result.model_dump()},
                to=sid,
            )
        except Exception as exc:
            payload = await _emit_service_error(sid, exc)
            await sio.emit("mission_pause_result", payload, to=sid)

    @sio.on("mission_resume")
    async def on_mission_resume(sid, data=None):
        from mission_services import resume_point_service
        from models import MissionResumeRequest

        if not _auth_ok(sid):
            return await _emit_unauth(sio, sid)
        payload = data if isinstance(data, dict) else {}
        req = MissionResumeRequest(
            expected_generation=payload.get("expected_generation")
        )
        try:
            result = await resume_point_service(_socket_service_context(), req)
            await sio.emit(
                "mission_resume_result",
                {"success": True, "status": 200, **result.model_dump()},
                to=sid,
            )
        except Exception as exc:
            payload = await _emit_service_error(sid, exc)
            await sio.emit("mission_resume_result", payload, to=sid)

    @sio.on("point_continue")
    async def on_point_continue(sid, data=None):
        from mission_services import continue_point_service

        if not _auth_ok(sid):
            return await _emit_unauth(sio, sid)
        try:
            result = await continue_point_service(_socket_service_context())
            await sio.emit(
                "point_continue_result",
                {"success": True, "status": 200, **result.model_dump()},
                to=sid,
            )
        except Exception as exc:
            payload = await _emit_service_error(sid, exc)
            await sio.emit("point_continue_result", payload, to=sid)

    @sio.on("mission_obstacle")
    async def on_mission_obstacle(sid, data=None):
        from mission_services import set_point_obstacle_service
        from models import ObstacleStatusRequest

        if not _auth_ok(sid):
            return await _emit_unauth(sio, sid)
        payload = data if isinstance(data, dict) else {}
        try:
            result = await set_point_obstacle_service(
                _socket_service_context(),
                ObstacleStatusRequest(clear=bool(payload.get("clear", True))),
            )
            await sio.emit(
                "mission_obstacle_result",
                {"success": True, "status": 200, **result.model_dump()},
                to=sid,
            )
        except Exception as exc:
            payload = await _emit_service_error(sid, exc)
            await sio.emit("mission_obstacle_result", payload, to=sid)

    @sio.on("point_skip")
    async def on_point_skip(sid, data=None):
        from mission_services import skip_point_service
        from models import PointSkipRequest

        if not _auth_ok(sid):
            return await _emit_unauth(sio, sid)
        payload = data if isinstance(data, dict) else {}
        try:
            result = await skip_point_service(
                _socket_service_context(),
                PointSkipRequest(
                    point_index=int(payload.get("point_index", 0)),
                    expected_generation=payload.get("expected_generation"),
                    reason=str(payload.get("reason") or "operator_skip"),
                ),
            )
            await sio.emit(
                "point_skip_result",
                {"success": True, "status": 200, **result.model_dump()},
                to=sid,
            )
        except Exception as exc:
            payload = await _emit_service_error(sid, exc)
            await sio.emit("point_skip_result", payload, to=sid)

    @sio.on("mission_restart")
    async def on_mission_restart(sid, data=None):
        from mission_services import restart_mission_service
        from models import MissionRestartRequest

        if not _auth_ok(sid):
            return await _emit_unauth(sio, sid)
        payload = data if isinstance(data, dict) else {}
        try:
            result = await restart_mission_service(
                _socket_service_context(),
                MissionRestartRequest(
                    mission_id=str(payload.get("mission_id", "")),
                    stop_first=bool(payload.get("stop_first", False)),
                    start_after_reset=bool(payload.get("start_after_reset", False)),
                    auto_origin=bool(payload.get("auto_origin", False)),
                ),
            )
            await sio.emit(
                "mission_restart_result",
                {"success": True, "status": 200, **result.model_dump()},
                to=sid,
            )
        except Exception as exc:
            payload = await _emit_service_error(sid, exc)
            await sio.emit("mission_restart_result", payload, to=sid)

    @sio.on("request_params")
    async def on_request_params(sid, data):
        from main import ros_node
        if not _auth_ok(sid):
            return await _emit_unauth(sio, sid)
        if ros_node is None:
            return
        names = data.get("names", []) if isinstance(data, dict) else []
        out = {}
        for name in names:
            ok, value, _ = await ros_node.get_param_async(name)
            out[name] = value if ok else None
        await sio.emit("params_result", out, to=sid)
