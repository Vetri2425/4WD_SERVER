import os
import sys
from collections import deque
from types import SimpleNamespace

import pytest
from fastapi import HTTPException

sys.path.insert(0, os.path.dirname(__file__))

import main
import offboard_controller as offboard_module
import routes.path as path_routes
import sockets.events as socket_events
from control_arbiter import reset_control_arbiter_for_tests
from mission_placement import GPS_SURVEYED
from models import MissionLoadRequest, MissionStartRequest, MissionState, PathPublishRequest
from offboard_controller import OffboardController
from routes.mission import clear_mission, load_mission, start_mission


@pytest.fixture
def anyio_backend():
    return "asyncio"


class FakeMonitor:
    def reset(self):
        pass


class FakeNode:
    def __init__(self):
        self.calls = []
        self.state = {
            "connected": True,
            "rpp_state": 1,
            "rpp_debug_fresh": True,
            "pose_received": True,
            "global_position_received": True,
            "gps_fix_received": True,
            "local_pose_age_ms": 10.0,
            "global_position_age_ms": 10.0,
            "gps_fix_age_ms": 10.0,
            "pose_global_skew_ms": 0.0,
            "gps_fix": 6,
            "pos_n": 7.4629,
            "pos_e": -0.9070,
            "lat": 13.0720864,
            "lon": 80.2619557,
        }

    def get_state(self):
        return self.state

    def get_rpp_monitor(self):
        return FakeMonitor()

    def publish_path(
        self, points, frame_id="local_ned", spray_flags=None, runtime_entry=False
    ):
        self.calls.append(("publish_path", list(points), spray_flags, runtime_entry))

    async def arm_async(self, arm):
        self.calls.append(("arm", arm))
        return True, ""

    async def set_mode_async(self, mode):
        self.calls.append(("set_mode", mode))
        return True, ""


class FakePathManager:
    def __init__(self):
        self.loads = []

    def load_path(self, name, **kwargs):
        self.loads.append((name, kwargs))
        return [(0.0, 0.0), (1.0, 0.0)]

    def preview_path(self, name):
        return SimpleNamespace(
            waypoints=[SimpleNamespace(spray=True), SimpleNamespace(spray=False)]
        )


def load_protected(ctrl, *, origin_gps=(13.072066, 80.261956)):
    ctrl.load_path(
        [(0.0, -0.035), (1.0, -0.035)],
        name="square_2x2.dxf",
        spray_flags=[True, True],
        placement_mode=GPS_SURVEYED,
        origin_gps=origin_gps,
        mission_id="stg_field",
        is_staged=True,
    )


@pytest.mark.anyio
async def test_start_path_name_cannot_replace_surveyed_mission(monkeypatch):
    node = FakeNode()
    ctrl = OffboardController(node, deque())
    load_protected(ctrl)
    mgr = FakePathManager()
    monkeypatch.setattr(main, "offboard_ctrl", ctrl)
    monkeypatch.setattr(main, "path_mgr", mgr)
    monkeypatch.setattr(main, "ros_node", node)

    with pytest.raises(HTTPException) as exc:
        await start_mission(MissionStartRequest(path_name="square_2x2.dxf"))

    assert exc.value.status_code == 409
    assert mgr.loads == []
    assert node.calls == []
    assert ctrl.loaded_mission_id == "stg_field"


@pytest.mark.anyio
async def test_legacy_path_name_load_and_start_remains_available(monkeypatch):
    old_grace = offboard_module.SETPOINT_STREAM_GRACE_S
    offboard_module.SETPOINT_STREAM_GRACE_S = 0.0
    try:
        node = FakeNode()
        ctrl = OffboardController(node, deque())
        mgr = FakePathManager()
        monkeypatch.setattr(main, "offboard_ctrl", ctrl)
        monkeypatch.setattr(main, "path_mgr", mgr)
        monkeypatch.setattr(main, "ros_node", node)

        response = await start_mission(MissionStartRequest(path_name="local.csv"))

        assert response["state"] == "running"
        assert mgr.loads[0][0] == "local.csv"
        assert ctrl.loaded_mission_id == "local.csv"
        assert ctrl.running_mission_id == "local.csv"
        assert node.calls[0][0] == "publish_path"
    finally:
        offboard_module.SETPOINT_STREAM_GRACE_S = old_grace


@pytest.mark.anyio
async def test_mission_id_mismatch_returns_409_without_side_effects(monkeypatch):
    node = FakeNode()
    ctrl = OffboardController(node, deque())
    load_protected(ctrl)
    monkeypatch.setattr(main, "offboard_ctrl", ctrl)
    monkeypatch.setattr(main, "path_mgr", FakePathManager())
    monkeypatch.setattr(main, "ros_node", node)

    with pytest.raises(HTTPException) as exc:
        await start_mission(MissionStartRequest(mission_id="stg_other"))

    assert exc.value.status_code == 409
    assert node.calls == []
    assert ctrl.running_mission_id is None


@pytest.mark.anyio
async def test_verified_prefix_start_cannot_replace_legacy_resident(monkeypatch):
    node = FakeNode()
    ctrl = OffboardController(node, deque())
    ctrl.load_path(
        [(0.0, 0.0), (1.0, 0.0)],
        name="local.csv",
        mission_id="local.csv",
        spray_flags=[False, False],
    )
    monkeypatch.setattr(main, "offboard_ctrl", ctrl)
    monkeypatch.setattr(main, "path_mgr", FakePathManager())
    monkeypatch.setattr(main, "ros_node", node)

    with pytest.raises(HTTPException) as exc:
        await start_mission(MissionStartRequest(mission_id="vwm_other"))

    assert exc.value.status_code == 409
    assert node.calls == []
    assert ctrl.loaded_mission_id == "local.csv"
    assert ctrl.loaded_mission_kind == "legacy"
    assert ctrl.running_mission_id is None


@pytest.mark.anyio
@pytest.mark.parametrize(
    "override",
    [
        {"local_pose_age_ms": 501.0},
        {"global_position_age_ms": 501.0},
        {"gps_fix_age_ms": 501.0},
        {"gps_fix": 5},
    ],
)
async def test_surveyed_telemetry_failures_return_422(monkeypatch, override):
    node = FakeNode()
    node.state.update(override)
    ctrl = OffboardController(node, deque())
    load_protected(ctrl)
    monkeypatch.setattr(main, "offboard_ctrl", ctrl)
    monkeypatch.setattr(main, "path_mgr", FakePathManager())
    monkeypatch.setattr(main, "ros_node", node)

    with pytest.raises(HTTPException) as exc:
        await start_mission(MissionStartRequest(mission_id="stg_field"))

    assert exc.value.status_code == 422
    assert node.calls == []
    assert ctrl.running_mission_id is None


@pytest.mark.anyio
async def test_invalid_survey_anchor_returns_422(monkeypatch):
    node = FakeNode()
    ctrl = OffboardController(node, deque())

    with pytest.raises(ValueError, match="origin_gps"):
        load_protected(ctrl, origin_gps=(float("nan"), 80.261956))
    assert node.calls == []
    assert ctrl.running_mission_id is None


@pytest.mark.anyio
async def test_surveyed_auto_origin_returns_422(monkeypatch):
    node = FakeNode()
    ctrl = OffboardController(node, deque())
    load_protected(ctrl)
    monkeypatch.setattr(main, "offboard_ctrl", ctrl)
    monkeypatch.setattr(main, "path_mgr", FakePathManager())
    monkeypatch.setattr(main, "ros_node", node)

    with pytest.raises(HTTPException) as exc:
        await start_mission(
            MissionStartRequest(mission_id="stg_field", auto_origin=True)
        )

    assert exc.value.status_code == 422
    assert node.calls == []
    assert ctrl.running_mission_id is None


@pytest.mark.anyio
async def test_legacy_load_cannot_replace_surveyed_mission(monkeypatch):
    node = FakeNode()
    ctrl = OffboardController(node, deque())
    load_protected(ctrl)
    mgr = FakePathManager()
    monkeypatch.setattr(main, "offboard_ctrl", ctrl)
    monkeypatch.setattr(main, "path_mgr", mgr)

    with pytest.raises(HTTPException) as exc:
        await load_mission(MissionLoadRequest(path_name="local.csv"))

    assert exc.value.status_code == 409
    assert mgr.loads == []
    assert ctrl.loaded_mission_id == "stg_field"


@pytest.mark.anyio
@pytest.mark.parametrize("state", [MissionState.IDLE, MissionState.COMPLETED])
async def test_clear_protected_mission_resets_resident_state(monkeypatch, state):
    ctrl = OffboardController(FakeNode(), deque())
    load_protected(ctrl)
    ctrl.state = state
    monkeypatch.setattr(main, "offboard_ctrl", ctrl)

    response = await clear_mission()

    assert response.cleared is True
    assert response.status.loaded is False
    assert response.status.state == "idle"
    assert response.status.name is None
    assert response.status.mission_id is None
    assert response.status.running_mission_id is None
    assert response.status.source_name is None
    assert response.status.placement_mode == "LOCAL_NED"
    assert response.status.origin_gps is None
    assert response.status.is_staged is False
    assert response.status.protected is False
    assert response.status.num_waypoints == 0
    assert response.status.has_spray_flags is False
    assert ctrl.spray_mode == "continuous"


@pytest.mark.anyio
async def test_clear_rejected_while_mission_running(monkeypatch):
    ctrl = OffboardController(FakeNode(), deque())
    load_protected(ctrl)
    ctrl.state = MissionState.RUNNING
    monkeypatch.setattr(main, "offboard_ctrl", ctrl)

    with pytest.raises(HTTPException) as exc:
        await clear_mission()

    assert exc.value.status_code == 409
    assert ctrl.loaded_mission_id == "stg_field"
    assert ctrl.has_protected_mission is True


@pytest.mark.anyio
async def test_legacy_load_succeeds_after_clear(monkeypatch):
    ctrl = OffboardController(FakeNode(), deque())
    load_protected(ctrl)
    mgr = FakePathManager()
    monkeypatch.setattr(main, "offboard_ctrl", ctrl)
    monkeypatch.setattr(main, "path_mgr", mgr)

    await clear_mission()
    response = await load_mission(MissionLoadRequest(path_name="local.csv"))

    assert response["loaded"] == "local.csv"
    assert ctrl.loaded_mission_id == "local.csv"
    assert ctrl.has_protected_mission is False


class FakeSio:
    def __init__(self):
        self.handlers = {}
        self.emitted = []

    def event(self, fn):
        self.handlers[fn.__name__] = fn
        return fn

    def on(self, name):
        def decorator(fn):
            self.handlers[name] = fn
            return fn
        return decorator

    async def emit(self, event, data, to=None):
        self.emitted.append((event, data, to))


@pytest.mark.anyio
async def test_socket_load_and_start_enforce_surveyed_identity(monkeypatch):
    node = FakeNode()
    ctrl = OffboardController(node, deque())
    load_protected(ctrl)
    mgr = FakePathManager()
    sio = FakeSio()
    monkeypatch.setattr(main, "offboard_ctrl", ctrl)
    monkeypatch.setattr(main, "path_mgr", mgr)
    monkeypatch.setattr(main, "ros_node", node)
    monkeypatch.setattr(socket_events, "_auth_ok", lambda data: True)
    socket_events.register_handlers(sio)

    await sio.handlers["mission_load"]("sid", {"path_name": "local.csv"})
    assert sio.emitted[-1][0] == "mission_error"
    assert sio.emitted[-1][1]["status"] == 409
    assert mgr.loads == []

    await sio.handlers["mission_start"]("sid", {"path_name": "local.csv"})
    assert sio.emitted[-1][0] == "mission_status_update"
    assert sio.emitted[-1][1]["status"] == 409
    assert node.calls == []

    node.state["gps_fix_age_ms"] = 501.0
    await sio.handlers["mission_start"]("sid", {"mission_id": "stg_field"})
    assert sio.emitted[-1][0] == "mission_status_update"
    assert sio.emitted[-1][1]["status"] == 422
    assert node.calls == []
    assert ctrl.running_mission_id is None


@pytest.mark.anyio
@pytest.mark.parametrize(
    "state",
    [
        MissionState.LOADING,
        MissionState.ARMING,
        MissionState.SWITCHING_OFFBOARD,
        MissionState.RUNNING,
        MissionState.STOPPING,
        MissionState.DISARMING,
    ],
)
async def test_direct_publish_blocked_during_active_states(monkeypatch, state):
    node = FakeNode()
    ctrl = OffboardController(node, deque())
    ctrl.load_path([(0.0, 0.0), (1.0, 0.0)], name="local.csv")
    ctrl.state = state
    mgr = FakePathManager()
    monkeypatch.setattr(main, "offboard_ctrl", ctrl)
    monkeypatch.setattr(main, "path_mgr", mgr)
    monkeypatch.setattr(main, "ros_node", node)

    with pytest.raises(HTTPException) as exc:
        await path_routes.publish_path(PathPublishRequest(name="local.csv"))

    assert exc.value.status_code == 409
    assert mgr.loads == []
    assert node.calls == []


@pytest.mark.anyio
async def test_direct_publish_blocked_for_idle_protected_mission(monkeypatch):
    node = FakeNode()
    ctrl = OffboardController(node, deque())
    load_protected(ctrl)
    mgr = FakePathManager()
    monkeypatch.setattr(main, "offboard_ctrl", ctrl)
    monkeypatch.setattr(main, "path_mgr", mgr)
    monkeypatch.setattr(main, "ros_node", node)

    with pytest.raises(HTTPException) as exc:
        await path_routes.publish_path(PathPublishRequest(name="local.csv"))

    assert exc.value.status_code == 409
    assert mgr.loads == []
    assert node.calls == []


@pytest.mark.anyio
async def test_direct_publish_blocked_while_joystick_owns_control(monkeypatch):
    arbiter = reset_control_arbiter_for_tests()
    arbiter.mark_joystick_active("session", "lease")
    node = FakeNode()
    ctrl = OffboardController(node, deque())
    ctrl.load_path([(0.0, 0.0), (1.0, 0.0)], name="local.csv")
    mgr = FakePathManager()
    monkeypatch.setattr(main, "offboard_ctrl", ctrl)
    monkeypatch.setattr(main, "path_mgr", mgr)
    monkeypatch.setattr(main, "ros_node", node)

    with pytest.raises(HTTPException) as exc:
        await path_routes.publish_path(PathPublishRequest(name="local.csv"))

    assert exc.value.status_code == 409
    assert "joystick owns manual control" in exc.value.detail
    assert mgr.loads == []
    assert node.calls == []
    reset_control_arbiter_for_tests()


@pytest.mark.anyio
async def test_direct_publish_available_for_idle_unprotected_mission(monkeypatch):
    node = FakeNode()
    ctrl = OffboardController(node, deque())
    ctrl.load_path(
        [(0.0, 0.0), (1.0, 0.0)],
        name="stg_example.csv",
        mission_id="stg_example.csv",
    )
    mgr = FakePathManager()
    monkeypatch.setattr(main, "offboard_ctrl", ctrl)
    monkeypatch.setattr(main, "path_mgr", mgr)
    monkeypatch.setattr(main, "ros_node", node)

    response = await path_routes.publish_path(PathPublishRequest(name="local.csv"))

    assert response == {"published": "local.csv", "num_points": 2}
    assert ctrl.has_protected_mission is False
    assert node.calls[0][0] == "publish_path"


def test_staged_flag_protects_mission_without_stg_prefix():
    ctrl = OffboardController(FakeNode(), deque())
    ctrl.load_path(
        [(0.0, 0.0), (1.0, 0.0)],
        name="local.csv",
        mission_id="mission-42",
        is_staged=True,
    )

    assert ctrl.has_protected_mission is True
    summary = ctrl.loaded_path_summary()
    assert summary["is_staged"] is True
    assert summary["protected"] is True
