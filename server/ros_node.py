"""Single rclpy node + MultiThreadedExecutor running in a background thread.

Threading model:
  - One rclpy node (`RosBridgeNode`).
  - Spun by a `MultiThreadedExecutor(num_threads=4)` on a daemon thread.
  - All service clients live in a `ReentrantCallbackGroup` so they can be
    invoked from any thread without deadlock.
  - Public methods are *async*: each wraps `call_async` with
    `add_done_callback` + `loop.call_soon_threadsafe(future.set_result, ...)`,
    so the FastAPI event loop is never blocked.

Routes / sockets call `await ros_node.arm_async(...)` etc. The legacy sync
methods `arm()` / `set_mode()` are kept as thin wrappers that block the
caller's thread (used only by the offboard controller's sync `start()`
shim if ever needed) but **must not** be called from the asyncio loop.
"""

from __future__ import annotations

import asyncio
import json
import math
import threading
import time
from typing import Any, Callable

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Float32MultiArray, String

from spray_runtime_protocol import (
    RUNTIME_STATUS_MAX_AGE_S,
    RUNTIME_STATUS_TOPIC,
    deserialize_runtime_status,
    parse_dwell_response,
    serialize_dwell_command,
)
from path_identity import (
    PATH_IDENTITY_TOPIC,
    make_path_identity,
)

from config import (
    SRV_RPP_GET_PARAMS,
    SRV_RPP_LIST_PARAMS,
    SRV_RPP_SET_PARAMS,
    SRV_SPRAY_APPLY_MISSION_CONFIG,
    SRV_SPRAY_CANCEL_DWELL,
    SRV_SPRAY_GET_PARAMS,
    SRV_SPRAY_SET_PARAMS,
    SRV_SPRAY_START_DWELL,
)
from logging_setup import get_logger
from path_validation import (
    normalize_path_points,
    normalize_spray_flags,
    verified_path_fingerprint,
)
from rpp_status import RppStatusMonitor

log = get_logger("server.ros")

# ── Optional MAVROS imports ───────────────────────────────────────────────────
try:
    from mavros_msgs.msg import State
    from sensor_msgs.msg import BatteryState, NavSatFix
    from mavros_msgs.srv import CommandBool, SetMode

    _HAS_MAVROS = True
except ImportError:
    _HAS_MAVROS = False
    State = BatteryState = NavSatFix = CommandBool = SetMode = None  # type: ignore

try:
    from mavros_msgs.msg import GPSRAW

    _HAS_GPSRAW = True
except ImportError:
    _HAS_GPSRAW = False
    GPSRAW = None  # type: ignore

# Standard rcl_interfaces param services (always available with ROS2)
try:
    from rcl_interfaces.srv import GetParameters, SetParameters, ListParameters
    from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

    _HAS_PARAM_SRV = True
except ImportError:
    _HAS_PARAM_SRV = False
    GetParameters = SetParameters = ListParameters = None  # type: ignore
    Parameter = ParameterValue = ParameterType = None  # type: ignore


# ── QoS helpers ───────────────────────────────────────────────────────────────


def _qos_reliable_tl(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )


def _qos_best_effort(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        depth=depth,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
    )


# ── Executor lifecycle helper ─────────────────────────────────────────────────


class RosExecutorThread:
    """Owns a MultiThreadedExecutor, drains it cooperatively in a thread."""

    def __init__(self, num_threads: int = 4) -> None:
        self._exe = MultiThreadedExecutor(num_threads=num_threads)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def add_node(self, node: Node) -> None:
        self._exe.add_node(node)

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(
            target=self._spin_loop, daemon=True, name="rclpy-mt-spin"
        )
        self._thread.start()

    def stop(self, join_timeout: float = 2.0) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=join_timeout)

    def _spin_loop(self) -> None:
        try:
            while rclpy.ok() and not self._stop.is_set():
                # spin_once with timeout lets us notice the stop event
                self._exe.spin_once(timeout_sec=0.1)
        except Exception:
            log.exception("rclpy executor crashed")
        finally:
            try:
                self._exe.shutdown()
            except Exception:
                pass


# ── Node ──────────────────────────────────────────────────────────────────────


class RosBridgeNode(Node):
    """Single rclpy node; thread-safe shared state dict."""

    _DEFAULT_STATE: dict[str, Any] = {
        "armed": False,
        "mode": "UNKNOWN",
        "connected": False,
        "pos_n": 0.0,
        "pos_e": 0.0,
        "pose_received": False,
        "global_position_received": False,
        "gps_fix_received": False,
        "heading_ned_deg": 0.0,
        "battery_v": 0.0,
        "battery_pct": 0.0,
        "lat": 0.0,
        "lon": 0.0,
        "alt": 0.0,
        "gps_fix": 0,
        "gps_sat": 0,
        "hrms": 0.0,
        "vrms": 0.0,
        "xtrack_m": 0.0,
        "heading_err_deg": 0.0,
        "lookahead_m": 0.0,
        "speed_m_s": 0.0,
        "kappa": 0.0,
        "dist_to_goal_m": 0.0,
        "pose_age_ms": 0.0,
        "rpp_state": 0,
        "v_north": 0.0,
        "v_east": 0.0,
        "yaw_rate_rad_s": 0.0,
        # B1 — predictive κ and pre-clamp Ld for tuning analysis
        "l_d_raw_m": 0.0,
        "kappa_speed": 0.0,
        "spraying": False,
        "spray_active": False,
        "spray_manual": False,
        "measured_speed_m_s": None,
        "rpp_debug_age_ms": None,
        "rpp_debug_fresh": False,
        "obstacle_clear": True,
    }

    def __init__(self) -> None:
        super().__init__("fastapi_bridge")
        self._lock = threading.Lock()
        self._state: dict[str, Any] = dict(self._DEFAULT_STATE)
        self._obstacle_callback: Callable[[bool], None] | None = None
        self._rpp_monitor = RppStatusMonitor()
        # Track last time /mavros/state was received.
        # TRANSIENT_LOCAL means a MAVROS process crash produces no new
        # messages; connected stays "True" forever from cached value.
        # We expose this timestamp so callers can detect true process death.
        self._state_recv_time: float | None = None
        self._pose_recv_time: float | None = None  # last /mavros/local_position/pose
        self._global_pos_recv_time: float | None = None
        self._gps_fix_recv_time: float | None = None
        self._velocity_recv_time: float | None = None
        self._MAVROS_STATE_TIMEOUT_S = 10.0  # PX4 HB at 1 Hz; observed gaps up to 7s on USB serial

        # Callback groups: subs mutually exclusive, services reentrant
        self._sub_group = MutuallyExclusiveCallbackGroup()
        self._svc_group = ReentrantCallbackGroup()

        if not _HAS_MAVROS:
            log.warning("mavros_msgs not available — running without MAVROS topics")

        # ── Subscribers ───────────────────────────────────────────────────────
        if _HAS_MAVROS:
            self.create_subscription(
                State,
                "/mavros/state",
                self._cb_state,
                _qos_reliable_tl(),
                callback_group=self._sub_group,
            )
            self.create_subscription(
                PoseStamped,
                "/mavros/local_position/pose",
                self._cb_pose,
                _qos_best_effort(),
                callback_group=self._sub_group,
            )
            self.create_subscription(
                BatteryState,
                "/mavros/battery",
                self._cb_battery,
                _qos_best_effort(),
                callback_group=self._sub_group,
            )
            self.create_subscription(
                NavSatFix,
                "/mavros/global_position/global",
                self._cb_global_pos,
                _qos_best_effort(),
                callback_group=self._sub_group,
            )
            if _HAS_GPSRAW:
                self.create_subscription(
                    GPSRAW,
                    "/mavros/gpsstatus/gps1/raw",
                    self._cb_gps_raw,
                    _qos_best_effort(),
                    callback_group=self._sub_group,
                )

        self.create_subscription(
            Float32MultiArray,
            "/rpp/debug",
            self._cb_rpp_debug,
            _qos_best_effort(),
            callback_group=self._sub_group,
        )
        self.create_subscription(
            Vector3Stamped,
            "/rpp/velocity_ned",
            self._cb_rpp_velocity,
            _qos_best_effort(),
            callback_group=self._sub_group,
        )
        if _HAS_MAVROS:
            self.create_subscription(
                TwistStamped,
                "/mavros/local_position/velocity_local",
                self._cb_velocity_local,
                _qos_best_effort(),
                callback_group=self._sub_group,
            )
        self.create_subscription(
            Bool,
            "/spray/state",
            self._cb_spray_state,
            _qos_best_effort(),
            callback_group=self._sub_group,
        )
        self.create_subscription(
            String,
            RUNTIME_STATUS_TOPIC,
            self._cb_spray_runtime_status,
            _qos_best_effort(),
            callback_group=self._sub_group,
        )
        self.create_subscription(
            Bool,
            "/spray/manual_state",
            self._cb_spray_manual_state,
            _qos_best_effort(),
            callback_group=self._sub_group,
        )
        self.create_subscription(
            Bool,
            "/rover/obstacle_clear",
            self._cb_obstacle_clear,
            _qos_best_effort(),
            callback_group=self._sub_group,
        )

        # ── Publishers ────────────────────────────────────────────────────────
        self._path_pub = self.create_publisher(Path, "/path", _qos_reliable_tl())
        self._path_identity_pub = self.create_publisher(
            String, PATH_IDENTITY_TOPIC, _qos_reliable_tl()
        )
        # Manual spray override command — reliable VOLATILE (depth 1): must
        # arrive, but a stale override must never replay to a restarted node.
        self._spray_manual_pub = self.create_publisher(Bool, "/spray/manual", 1)
        # F-01/F-02: feed the spray node's independent GPS_SURVEYED runtime gate.
        self._gps_gate_pub = self.create_publisher(String, "/spray/gps_gate", 1)

        # ── Service clients (reentrant group, can be called from any thread) ──
        self._arming_cli = None
        self._set_mode_cli = None
        self._param_get_cli = None
        self._param_set_cli = None
        if _HAS_MAVROS:
            self._arming_cli = self.create_client(
                CommandBool, "/mavros/cmd/arming", callback_group=self._svc_group
            )
            self._set_mode_cli = self.create_client(
                SetMode, "/mavros/set_mode", callback_group=self._svc_group
            )
        if _HAS_PARAM_SRV:
            self._param_get_cli = self.create_client(
                GetParameters,
                "/mavros/param/get_parameters",
                callback_group=self._svc_group,
            )
            self._param_set_cli = self.create_client(
                SetParameters,
                "/mavros/param/set_parameters",
                callback_group=self._svc_group,
            )

        # ── RPP controller param service clients ──────────────────────────────
        # These talk to the running rpp_controller node via standard ROS2
        # rcl_interfaces services. The controller starts independently and may
        # not be up when the bridge starts; readiness is checked on demand.
        self._rpp_param_get_cli: GetParameters.Request | None = None
        self._rpp_param_set_cli: SetParameters.Request | None = None
        self._rpp_param_list_cli: ListParameters.Request | None = None
        if _HAS_PARAM_SRV:
            self._rpp_param_get_cli = self.create_client(
                GetParameters,
                SRV_RPP_GET_PARAMS,
                callback_group=self._svc_group,
            )
            self._rpp_param_set_cli = self.create_client(
                SetParameters,
                SRV_RPP_SET_PARAMS,
                callback_group=self._svc_group,
            )
            self._rpp_param_list_cli = self.create_client(
                ListParameters,
                SRV_RPP_LIST_PARAMS,
                callback_group=self._svc_group,
            )

        # ── Spray controller param service clients ────────────────────────────
        self._spray_param_get_cli = None
        self._spray_param_set_cli = None
        self._spray_apply_cli = None
        self._spray_start_dwell_cli = None
        self._spray_cancel_dwell_cli = None
        self._spray_runtime_status: dict[str, Any] = {}
        self._spray_runtime_status_recv_time: float | None = None
        self._spray_dwell_revision = 0
        if _HAS_PARAM_SRV:
            self._spray_param_get_cli = self.create_client(
                GetParameters,
                SRV_SPRAY_GET_PARAMS,
                callback_group=self._svc_group,
            )
            self._spray_param_set_cli = self.create_client(
                SetParameters,
                SRV_SPRAY_SET_PARAMS,
                callback_group=self._svc_group,
            )
        try:
            from std_srvs.srv import Trigger

            self._spray_apply_cli = self.create_client(
                Trigger,
                SRV_SPRAY_APPLY_MISSION_CONFIG,
                callback_group=self._svc_group,
            )
            self._spray_start_dwell_cli = self.create_client(
                Trigger,
                SRV_SPRAY_START_DWELL,
                callback_group=self._svc_group,
            )
            self._spray_cancel_dwell_cli = self.create_client(
                Trigger,
                SRV_SPRAY_CANCEL_DWELL,
                callback_group=self._svc_group,
            )
        except ImportError:
            Trigger = None  # type: ignore

        # Do not wait for services here: RosBridgeNode is constructed
        # inside FastAPI lifespan, so startup service discovery must not block
        # the asyncio loop. Request paths perform a fail-fast readiness check.
        log.info("RosBridgeNode initialised; service readiness checked fail-fast")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cb_state(self, msg) -> None:
        self._state_recv_time = time.monotonic()
        with self._lock:
            self._state["armed"] = msg.armed
            self._state["mode"] = msg.mode
            self._state["connected"] = msg.connected

    def _cb_pose(self, msg) -> None:
        """ENU (MAVROS REP-103) → NED conversion."""
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_enu = math.atan2(siny_cosp, cosy_cosp)
        yaw_ned = math.pi / 2.0 - yaw_enu
        yaw_ned = math.atan2(math.sin(yaw_ned), math.cos(yaw_ned))
        with self._lock:
            self._pose_recv_time = time.monotonic()
            self._state["pos_n"] = msg.pose.position.y  # ENU y = North → pos_n
            self._state["pos_e"] = msg.pose.position.x  # ENU x = East  → pos_e
            self._state["pose_received"] = True
            self._state["heading_ned_deg"] = math.degrees(yaw_ned)

    def _cb_battery(self, msg) -> None:
        pct = msg.percentage
        if pct is not None and 0.0 <= pct <= 1.0:
            pct = pct * 100.0
        with self._lock:
            self._state["battery_v"] = msg.voltage
            self._state["battery_pct"] = pct if pct is not None else 0.0

    def _cb_global_pos(self, msg) -> None:
        hrms = 0.0
        vrms = 0.0
        try:
            cov = msg.position_covariance
            hrms = round(math.sqrt(abs(cov[0]) + abs(cov[4])), 3)
            vrms = round(math.sqrt(abs(cov[8])), 3)
        except (ValueError, IndexError, TypeError):
            pass

        with self._lock:
            self._global_pos_recv_time = time.monotonic()
            self._state["lat"] = msg.latitude
            self._state["lon"] = msg.longitude
            self._state["alt"] = msg.altitude
            self._state["hrms"] = hrms
            self._state["vrms"] = vrms
            self._state["global_position_received"] = True

    def _cb_gps_raw(self, msg) -> None:
        with self._lock:
            self._gps_fix_recv_time = time.monotonic()
            self._state["gps_fix"] = msg.fix_type
            self._state["gps_sat"] = msg.satellites_visible
            self._state["gps_fix_received"] = True

    def _cb_rpp_debug(self, msg: Float32MultiArray) -> None:
        # /rpp/debug is append-only. Consume stable legacy fields first and
        # read newer fields only when present so old bag replays still work.
        if len(msg.data) >= 8:
            data = list(msg.data)
            self._rpp_monitor.update(data)
            with self._lock:
                self._state["xtrack_m"] = data[0]
                self._state["heading_err_deg"] = math.degrees(data[1])
                self._state["lookahead_m"] = data[2]
                self._state["speed_m_s"] = data[3]
                self._state["kappa"] = data[4]
                self._state["dist_to_goal_m"] = data[5]
                self._state["pose_age_ms"] = data[6]
                self._state["rpp_state"] = int(data[7])
                # B1 — only populate if the producer is the new version
                self._state["l_d_raw_m"] = data[8] if len(data) >= 9 else float("nan")
                self._state["kappa_speed"] = (
                    data[9] if len(data) >= 10 else float("nan")
                )
                if len(data) >= 40:
                    self._state["spray_active"] = data[39] > 0.5

    def _cb_rpp_velocity(self, msg: Vector3Stamped) -> None:
        with self._lock:
            self._state["v_north"] = msg.vector.x
            self._state["v_east"] = msg.vector.y

    def _cb_velocity_local(self, msg: TwistStamped) -> None:
        # MAVROS velocity_local is ENU; yaw rate CCW+ → NED CW+ via negation.
        with self._lock:
            self._velocity_recv_time = time.monotonic()
            self._state["measured_speed_m_s"] = math.hypot(
                float(msg.twist.linear.x), float(msg.twist.linear.y)
            )
            self._state["yaw_rate_rad_s"] = -float(msg.twist.angular.z)

    def _cb_spray_runtime_status(self, msg: String) -> None:
        try:
            status = deserialize_runtime_status(msg.data)
        except (TypeError, ValueError) as exc:
            log.warning("invalid spray runtime status ignored: %s", exc)
            return
        with self._lock:
            self._spray_runtime_status = status
            self._spray_runtime_status_recv_time = time.monotonic()

    def _cb_spray_state(self, msg: Bool) -> None:
        with self._lock:
            self._state["spraying"] = bool(msg.data)

    def _cb_spray_manual_state(self, msg: Bool) -> None:
        with self._lock:
            self._state["spray_manual"] = bool(msg.data)

    # ── Public API: spray manual override ────────────────────────────────────

    def publish_spray_manual(self, on: bool) -> None:
        """Command the spray_controller manual override (True=ON, False=cancel)."""
        msg = Bool()
        msg.data = bool(on)
        self._spray_manual_pub.publish(msg)
        log.info("published /spray/manual: %s", "ON" if on else "OFF")

    def publish_gps_gate(
        self, *, active: bool, ok: bool, reason: str, seq: int
    ) -> None:
        """Feed the spray node's independent GPS_SURVEYED runtime gate (F-01/F-02).

        Published every telemetry tick while a mission is RUNNING so the node can
        detect feed loss (server death) and fail-closed. `active` is True only for
        a GPS_SURVEYED continuous/dash mission; the node ignores the gate when
        active is False, so LOCAL_NED missions are never RTK-gated."""
        msg = String()
        msg.data = json.dumps(
            {
                "active": bool(active),
                "ok": bool(ok),
                "reason": str(reason or ""),
                "seq": int(seq),
            }
        )
        self._gps_gate_pub.publish(msg)

    # ── Public API: state ─────────────────────────────────────────────────────

    def get_state(self) -> dict[str, Any]:
        """Return a shallow copy of current telemetry state (thread-safe).

        The `connected` field is overridden to False if no /mavros/state
        message has been received within MAVROS_STATE_TIMEOUT_S, which
        catches the case where the MAVROS process dies (its last
        TRANSIENT_LOCAL State message stays cached with connected=True,
        but no new messages arrive to reflect the crash).
        """
        with self._lock:
            state = dict(self._state)
            pose_recv_time = self._pose_recv_time
            global_pos_recv_time = self._global_pos_recv_time
            gps_fix_recv_time = self._gps_fix_recv_time
            velocity_recv_time = self._velocity_recv_time
        now = time.monotonic()
        state["local_pose_age_ms"] = (
            (now - pose_recv_time) * 1000.0 if pose_recv_time is not None else None
        )
        state["global_position_age_ms"] = (
            (now - global_pos_recv_time) * 1000.0
            if global_pos_recv_time is not None else None
        )
        state["gps_fix_age_ms"] = (
            (now - gps_fix_recv_time) * 1000.0
            if gps_fix_recv_time is not None else None
        )
        state["velocity_age_ms"] = (
            (now - velocity_recv_time) * 1000.0
            if velocity_recv_time is not None else None
        )
        state["pose_global_skew_ms"] = (
            # Callback receive-time skew from monotonic clocks. This is not
            # sensor-time or ROS-header timestamp synchronization.
            abs(pose_recv_time - global_pos_recv_time) * 1000.0
            if pose_recv_time is not None and global_pos_recv_time is not None
            else None
        )
        # Outside the lock — monotonic check does not need it
        if self._state_recv_time is not None:
            age = time.monotonic() - self._state_recv_time
            if age > self._MAVROS_STATE_TIMEOUT_S:
                state["connected"] = False
        rpp_age_s = self._rpp_monitor.snapshot_age_s()
        state["rpp_debug_age_ms"] = (
            rpp_age_s * 1000.0 if rpp_age_s is not None else None
        )
        state["rpp_debug_fresh"] = self._rpp_monitor.is_fresh()
        return state

    def get_bridge_snapshot(self) -> dict[str, Any]:
        """Cheap, cached-only health view of the MAVROS bridge link.

        `state_age_ms` (freshness of /mavros/state) is the authoritative
        liveness signal — it goes stale when MAVROS dies or the FCU link
        drops, even while the TRANSIENT_LOCAL cached State still reads
        connected=True. `pose_age_ms` is INFORMATIONAL only: pose can be
        legitimately absent (EKF without a GPS/RTK solution) and must NOT
        be used to declare the bridge frozen.
        """
        with self._lock:
            armed = self._state.get("armed")
            mode = self._state.get("mode")
            connected_cached = self._state.get("connected", False)
        now = time.monotonic()
        state_age_ms = (
            (now - self._state_recv_time) * 1000.0
            if self._state_recv_time is not None
            else None
        )
        pose_age_ms = (
            (now - self._pose_recv_time) * 1000.0
            if self._pose_recv_time is not None
            else None
        )
        # True process-death-aware connected flag (mirror get_state override).
        fcu_connected = bool(connected_cached)
        if state_age_ms is not None and state_age_ms > self._MAVROS_STATE_TIMEOUT_S * 1000.0:
            fcu_connected = False
        try:
            mavros_state_publishers = self.count_publishers("/mavros/state")
        except Exception:
            mavros_state_publishers = -1  # unknown (graph query failed)
        return {
            "fcu_connected": fcu_connected,
            "state_age_ms": state_age_ms,
            "pose_age_ms": pose_age_ms,
            "mavros_state_publishers": mavros_state_publishers,
            "armed": armed,
            "mode": mode,
        }

    def get_rpp_monitor(self) -> RppStatusMonitor:
        return self._rpp_monitor

    # ── Public API: async service wrappers ────────────────────────────────────

    async def _call_async(
        self,
        cli,
        request,
        timeout: float,
        success_attr: str,
    ) -> tuple[bool, str]:
        """Common async-friendly wrapper for any rclpy service client.

        Returns (ok, message). `ok` reflects future completion AND the
        success flag (`success_attr`) on the response. `message` is empty
        on success or a short diagnostic on failure.
        """
        if cli is None:
            return False, "service client not available"
        if not await self._service_ready_async(cli, timeout_sec=0.5):
            return False, f"service {cli.srv_name} not ready"

        future = cli.call_async(request)

        try:
            result = await self._await_ros_future(future, timeout=timeout)
        except asyncio.TimeoutError:
            return False, f"service {cli.srv_name} timed out after {timeout}s"
        except Exception as exc:
            return False, f"service {cli.srv_name} raised: {exc}"

        if result is None:
            return False, "service returned None"
        flag = getattr(result, success_attr, None)
        if flag is None:
            # No success attr — treat presence of result as success
            return True, ""
        return bool(flag), "" if flag else f"service rejected (success={flag})"

    async def _service_ready_async(self, cli, timeout_sec: float = 0.5) -> bool:
        """Fail-fast service readiness check for command/control paths."""
        if cli is None:
            return False
        return bool(cli.service_is_ready())

    async def _await_ros_future(self, future, timeout: float):
        """Await an rclpy Future from asyncio without late-result races.

        `asyncio.wait_for()` cancels the asyncio-side future on timeout. ROS
        service responses can still arrive later, so the callback must not call
        set_result/set_exception on an already-done Future.
        """
        loop = asyncio.get_running_loop()
        af: asyncio.Future = loop.create_future()

        def _done_cb(f) -> None:
            def _complete_result(result) -> None:
                if not af.done():
                    af.set_result(result)

            def _complete_exception(exc: BaseException) -> None:
                if not af.done():
                    af.set_exception(exc)

            try:
                result = f.result()
            except Exception as exc:
                loop.call_soon_threadsafe(_complete_exception, exc)
                return
            loop.call_soon_threadsafe(_complete_result, result)

        future.add_done_callback(_done_cb)
        try:
            return await asyncio.wait_for(af, timeout=timeout)
        except asyncio.TimeoutError:
            af.cancel()
            raise

    async def arm_async(self, arm: bool, timeout: float = 5.0) -> tuple[bool, str]:
        if self._arming_cli is None:
            return False, "mavros not available"
        req = CommandBool.Request()
        req.value = arm
        return await self._call_async(self._arming_cli, req, timeout, "success")

    async def set_mode_async(self, mode: str, timeout: float = 5.0) -> tuple[bool, str]:
        if self._set_mode_cli is None:
            return False, "mavros not available"
        req = SetMode.Request()
        req.custom_mode = mode
        return await self._call_async(self._set_mode_cli, req, timeout, "mode_sent")

    async def get_param_async(
        self, name: str, timeout: float = 5.0
    ) -> tuple[bool, Any, str]:
        """Returns (ok, value, message). value is None when ok=False."""
        if self._param_get_cli is None:
            return False, None, "param service not available"
        req = GetParameters.Request()
        req.names = [name]
        if not await self._service_ready_async(self._param_get_cli, timeout_sec=0.5):
            return False, None, "param get service not ready"

        try:
            result = await self._await_ros_future(
                self._param_get_cli.call_async(req), timeout=timeout
            )
        except asyncio.TimeoutError:
            return False, None, "param get timed out"
        except Exception as exc:
            return False, None, f"param get failed: {exc}"
        if result is None or not result.values:
            return False, None, "param not found"
        return True, _param_value_to_python(result.values[0]), ""

    async def set_param_async(
        self, name: str, value: float | int | bool | str, timeout: float = 5.0
    ) -> tuple[bool, str]:
        if self._param_set_cli is None:
            return False, "param service not available"
        req = SetParameters.Request()
        param = Parameter()
        param.name = name
        param.value = _python_to_param_value(value)
        req.parameters = [param]

        ok, _, msg = await self._call_set_param(req, timeout)
        return ok, msg

    async def _call_set_param(self, req, timeout: float) -> tuple[bool, list, str]:
        if not await self._service_ready_async(self._param_set_cli, timeout_sec=0.5):
            return False, [], "param set service not ready"
        try:
            result = await self._await_ros_future(
                self._param_set_cli.call_async(req), timeout=timeout
            )
        except asyncio.TimeoutError:
            return False, [], "param set timed out"
        except Exception as exc:
            return False, [], f"param set failed: {exc}"
        if result is None:
            return False, [], "param set returned None"
        results = list(result.results)
        if results and not results[0].successful:
            return False, results, results[0].reason or "param set rejected"
        return True, results, ""

    # ── Public API: RPP controller params (via rcl_interfaces) ───────────────

    async def get_rpp_param_async(
        self, name: str, timeout: float = 5.0
    ) -> tuple[bool, Any, str]:
        """Returns (ok, value, message) for a single RPP controller param."""
        if self._rpp_param_get_cli is None:
            return False, None, "RPP param service not available"
        req = GetParameters.Request()
        req.names = [name]
        if not await self._service_ready_async(
            self._rpp_param_get_cli, timeout_sec=0.5
        ):
            return False, None, "RPP controller not running"
        try:
            result = await self._await_ros_future(
                self._rpp_param_get_cli.call_async(req), timeout=timeout
            )
        except asyncio.TimeoutError:
            return False, None, "RPP param get timed out"
        except Exception as exc:
            return False, None, f"RPP param get failed: {exc}"
        if result is None or not result.values:
            return False, None, f"param '{name}' not found on RPP controller"
        return True, _param_value_to_python(result.values[0]), ""

    async def get_rpp_params_bulk_async(
        self, names: list[str], timeout: float = 5.0
    ) -> tuple[bool, dict[str, Any], str]:
        """Returns (ok, {name: value, ...}, message) for multiple params."""
        if self._rpp_param_get_cli is None:
            return False, {}, "RPP param service not available"
        req = GetParameters.Request()
        req.names = names
        if not await self._service_ready_async(
            self._rpp_param_get_cli, timeout_sec=0.5
        ):
            return False, {}, "RPP controller not running"
        try:
            result = await self._await_ros_future(
                self._rpp_param_get_cli.call_async(req), timeout=timeout
            )
        except asyncio.TimeoutError:
            return False, {}, "RPP param get timed out"
        except Exception as exc:
            return False, {}, f"RPP param get failed: {exc}"
        if result is None:
            return False, {}, "RPP param get returned None"
        if len(result.values) != len(names):
            log.warning(
                "RPP bulk get: expected %d values, got %d — some params missing",
                len(names),
                len(result.values),
            )
        values = {}
        for n, v in zip(names, result.values):
            values[n] = _param_value_to_python(v)
        return True, values, ""

    async def set_rpp_param_async(
        self, name: str, value: float | int | bool | str, timeout: float = 5.0
    ) -> tuple[bool, str]:
        """Set a single RPP controller parameter at runtime."""
        if self._rpp_param_set_cli is None:
            return False, "RPP param service not available"
        req = SetParameters.Request()
        param = Parameter()
        param.name = name
        param.value = _python_to_param_value(value)
        req.parameters = [param]
        ok, _, msg = await self._call_rpp_set_param(req, timeout)
        return ok, msg

    async def set_rpp_params_bulk_async(
        self, params: dict[str, float | int | bool | str], timeout: float = 5.0
    ) -> tuple[bool, list[bool], str]:
        """Set multiple RPP controller params atomically.

        Returns (ok, per_param_success_flags, message). When one param fails
        the entire batch is rejected by the RPP controller.
        """
        if self._rpp_param_set_cli is None:
            return False, [], "RPP param service not available"
        req = SetParameters.Request()
        for name, value in params.items():
            param = Parameter()
            param.name = name
            param.value = _python_to_param_value(value)
            req.parameters.append(param)
        ok, results, msg = await self._call_rpp_set_param(req, timeout)
        flags = [r.successful for r in results] if results else []
        return ok, flags, msg

    async def _call_rpp_set_param(self, req, timeout: float) -> tuple[bool, list, str]:
        """Shared rcl SetParameters call wrapper for RPP controller."""
        if not await self._service_ready_async(
            self._rpp_param_set_cli, timeout_sec=0.5
        ):
            return False, [], "RPP controller not running"
        try:
            result = await self._await_ros_future(
                self._rpp_param_set_cli.call_async(req), timeout=timeout
            )
        except asyncio.TimeoutError:
            return False, [], "RPP param set timed out"
        except Exception as exc:
            return False, [], f"RPP param set failed: {exc}"
        if result is None:
            return False, [], "RPP param set returned None"
        results = list(result.results)
        if results and not results[0].successful:
            return False, results, results[0].reason or "RPP param set rejected"
        return True, results, ""

    # ── Spray controller param access ─────────────────────────────────────────

    async def get_spray_param_async(
        self, name: str, timeout: float = 5.0
    ) -> tuple[bool, Any, str]:
        """Returns (ok, value, message) for a single spray_controller param."""
        if self._spray_param_get_cli is None:
            return False, None, "Spray param service not available"
        req = GetParameters.Request()
        req.names = [name]
        if not await self._service_ready_async(
            self._spray_param_get_cli, timeout_sec=0.5
        ):
            return False, None, "spray_controller not running"
        try:
            result = await self._await_ros_future(
                self._spray_param_get_cli.call_async(req), timeout=timeout
            )
        except asyncio.TimeoutError:
            return False, None, "Spray param get timed out"
        except Exception as exc:
            return False, None, f"Spray param get failed: {exc}"
        if result is None or not result.values:
            return False, None, f"param '{name}' not found on spray_controller"
        return True, _param_value_to_python(result.values[0]), ""

    async def get_spray_params_bulk_async(
        self, names: list[str], timeout: float = 5.0
    ) -> tuple[bool, dict[str, Any], str]:
        """Returns (ok, {name: value, ...}, message) for multiple spray params."""
        if self._spray_param_get_cli is None:
            return False, {}, "Spray param service not available"
        req = GetParameters.Request()
        req.names = names
        if not await self._service_ready_async(
            self._spray_param_get_cli, timeout_sec=0.5
        ):
            return False, {}, "spray_controller not running"
        try:
            result = await self._await_ros_future(
                self._spray_param_get_cli.call_async(req), timeout=timeout
            )
        except asyncio.TimeoutError:
            return False, {}, "Spray param get timed out"
        except Exception as exc:
            return False, {}, f"Spray param get failed: {exc}"
        if result is None:
            return False, {}, "Spray param get returned None"
        values = {}
        for n, v in zip(names, result.values):
            values[n] = _param_value_to_python(v)
        return True, values, ""

    async def set_spray_param_async(
        self, name: str, value: float | int | bool | str, timeout: float = 8.0
    ) -> tuple[bool, str]:
        """Set a single spray_controller parameter at runtime."""
        req = SetParameters.Request()
        param = Parameter()
        param.name = name
        param.value = _python_to_param_value(value)
        req.parameters = [param]
        ok, _, msg = await self._call_spray_set_param(
            req, response_timeout_s=timeout
        )
        return ok, msg

    async def set_spray_params_bulk_async(
        self, params: dict[str, float | int | bool | str], timeout: float = 8.0
    ) -> tuple[bool, list[bool], str]:
        """Set multiple spray_controller params atomically."""
        req = SetParameters.Request()
        for name, value in params.items():
            param = Parameter()
            param.name = name
            param.value = _python_to_param_value(value)
            req.parameters.append(param)
        ok, results, msg = await self._call_spray_set_param(
            req, response_timeout_s=timeout
        )
        flags = [r.successful for r in results] if results else []
        return ok, flags, msg

    async def trigger_spray_apply_mission_config_async(
        self, timeout: float = 5.0
    ) -> tuple[bool, str]:
        if self._spray_apply_cli is None:
            return False, "spray apply service not available"
        from std_srvs.srv import Trigger

        req = Trigger.Request()
        if not await self._service_ready_async(self._spray_apply_cli, timeout_sec=0.5):
            return False, "spray_controller apply service not running"
        try:
            result = await self._await_ros_future(
                self._spray_apply_cli.call_async(req), timeout=timeout
            )
        except asyncio.TimeoutError:
            return False, "spray apply_mission_config timed out"
        except Exception as exc:
            return False, f"spray apply_mission_config failed: {exc}"
        if result is None:
            return False, "spray apply_mission_config returned None"
        if not bool(result.success):
            return False, result.message or "spray apply_mission_config rejected"
        return True, result.message or "ok"

    async def start_spray_dwell_async(
        self,
        *,
        mission_id: str,
        point_index: int,
        duration_s: float,
        command_id: int,
        configuration_revision: int,
        timeout: float = 5.0,
    ) -> tuple[bool, str]:
        if self._spray_start_dwell_cli is None or self._spray_param_set_cli is None:
            return False, "spray dwell service not available"
        from std_srvs.srv import Trigger

        with self._lock:
            self._spray_dwell_revision = max(
                self._spray_dwell_revision + 1, time.monotonic_ns()
            )
            revision = self._spray_dwell_revision
        params = {"pending_dwell_command_json": serialize_dwell_command(
            revision=revision,
            mission_id=mission_id,
            point_index=point_index,
            command_id=command_id,
            duration_s=duration_s,
            configuration_revision=configuration_revision,
        )}
        ok, _, why = await self.set_spray_params_bulk_async(params, timeout=timeout)
        if not ok:
            return False, why or "failed to stage dwell parameters"
        req = Trigger.Request()
        if not await self._service_ready_async(self._spray_start_dwell_cli, timeout_sec=0.5):
            return False, "spray start_dwell service not running"
        try:
            result = await self._await_ros_future(
                self._spray_start_dwell_cli.call_async(req), timeout=timeout
            )
        except asyncio.TimeoutError:
            return False, "spray start_dwell timed out"
        except Exception as exc:
            return False, f"spray start_dwell failed: {exc}"
        if result is None:
            return False, "spray start_dwell returned None"
        if not bool(result.success):
            return False, result.message or "spray start_dwell rejected"
        try:
            accepted = parse_dwell_response(result.message)
        except (TypeError, ValueError) as exc:
            return False, f"invalid dwell acceptance response: {exc}"
        if int(accepted.get("command_id", -1)) != command_id:
            return False, "dwell acceptance command mismatch"
        return True, result.message

    async def cancel_spray_dwell_async(self, timeout: float = 5.0) -> tuple[bool, str]:
        if self._spray_cancel_dwell_cli is None or self._spray_param_set_cli is None:
            return False, "spray cancel dwell service not available"
        from std_srvs.srv import Trigger

        with self._lock:
            self._spray_dwell_revision = max(
                self._spray_dwell_revision + 1, time.monotonic_ns()
            )
            cancel_revision = self._spray_dwell_revision
        ok, _, why = await self.set_spray_params_bulk_async(
            {"dwell_cancel_revision": cancel_revision}, timeout=timeout
        )
        if not ok:
            return False, why or "failed to invalidate prepared dwell commands"
        req = Trigger.Request()
        if not await self._service_ready_async(self._spray_cancel_dwell_cli, timeout_sec=0.5):
            return False, "spray cancel_dwell service not running"
        try:
            result = await self._await_ros_future(
                self._spray_cancel_dwell_cli.call_async(req), timeout=timeout
            )
        except asyncio.TimeoutError:
            return False, "spray cancel_dwell timed out"
        except Exception as exc:
            return False, f"spray cancel_dwell failed: {exc}"
        if result is None:
            return False, "spray cancel_dwell returned None"
        return bool(result.success), result.message or "ok"

    def get_spray_runtime_status(self) -> dict[str, Any]:
        with self._lock:
            status = dict(self._spray_runtime_status)
            recv_time = self._spray_runtime_status_recv_time
        age_s = time.monotonic() - recv_time if recv_time is not None else float("inf")
        status["status_age_s"] = age_s
        status["status_stale"] = age_s > RUNTIME_STATUS_MAX_AGE_S
        return status

    def update_spray_runtime_status(self, status: dict[str, Any]) -> None:
        """Test hook; production updates arrive only from the ROS subscriber."""
        with self._lock:
            self._spray_runtime_status = dict(status)
            self._spray_runtime_status_recv_time = time.monotonic()

    async def _call_spray_set_param(
        self,
        req,
        service_wait_timeout_s: float = 2.0,
        response_timeout_s: float = 8.0,
    ) -> tuple[bool, list, str]:
        """Shared rcl SetParameters call wrapper for spray_controller."""
        client = self._spray_param_set_cli
        if client is None:
            return False, [], "Spray parameter client is not initialized"

        try:
            service_ready = await asyncio.to_thread(
                lambda: client.wait_for_service(timeout_sec=service_wait_timeout_s)
            )
        except Exception as exc:
            log.warning(
                "spray parameter service check failed: %s", exc, exc_info=True
            )
            return False, [], f"Spray parameter service check failed: {exc}"

        if not service_ready:
            log.warning(
                "spray parameter service unavailable after %.1fs",
                service_wait_timeout_s,
            )
            return (
                False,
                [],
                f"Spray parameter service unavailable after "
                f"{service_wait_timeout_s:.1f}s",
            )

        t0 = time.monotonic()
        try:
            ros_future = client.call_async(req)
            response = await self._await_ros_future(
                ros_future, timeout=response_timeout_s
            )
        except asyncio.TimeoutError:
            latency_s = time.monotonic() - t0
            log.warning(
                "spray parameter response timed out after %.1fs (latency_s=%.3f)",
                response_timeout_s,
                latency_s,
            )
            return (
                False,
                [],
                f"Spray parameter response timed out after "
                f"{response_timeout_s:.1f}s",
            )
        except Exception as exc:
            latency_s = time.monotonic() - t0
            log.warning(
                "spray parameter call failed after %.3fs: %s",
                latency_s,
                exc,
                exc_info=True,
            )
            return False, [], f"Spray parameter call failed: {exc}"

        latency_s = time.monotonic() - t0
        if response is None:
            log.warning(
                "spray parameter service returned no response (latency_s=%.3f)",
                latency_s,
            )
            return False, [], "Spray parameter service returned no response"

        results = list(response.results)
        failed_reasons = [
            result.reason or "parameter rejected"
            for result in results
            if not result.successful
        ]
        if failed_reasons:
            log.warning(
                "spray parameter rejected (latency_s=%.3f): %s",
                latency_s,
                "; ".join(failed_reasons),
            )
            return False, results, "; ".join(failed_reasons)

        log.debug("spray parameter set succeeded (latency_s=%.3f)", latency_s)
        return True, results, ""

    async def list_rpp_params_async(
        self, timeout: float = 5.0
    ) -> tuple[bool, list[str], str]:
        """List all parameter names on the RPP controller node."""
        if self._rpp_param_list_cli is None:
            return False, [], "RPP param service not available"
        req = ListParameters.Request()
        req.depth = 0  # 0 = unlimited recursion (flat list)
        if not await self._service_ready_async(
            self._rpp_param_list_cli, timeout_sec=0.5
        ):
            return False, [], "RPP controller not running"
        try:
            result = await self._await_ros_future(
                self._rpp_param_list_cli.call_async(req), timeout=timeout
            )
        except asyncio.TimeoutError:
            return False, [], "RPP list params timed out"
        except Exception as exc:
            return False, [], f"RPP list params failed: {exc}"
        if result is None:
            return False, [], "RPP list params returned None"
        if result.result is None:
            return False, [], "RPP list params returned null result (service bug)"
        names = list(result.result.names)
        return True, names, ""

    # ── Public API: path publishing ───────────────────────────────────────────

    def publish_path(
        self,
        points: list[tuple[float, float]],
        frame_id: str = "local_ned",
        spray_flags: list[bool] | None = None,
        runtime_entry: bool = False,
        mission_id: str = "",
        configuration_revision: int = 0,
        path_fingerprint: str = "",
        verify_supplied_fingerprint: bool = True,
    ) -> None:
        """Publish nav_msgs/Path. Empty list → see publish_stop_path()."""
        points = normalize_path_points(points, label="ROS path")
        if spray_flags is None:
            flags = [False] * len(points)
        elif len(spray_flags) != len(points):
            log.warning(
                "publish_path: spray_flags length %d != points length %d — forcing all OFF",
                len(spray_flags),
                len(points),
            )
            flags = [False] * len(points)
        else:
            flags = normalize_spray_flags(spray_flags, len(points), default=False)
        if path_fingerprint and not verify_supplied_fingerprint:
            fingerprint = str(path_fingerprint)
        else:
            fingerprint = verified_path_fingerprint(points, flags, path_fingerprint)

        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = frame_id
        for index, ((n, e), spray) in enumerate(zip(points, flags)):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(n)
            ps.pose.position.y = float(e)
            ps.pose.position.z = 1.0 if spray else 0.0
            if runtime_entry and index == 0:
                # Explicit marker consumed only by RPP conditioning. Position z
                # remains the spray channel, so the spray controller sees OFF.
                ps.pose.orientation.x = 1.0
                ps.pose.orientation.w = 0.0
            else:
                ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        ident = String()
        ident.data = make_path_identity(
            mission_id=mission_id,
            path_fingerprint=fingerprint,
            configuration_revision=configuration_revision,
            source="raw_path",
        )
        self._path_identity_pub.publish(ident)
        self._path_pub.publish(path)
        log.info(
            "published path: %d points → %s (spray_on=%d, mission_id=%s)",
            len(points),
            frame_id,
            sum(1 for f in flags if f),
            mission_id or "",
        )

    def publish_path_clear(self, frame_id: str = "local_ned") -> None:
        """Publish an explicit invalid path/identity reset for latched topics."""
        ident = String()
        ident.data = make_path_identity(
            mission_id="",
            path_fingerprint="",
            configuration_revision=0,
            source="clear",
        )
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = frame_id
        self._path_identity_pub.publish(ident)
        self._path_pub.publish(path)
        log.info("published latched path clear")

    def set_obstacle_callback(self, cb: Callable[[bool], None] | None) -> None:
        """Register orchestrator hook for ``/rover/obstacle_clear`` updates."""
        self._obstacle_callback = cb

    def _cb_obstacle_clear(self, msg: Bool) -> None:
        clear = bool(msg.data)
        with self._lock:
            self._state["obstacle_clear"] = clear
        if self._obstacle_callback is not None:
            try:
                self._obstacle_callback(clear)
            except Exception:
                log.exception("obstacle callback raised")

    def publish_stop_path(
        self, frame_id: str = "local_ned"
    ) -> tuple[float, float] | None:
        """Publish a single-point path at the rover's current NED position.

        Workaround for the upstream RPP node that ignores empty-path messages.
        A single-point path is treated as DONE on the first control tick (the
        rover is already within `xy_goal_tolerance` of itself), so RPP zeroes
        its velocity output. This is the safe `mission_stop` semantic.

        Guard: if the server has never received a pose, publishing at origin
        (0,0) could issue an unintended movement command if the rover is not
        actually at the EKF origin. In that case we publish nothing, log a
        warning, and return None. `set_mode_async("MANUAL")` in the abort
        chain still fires, which is the actual safety net.
        """
        s = self.get_state()
        n, e = float(s.get("pos_n", 0.0)), float(s.get("pos_e", 0.0))
        if not s.get("pose_received", False):
            log.warning(
                "publish_stop_path: no pose received yet — "
                "no stop-path published"
            )
            return None
        self.publish_path([(n, e)], frame_id=frame_id)
        log.info("published stop-path at (N=%.2f, E=%.2f)", n, e)
        return (n, e)


# ── Param value <-> Python helpers ────────────────────────────────────────────


def _param_value_to_python(pv) -> Any:
    if not _HAS_PARAM_SRV:
        return None
    t = pv.type
    if t == ParameterType.PARAMETER_BOOL:
        return bool(pv.bool_value)
    if t == ParameterType.PARAMETER_INTEGER:
        return int(pv.integer_value)
    if t == ParameterType.PARAMETER_DOUBLE:
        return float(pv.double_value)
    if t == ParameterType.PARAMETER_STRING:
        return str(pv.string_value)
    return None


def _python_to_param_value(value: Any):
    if not _HAS_PARAM_SRV:
        raise RuntimeError("param services not available")
    pv = ParameterValue()
    if isinstance(value, bool):
        pv.type = ParameterType.PARAMETER_BOOL
        pv.bool_value = value
    elif isinstance(value, int):
        pv.type = ParameterType.PARAMETER_INTEGER
        pv.integer_value = value
    elif isinstance(value, float):
        pv.type = ParameterType.PARAMETER_DOUBLE
        pv.double_value = value
    elif isinstance(value, str):
        pv.type = ParameterType.PARAMETER_STRING
        pv.string_value = value
    else:
        raise TypeError(f"Unsupported param value type: {type(value).__name__}")
    return pv
