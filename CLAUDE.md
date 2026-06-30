# 4WD Marking Rover — Jetson Companion

Scope: runtime, ROS2, MAVROS2, on-device debugging on Jetson Orin `192.168.1.101`.
Not your job: PX4 firmware, waypoint gen, log analysis — those live on Mac GCS at `/Users/dyx_a1/Vetri/3WD_GCS_transfer/3WD_GCS/`.

> **Cross-project memory:** `/Users/dyx_a1/Vetri/PX4-Autopilot/.claude/memory/integration.md`
> Load this when working on firmware↔companion integration, OFFBOARD interface, coordinate conventions, or open issues spanning both projects.

## Hardware

| Item | Value |
|---|---|
| IP | `192.168.1.101` (eno1), user `flash` |
| OS | Ubuntu aarch64, ROS2 Humble |
| FCU | `/dev/ttyACM0` @ 921600 (CubeOrangePlus, PX4 v1.16.2) |
| RTK | UM982 on TELEM1 — NTRIP via MAVROS |

## Service restart (narrowest scope)

| Changed | Restart | Drops MAVROS? |
|---|---|---|
| `src/*.py` | `sudo systemctl restart rpp-pipeline` | No (~2s) |
| `server/**` | `sudo systemctl restart rover-server` | No (~2s) |
| `px4_start_service.sh`, pluginlist, NTRIP | `sudo systemctl restart px4-dxp` | Yes (~11s) |
| `*.service` / new files | `./deploy.sh` (daemon-reload) | — |

`rpp-pipeline PartOf=px4-dxp` — px4-dxp restart cascades down; not up.

## Critical impl rules

- **E-stop:** publish current pos as single-point path — RPP ignores empty Path
- **Async only:** `arm_async()`, `set_mode_async()` use `call_async` + `add_done_callback`
- **ENU→NED (RPP input):** `yaw_NED = π/2 - yaw_ENU`, `pos_n = pose.y`, `pos_e = pose.x`
- **NED→ENU (twist output):** `vel.x = v_e`, `vel.y = v_n`, `vel.z = -v_d`
- **MAVROS crash detect:** TRANSIENT_LOCAL keeps stale `connected=True`; server overrides after 2s via `_state_recv_time`

## OFFBOARD rules

1. Stream setpoints ≥2 Hz **before** requesting OFFBOARD or PX4 rejects it
2. Gap >0.5s in OFFBOARD → PX4 exits to failsafe
3. Use `OFFBOARD` not `GUIDED`
4. Velocity: `/mavros/setpoint_velocity/cmd_vel` (TwistStamped)
5. Path/arc: `/mavros/setpoint_raw/local` (PositionTarget)

## GCS Machine (Mac)

| Item | Value |
|---|---|
| Host | MacBook Air, user `dyx_a1` |
| GCS frontend | DYX_GCS_V |
| GCS path | `/Users/dyx_a1/Vetri/3WD_GCS_transfer/3WD_GCS/` |
| SSH to Jetson | `ssh flash@192.168.1.101` |
| QGC | QGroundControl on macOS — UDP `192.168.1.101:14550` |

## Current status (2026-06-30)

- 4WD_SERVER repo created and deployed on new Jetson (`192.168.1.101`)
- QGC connected via UDP:14550 — FCU link confirmed
- Phase 2 OFFBOARD stack running; FastAPI + DYX_GCS_V mobile frontend built
- **Controller + tuning phase CLOSED & VALIDATED** — frozen at validated config (`@510be9b`+ bug fixes). Production tracking = **segment / stop-pivot profile**. Do not re-open arc PID/lookahead tuning unless a regression appears.
- **All 3 priority bugs FIXED + VALIDATED (2026-06-15)** via 11-bag campaign + `tools/validate_build.py`:
  - **BUG-T3** wrong-initial-turn — `fix(rpp): forward-cone clamp` (`510be9b`). PASS on all 11 incl. ~90° mis-headed starts (correct turn, no reverse).
  - **BUG-T2** stop-and-go at smooth/tangent junctions — `1af51ac` (`_apply_run` heading-delta gate + intrinsic `_segment_angle_deg`). U-turn flows continuously (0 stops, 0.97cm RMS).
  - **BUG-T1** stop-pivot oscillation — `036f116` (yaw-rate settle gates; `segment_debug` 9→10, `[9]`=actual yaw-rate). Validated `square_2x2_20260615_144019`: clean single-direction pivots, **1 significant reversal (>0.1)/0 large** (was 8/2 on 06-13). xtrack 0.53cm RMS.
- **Tracking @0.35 m/s — all shapes sub-2cm RMS:** arc 1.46 / lshape 0.90 / square 0.87 / U-turn 1.06 cm. ulogs clean (no clipping).
- **Arc (smooth RPP) — structural floor, DEFERRED** (not blocking; segment profile is production): velocity OFFBOARD discards `trajectory_setpoint.yawspeed`; pure-P attitude loop → following err `≈ ω/RO_YAW_P`. `RO_YAW_P` now **1.5** (lag ~9°). To beat 2cm on smooth curves: raise `RO_YAW_P` (QGC) OR `body_rate` offboard. Companion `yaw_rate_feedback_gain` is a NO-OP in velocity mode.
- Validated RPP params: `max_yaw_rate_body=0.45`, `a_lat_max=0.3`, `corner_smooth_radius_m=0.5`, `segment_heading_tolerance_deg=2.0`, `segment_stop_yaw_rate_threshold=0.05`, `segment_align_settle_s=0.10`. PX4: `RO_YAW_P=1.5`, `RO_YAW_RATE_LIM=30`, `EKF2_WENC_CTRL=1`, `RBCLW_COUNTS_REV=148000` (encoder fusion validated log_150). Full as-flown set in `PX4_DXP_Tracker.xlsx` → "PX4 FCU Params".
- **FUTURE — high-speed tuning (SPD-T1, backlog):** target 1.0 m/s line / 0.6 m/s arc (now 0.35). Prereq: `RO_MAX_THR_SPEED=0.9` ⇒ full throttle ≈0.9-0.95 m/s, so 1.0 has NO headroom — verify RoboClaw top speed first. Line/arc split is free via `a_lat_max` regulator (mission_speed=1.0 + a_lat_max≈0.24 → 1.0 straight / ~0.6 on R1.5). Watch: corner braking dist (slowdown_dist 0.5m too short at 1 m/s), arc heading lag grows (~15° at 0.6/R1.5), speed-loop overshoot. See tracker SPD-T1.
- Known minor (not blocking): pivots ~6s near 5s align watchdog, 2/3 exit ~0.17 rad/s residual; speed loop overshoots (~0.42 vs 0.35) — tighten `RO_SPEED_P/I` when convenient.
- **Auto-bag recorder LIVE** — `bag-autorecord.service` captures every API-started mission to `~/bags_jet` (start→complete). Validate via `tools/validate_build.py <dir>`.
- Tracking profiles live: `tracking_profile=auto|segment|smooth` — auto splits missions per-entity (spray-flag + hard-corner splits), lines→segment, arcs/circles→smooth, pivot-align at transitions.
- Phase 3 spray: **built, live & hardware-validated (2026-06-17)** — `spray_controller_node.py` drives PX4 AUX1 via `MAV_CMD_DO_SET_ACTUATOR` cmd 187 (MAVROS), safety-gated (armed+OFFBOARD, staleness watchdog, debounce); manual test via `POST /api/spray/test`.
  - **Validated spray config (SmartFLEX DC motor driver on AUX1):**
    - `actuator_backend = mavlink_actuator` (cmd 187, normalized)
    - `on_value = 1.0` → maps to `PWM_AUX_MAX1` (full flow). **`on_value` is a NORMALIZED actuator value, range −1.0…+1.0 only — NOT PWM µs and it sets no PX4 param.** Strength is the QGC `PWM_AUX_MAX1` knob below, not `on_value`. (`on_pwm_us`/`off_pwm_us` are dead in this backend; only the unused `mavlink_servo_pwm`/cmd-183 backend reads them.)
    - `off_value = -1.0` → 0 µs (motor fully stopped, field-confirmed)
  - **Required QGC FCU params (source of truth — set via QGC, not Jetson):**
    - `PWM_AUX_FUNC1 = 301` (RC AUX passthrough)
    - `PWM_AUX_MIN1 = 0` (was 1000 — changed so OFF idles at 0 µs, not 1000 µs)
    - `PWM_AUX_MAX1 = 15000` (2026-06-19: raised 3000→15000 — at 3000 the SmartFLEX motor ran too slow / spray dripped, not smooth. 15000 µs is far beyond normal servo range; SmartFLEX maps a wide PWM band to motor speed. TODO: confirm thermal/stability over a sustained run.)
    - `PWM_AUX_DIS1 = 0` (disarmed output = 0 µs)
  - **Reading actual AUX1 output:** `/mavros/rc/out` does NOT publish on the Jetson link (restricted pluginlist; PX4 not emitting SERVO_OUTPUT_RAW). Read AUX1 PWM via QGC → MAVLink Inspector → `SERVO_OUTPUT_RAW.servo9_raw`.
  - **Why not cmd 183 (DO_SET_SERVO):** PX4 denies cmd 183 with `PWM_AUX_FUNC1=301` (RC passthrough). Switching to `mavlink_servo_pwm` backend requires changing FUNC1 to Generic Actuator in QGC — deferred, not needed.
  - **Bench test command (armed, no OFFBOARD needed):** `MAV_CMD_DO_SET_ACTUATOR` cmd 187 is accepted while armed in any mode. cmd 183 requires OFFBOARD.
- robot_localization fusion: not yet built

### Active focus (4WD — Phase 3)
1. **Path engine + trajectory planning** — mission/path generation, segment splitting, corner handling
2. **CRS / coordinate handling** — coordinate reference system + geodesic conversion for path import
3. **Spray control logic** — validate flag conditioning, timing, safety gates end-to-end
4. **Full-pipeline validation** — CAD/DXF → path → mission → drive → spray, on hardware

## Hard rules

- Do not edit PX4 firmware on Jetson
- Do not stop `px4-dxp.service` without warning — carries QGC bridge
- Do not disable RTK (`ntrip_rtcm_node.py`)
- Do not push FCU params from Jetson — QGC (DYX_GCS_V) on Mac is source of truth
- ArduRover is abandoned — do not propose ArduRover solutions

## Quick reference

```bash
ros2 topic echo /mavros/state --once
ros2 topic echo /mavros/local_position/pose
journalctl -u px4-dxp.service -f
ros2 bag record /mavros/local_position/pose /mavros/setpoint_raw/local /mavros/state -o ~/bags/$(date +%Y%m%d_%H%M%S)
```

- NTRIP creds: `~/4WD_SERVER/config/ntrip.env` (gitignored)
- MAVROS pluginlist: `~/4WD_SERVER/px4_pluginlists_rover.yaml`
- FastAPI: port 5001 — `curl http://localhost:5001/api/ping`
- QGC UDP: 14550 | ROS_DOMAIN_ID: 0

## Telemetry debugging

Use `tools/capture_telemetry.py` to inspect live WebSocket telemetry — prefer this over `curl /api/telemetry/latest` when you need multiple samples or want to watch values change.

```bash
# From Mac — single snapshot
ssh flash@192.168.1.101 'cd ~/4WD_SERVER && python3 tools/capture_telemetry.py -n 1 --host localhost'

# From Mac — 5 samples (one per 100ms tick at 10 Hz)
ssh flash@192.168.1.101 'cd ~/4WD_SERVER && python3 tools/capture_telemetry.py -n 5 --host localhost'

# From Mac — continuous stream until Ctrl-C
ssh flash@192.168.1.101 'cd ~/4WD_SERVER && python3 tools/capture_telemetry.py -n 0 --host localhost'

# Filter a specific field (e.g. GPS accuracy)
ssh flash@192.168.1.101 'cd ~/4WD_SERVER && python3 tools/capture_telemetry.py -n 5 --host localhost 2>/dev/null' \
  | python3 -c "import sys,json; [print(json.loads(l)['gps_fix_name'], json.loads(l)['hrms'], json.loads(l)['vrms']) for l in sys.stdin]"
```

Output is NDJSON (one JSON object per line). Fields: all `TelemetryData` fields + `_captured_at` (UTC ISO-8601). NaN → null.
