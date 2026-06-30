# 4WD_SERVER — 4WD Marking Rover (Jetson Runtime)

Runtime workspace on the Jetson Orin companion computer for the DYX Autonomous 4WD marking rover.

- **FCU:** CubeOrangePlus running PX4 v1.16.2 (custom rover build, fork: [Vetri2425/PX4-Autopilot](https://github.com/Vetri2425/PX4-Autopilot))
- **Bridge:** MAVROS2 over `/dev/ttyACM0` @ 921600
- **RTK:** Holybro UM982 dual-antenna with NTRIP injection
- **ROS2:** Humble on Ubuntu (Tegra)
- **GCS Frontend:** DYX_GCS_V (QGroundControl on macOS + FastAPI mobile UI)
- **Role:** Phase 2 ROS2 OFFBOARD arc controller (replaces PX4 AUTO densified-waypoint method)

## Contents

| File | Purpose |
|---|---|
| `px4_start_service.sh` | systemd launcher for MAVROS2 + NTRIP |
| `px4_pluginlists_rover.yaml` | MAVROS plugin allowlist |
| `CLAUDE.md` | Context file for Claude Code (runtime brain scope) |

## Architecture

See [Architecture Decision](https://github.com/Vetri2425/PX4-Autopilot) — laptop side owns firmware patches, this side owns ROS2 runtime.

## Changelog

**2026-06-30 — 4WD_SERVER repo created:** Forked from PX4_DXP (3WD) into dedicated 4WD repo. Updated all service/script paths from `PX4_DXP` → `4WD_SERVER`. Deployed on new Jetson at `192.168.1.101`. QGC connected via UDP:14550.

**2026-06-11 — codebase audit update:** Phase 3 spray software is implemented in this repo: `spray_flags` ride the `/path` z-channel, RPP conditions exact CAD PRE/MARK/AFT geometry and publishes `/rpp/conditioned_path` plus identity, and `spray_controller_node.py` consumes that conditioned geometry for runtime timing, flow, speed safety, and actuator control. `/spray/active` remains telemetry/legacy fallback only when distance-aware mode is disabled. `rpp_start.sh` starts/watchdogs the node, and the server exposes `/api/spray/*` plus `marking_state` telemetry. Remaining spray work is QGC AUX configuration, physical wiring, bench latency measurement, and hardware safety validation.

**2026-05-25 — path_engine v1.0 (Phases 1-4):** Added complete path planning subsystem for DXF/CSV/QGC mission files. Phase 1: core data models (SegmentType, PathSegment, PlannedPath, DXFEntity), parsers (ezdxf-based DXF with LINE/POINT/SPLINE, enhanced 6-col CSV with backward-compatible 2-col, QGC .waypoints via Karney geodesic), straight-line densification at 5cm MARK/15cm TRANSIT spacing. Phase 2: curvature-adaptive arc/circle discretization using chord-error (sagitta) method, LWPOLYLINE bulge-to-arc conversion (positive=CCW, negative=CW per DXF standard), ELLIPSE via ezdxf make_path+flattening. Phase 3: nearest-neighbor TSP segment ordering with endpoint reversal, TRANSIT segment insertion, exact PRE/MARK/AFT geometry, per-entity spray overrides, and `spray_flags` parallel to `merged_waypoints`. Phase 4/current runtime: planned paths publish flags via `/path` `pose.position.z`; RPP owns motion tracking and conditioned geometry; the spray controller owns runtime timing, flow, safety, and actuator control; FastAPI exposes path planning, staging, load-to-controller, and spray endpoints. Dependencies: ezdxf, geographiclib (existing). scipy NOT required.

## Service

```bash
systemctl status px4-dxp.service
journalctl -u px4-dxp.service -f
```

## License

TBD
