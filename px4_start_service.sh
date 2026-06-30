#!/bin/bash

# PX4 MAVROS bridge — CubeOrangePlus via /dev/ttyACM0
# QGC connects via directed UDP (no telemetry radio needed):
#   In QGC: Comm Links → Add → UDP → Port 14550 → connect
#   MAVROS sends directed packets to LAPTOP_IP, not broadcast.

set -euo pipefail

FCU_DEVICE="/dev/serial/by-id/usb-CubePilot_CubeOrange+_0-if00"
FCU_BAUD="921600"
GCS_UDP_PORT="14550"
JETSON_IP="192.168.1.101"
# GCS link. Default = server mode: MAVROS binds :14550 and the GCS dials in.
# mavros2's router only forwards FCU telemetry to a GCS AFTER that GCS has sent
# a packet first, so proactive push (unicast remote OR broadcast) does NOT work
# — the GCS must initiate. In QGC: add a UDP link with Server Address
# <jetson-ip>:14550 (NOT the listen-only AutoConnect default). Override via env:
#   GCS_URL="udp://:14550@192.168.1.7:14550" (if a specific GCS sends first)
GCS_URL="${GCS_URL:-udp://:${GCS_UDP_PORT}@}"
ROS_SETUP="/opt/ros/humble/setup.bash"

# Timing constants
# MAVROS_READY_TIMEOUT is the main-body wait for the flag file;
# it must be > MAVROS_READY_WAIT (the watchdog's per-start attempt limit)
# so the main body always gives the watchdog at least one full attempt.
MAVROS_READY_TIMEOUT=35
MAVROS_READY_WAIT=30
MAVROS_RESTART_DELAY=3
MAVROS_FAIL_DELAY=5
SHUTDOWN_GRACE_TICKS=25

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
MAVROS_HEALTH_SCRIPT="${SCRIPT_DIR}/tools/ros2_mavros_health.py"
MAVROS_READY_FLAG="/tmp/px4_mavros_ready"

declare -a CHILD_PIDS=()
MAVROS_WATCHDOG_PID=""

log() { echo "[px4_service] $(date '+%H:%M:%S') $*"; }

terminate_pid() {
    local pid="${1:-}"
    local label="${2:-process}"
    if [[ -z "$pid" ]] || ! kill -0 "$pid" 2>/dev/null; then
        return 0
    fi

    kill -TERM "$pid" 2>/dev/null || true
    for _ in $(seq 1 "$SHUTDOWN_GRACE_TICKS"); do
        if ! kill -0 "$pid" 2>/dev/null; then
            wait "$pid" 2>/dev/null || true
            return 0
        fi
        sleep 0.2
    done

    log "WARNING: $label did not exit after TERM — sending KILL"
    kill -KILL "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
}

cleanup() {
    log "Cleaning up child processes..."
    rm -f "$MAVROS_READY_FLAG"
    for pid in "${CHILD_PIDS[@]:-}"; do
        terminate_pid "$pid" "watchdog PID $pid"
    done
}

handle_exit() {
    local code=$1
    trap - EXIT INT TERM
    cleanup
    exit "$code"
}

check_ros_node() {
    # Use direct rclpy discovery instead of `ros2 node list`, which goes
    # through the ROS2 CLI daemon and can be slow/stale during MAVROS restarts.
    timeout 6 python3 "$MAVROS_HEALTH_SCRIPT" --timeout 5 node "$1" 2>/dev/null
}

free_port() {
    local port=$1
    if lsof -i ":$port" >/dev/null 2>&1; then
        log "Port $port busy — freeing gracefully..."
        lsof -ti ":$port" | xargs -r kill -TERM 2>/dev/null || true
        sleep 2
        if lsof -i ":$port" >/dev/null 2>&1; then
            log "Port $port still busy — force killing..."
            lsof -ti ":$port" | xargs -r kill -9 2>/dev/null || true
        fi
    fi
}

mavros_watchdog() {
    local mavros_pid=""

    _wd_cleanup() {
        terminate_pid "$mavros_pid" "MAVROS"
        exit 0
    }
    trap '_wd_cleanup' TERM INT

    while true; do
        log "Watchdog: starting MAVROS (PX4)..."
        free_port 14550
        sleep 1

        ros2 launch mavros node.launch \
            fcu_url:=${FCU_DEVICE}:${FCU_BAUD} \
            gcs_url:=${GCS_URL} \
            pluginlists_yaml:=${SCRIPT_DIR}/px4_pluginlists_rover.yaml \
            config_yaml:=/opt/ros/humble/share/mavros/launch/px4_config.yaml \
            fcu_protocol:=v2.0 \
            tgt_system:=1 \
            tgt_component:=1 \
            log_output:=screen \
            respawn_mavros:=false &
        mavros_pid=$!

        local ready=0
        for i in $(seq 1 "$MAVROS_READY_WAIT"); do
            if check_ros_node "/mavros"; then
                ready=1
                break
            fi
            if ! kill -0 "$mavros_pid" 2>/dev/null; then
                log "Watchdog: MAVROS exited before node appeared"
                break
            fi
            log "Waiting for /mavros node... ($i/$MAVROS_READY_WAIT)"
            sleep 1
        done

        if [[ "$ready" -eq 1 ]]; then
            log "Watchdog: MAVROS ready (PID $mavros_pid)"
            touch "$MAVROS_READY_FLAG"
            wait "$mavros_pid" 2>/dev/null || true
            log "Watchdog: MAVROS exited — restarting in ${MAVROS_RESTART_DELAY}s..."
            mavros_pid=""
            sleep "$MAVROS_RESTART_DELAY"
        else
            log "Watchdog: MAVROS failed to start — retrying in ${MAVROS_FAIL_DELAY}s..."
            terminate_pid "$mavros_pid" "MAVROS failed start"
            mavros_pid=""
            sleep "$MAVROS_FAIL_DELAY"
        fi
    done
}

trap 'handle_exit $?' EXIT
trap 'handle_exit 130' INT
trap 'handle_exit 143' TERM

# Verify FCU device
if [[ ! -c "$FCU_DEVICE" ]]; then
    log "ERROR: $FCU_DEVICE not found — is CubeOrangePlus plugged in via USB?"
    exit 1
fi
log "FCU device found: $FCU_DEVICE (CubeOrangePlus PX4)"

# Source ROS 2
if [[ ! -f "$ROS_SETUP" ]]; then
    log "ERROR: ROS 2 setup not found at $ROS_SETUP"
    exit 1
fi
set +u; source "$ROS_SETUP"; set -u

# Kill stale processes — all best-effort, explicitly bracketed.
# `set -e` would abort the script if any of these fail, which is wrong here
# (they're cleanup ops; non-zero exit is expected when nothing is running).
set +e
pkill -f "mavros.*node.launch" 2>/dev/null
set -e
sleep 1

log "====================================================="
log " PX4 MAVROS UDP Bridge Starting"
log " FCU : $FCU_DEVICE @ ${FCU_BAUD} baud"
log " QGC : UDP server mode — listening on :$GCS_UDP_PORT (any device connects)"
log " QGC setup: Comm Links → Add → UDP → Port $GCS_UDP_PORT"
log " Or QGC auto-discovers on same LAN (no config needed)"
log "====================================================="

rm -f "$MAVROS_READY_FLAG"

mavros_watchdog &
MAVROS_WATCHDOG_PID=$!
CHILD_PIDS+=("$MAVROS_WATCHDOG_PID")

log "Waiting for MAVROS to initialise..."
mavros_ready=0
for i in $(seq 1 "$MAVROS_READY_TIMEOUT"); do
    if [[ -f "$MAVROS_READY_FLAG" ]]; then
        mavros_ready=1
        rm -f "$MAVROS_READY_FLAG"
        break
    fi
    if ! kill -0 "$MAVROS_WATCHDOG_PID" 2>/dev/null; then
        log "ERROR: MAVROS watchdog died unexpectedly"
        exit 1
    fi
    log "Waiting... ($i/$MAVROS_READY_TIMEOUT)"
    sleep 1
done

if [[ "$mavros_ready" -eq 0 ]]; then
    log "ERROR: MAVROS did not come up in time"
    exit 1
fi

log "MAVROS is READY"

# Validate FCU connection with direct rclpy subscription instead of
# `ros2 topic echo`, avoiding ROS2 CLI daemon latency/stale process state.
if timeout 11 python3 "$MAVROS_HEALTH_SCRIPT" --timeout 10 state --require-connected 2>/dev/null; then
    log "FCU connected via MAVROS"
else
    log "WARNING: MAVROS node exists but FCU may not be connected — check serial link"
fi

log "Active ROS nodes:"
timeout 10 python3 "$MAVROS_HEALTH_SCRIPT" --timeout 5 node "/mavros" >/dev/null 2>&1 \
    && log "  /mavros" || log "  /mavros not discovered"
log "=== Bridge running. QGC → UDP → ${JETSON_IP}:${GCS_UDP_PORT} ==="

wait "$MAVROS_WATCHDOG_PID"
