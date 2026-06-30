#!/usr/bin/env bash
# Wrapper for the auto rosbag recorder: source ROS2, then exec the daemon.
set -euo pipefail

set +u; source /opt/ros/humble/setup.bash; set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

# Where to save bags (override via systemd Environment=BAGS_DIR=...)
export BAGS_DIR="${BAGS_DIR:-$HOME/bags_jet}"
export PX4_DXP_DIR="${PX4_DXP_DIR:-$HOME/4WD_SERVER}"
export MISSION_DEBUG_CONTROL_DIR="${MISSION_DEBUG_CONTROL_DIR:-$PX4_DXP_DIR/runtime/mission-debug}"
export BAG_QOS_OVERRIDES="${BAG_QOS_OVERRIDES:-$PX4_DXP_DIR/config/rosbag_qos_overrides.yaml}"
mkdir -p "$BAGS_DIR"
mkdir -p "$MISSION_DEBUG_CONTROL_DIR"

exec python3 "$(dirname "$0")/bag_autorecord.py"
