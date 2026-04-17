#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash

# Use the script's parent directory as the workspace root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${WORKSPACE_DIR:-$SCRIPT_DIR}"

cd "$WORKSPACE_DIR"
source install/setup.bash

LOG=/tmp/round3_full_run.log
: > "$LOG"

ros2 launch rover_bringup full_stack.launch.py headless:=true > "$LOG" 2>&1 &
LAUNCH_PID=$!

sleep 25

echo '--- nodes ---'
ros2 node list || true

echo '--- topics ---'
ros2 topic list || true

echo '--- camera topic info ---'
ros2 topic info /camera/image_raw || true

echo '--- scan topic info ---'
ros2 topic info /scan || true

echo '--- mission state sample ---'
timeout 5s ros2 topic echo /mission/state --once || true

echo '--- path tracking sample ---'
timeout 8s ros2 topic echo /vision/path_tracking --once || true

echo '--- scan sample ---'
timeout 8s ros2 topic echo /scan --once || true

echo '--- camera info sample ---'
timeout 8s ros2 topic echo /camera/camera_info --once || true

kill $LAUNCH_PID || true
sleep 5

echo '--- launch log tail ---'
tail -n 120 "$LOG" || true
