#!/bin/bash
set -e

echo "=== Setting up ROS 2 Jazzy environment ==="
source /opt/ros/jazzy/setup.bash

echo "=== Installing Python dependencies ==="
pip install --break-system-packages pupil-apriltags numpy opencv-python-headless 2>/dev/null || \
pip install --break-system-packages pupil-apriltags numpy 2>/dev/null || true

echo "=== Initializing rosdep ==="
sudo rosdep init 2>/dev/null || true
rosdep update 2>/dev/null || true

echo "=== Installing ROS dependencies ==="
cd ~/gaws_ws
rosdep install --from-paths src --ignore-src -r -y --skip-keys "ament_python" 2>&1 | tail -5

echo "=== Building workspace ==="
colcon build --symlink-install 2>&1 | tail -20
source install/setup.bash

echo "=== Build complete ==="
echo "To run: ros2 launch artpark_bringup full_run.launch.py spawn_yaw:=0.0"
