#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUTPUT_DIR="${ROOT_DIR}/newdir"

if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  echo "GUI display not detected. Run this from a normal Ubuntu desktop terminal."
  exit 1
fi

export PYTHONNOUSERSITE=1
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES-}"
export COLCON_TRACE="${COLCON_TRACE-}"

set +u
source /opt/ros/jazzy/setup.bash
set -u

cd "${ROOT_DIR}"
mkdir -p "${OUTPUT_DIR}"

rosdep install --from-paths src --ignore-src -r -y --skip-keys ament_python

rm -rf build install log

colcon build --symlink-install \
  --allow-overriding mini_r1_v1_description mini_r1_v1_gz \
  --packages-up-to \
    grid_world \
    mini_r1_v1_description \
    mini_r1_v1_gz \
    rover_autonomy \
    rover_logging \
    rover_bringup

export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES-}"
export COLCON_TRACE="${COLCON_TRACE-}"
set +u
source install/setup.bash
set -u

ros2 launch rover_bringup full_stack.launch.py \
  headless:=false \
  output_dir:="${OUTPUT_DIR}"
