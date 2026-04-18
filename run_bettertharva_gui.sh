#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUTPUT_DIR="${ROOT_DIR}/output"

source /opt/ros/jazzy/setup.bash

cd "${ROOT_DIR}"
export PYTHONNOUSERSITE=1

mkdir -p "${OUTPUT_DIR}"

rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install --packages-up-to \
  grid_world \
  mini_r1_v1_description \
  mini_r1_v1_gz \
  rover_autonomy \
  rover_logging \
  rover_bringup

source install/setup.bash

ros2 launch rover_bringup full_stack.launch.py \
  headless:=false \
  output_dir:="${OUTPUT_DIR}"
