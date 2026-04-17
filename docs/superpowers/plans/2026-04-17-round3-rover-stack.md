# Round 3 Rover Stack Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a lean ROS 2 autonomy stack for the Round 3 rover with hybrid AprilTag plus OpenCV perception, a deterministic mission state machine, lidar-only safety override, and score-critical CSV/PNG logging.

**Architecture:** The stack uses `apriltag_ros` for tag detection and four custom nodes split across three lightweight packages: `rover_autonomy`, `rover_logging`, and `rover_bringup`. A separate `rover_robot` package holds robot description and Gazebo launch support. The hot path stays compact by using one vision node and one state-machine node, with lidar isolated in a safety override node.

**Tech Stack:** ROS 2 Python nodes (`rclpy`), OpenCV, `cv_bridge`, `apriltag_ros`, Gazebo, YAML launch/config.

---

### Task 1: Scaffold ROS 2 package layout

**Files:**
- Create: `src/rover_bringup/package.xml`
- Create: `src/rover_bringup/CMakeLists.txt`
- Create: `src/rover_autonomy/package.xml`
- Create: `src/rover_autonomy/setup.py`
- Create: `src/rover_autonomy/setup.cfg`
- Create: `src/rover_logging/package.xml`
- Create: `src/rover_logging/setup.py`
- Create: `src/rover_logging/setup.cfg`
- Create: `src/rover_robot/package.xml`
- Create: `src/rover_robot/CMakeLists.txt`

- [ ] Write package manifests and Python packaging so `colcon` can discover all packages.
- [ ] Add console entry points for `vision_processor`, `state_machine_controller`, `safety_controller`, and `run_logger`.
- [ ] Run a package discovery check from WSL using `colcon list`.

### Task 2: Add shared configs and launch surface

**Files:**
- Create: `src/rover_bringup/config/apriltag.yaml`
- Create: `src/rover_bringup/config/hsv_thresholds.yaml`
- Create: `src/rover_bringup/config/navigation.yaml`
- Create: `src/rover_bringup/config/tag_map.yaml`
- Create: `src/rover_bringup/launch/sim_world.launch.py`
- Create: `src/rover_bringup/launch/robot_spawn.launch.py`
- Create: `src/rover_bringup/launch/autonomy.launch.py`
- Create: `src/rover_bringup/launch/full_stack.launch.py`

- [ ] Define the YAML parameters for tag mapping, HSV thresholds, safety thresholds, and motion gains.
- [ ] Create the launch files with `headless`, `debug_gui`, `camera_width`, `camera_height`, `camera_fps`, and `skip_tag1_return` arguments.
- [ ] Run a Python syntax check on all launch files.

### Task 3: Implement the vision pipeline

**Files:**
- Create: `src/rover_autonomy/rover_autonomy/vision_processor.py`
- Create: `src/rover_autonomy/rover_autonomy/hsv_utils.py`
- Create: `src/rover_autonomy/rover_autonomy/arrow_geometry.py`
- Create: `src/rover_autonomy/rover_autonomy/tag_mapper.py`
- Test: `tests/rover_autonomy/test_arrow_geometry.py`
- Test: `tests/rover_autonomy/test_tag_mapper.py`

- [ ] Write failing unit tests for tag ID mapping and arrow-tip direction extraction.
- [ ] Run the tests to confirm they fail for missing code.
- [ ] Implement the helpers and the vision node with ROI cropping, HSV masks, arrow direction estimation, tile event debouncing, and tag-event normalization.
- [ ] Re-run the focused tests and then a Python syntax check for the package.

### Task 4: Implement the mission state machine

**Files:**
- Create: `src/rover_autonomy/rover_autonomy/state_machine_controller.py`
- Test: `tests/rover_autonomy/test_state_machine_logic.py`

- [ ] Write failing tests for the mission state sequence, legal return-to-Tag-2 path, and `skip_tag1_return` override.
- [ ] Run the tests to confirm they fail for the expected reasons.
- [ ] Implement the deterministic state machine and raw velocity generation.
- [ ] Re-run the state-machine tests and verify they pass.

### Task 5: Implement lidar-only safety override

**Files:**
- Create: `src/rover_autonomy/rover_autonomy/safety_controller.py`
- Test: `tests/rover_autonomy/test_safety_controller_logic.py`

- [ ] Write failing tests for front-wall stop behavior and turn-away override direction.
- [ ] Run the tests to confirm they fail.
- [ ] Implement the safety node so it is the only publisher to `/cmd_vel`.
- [ ] Re-run the safety tests and verify they pass.

### Task 6: Implement score logging

**Files:**
- Create: `src/rover_logging/rover_logging/run_logger.py`
- Test: `tests/rover_logging/test_log_row_builder.py`

- [ ] Write failing tests for ordered CSV row formatting and PNG filename generation.
- [ ] Run the tests to confirm they fail.
- [ ] Implement the logger node and helper logic for `run_log.csv` and PNG capture.
- [ ] Re-run the logging tests and verify they pass.

### Task 7: Add robot package and spawn support

**Files:**
- Create: `src/rover_robot/urdf/rover.urdf.xacro`
- Create: `src/rover_robot/launch/robot.launch.py`

- [ ] Add a lightweight differential-drive robot model with RGB camera and 2D lidar placeholders compatible with Gazebo bringup.
- [ ] Wire spawn arguments to the master launch.
- [ ] Run launch-file syntax checks and inspect the generated robot description for obvious errors.

### Task 8: Verify workspace and WSL execution

**Files:**
- Modify: `docs/superpowers/specs/2026-04-17-round3-rover-design.md`

- [ ] Run local Python syntax validation for all created Python files.
- [ ] If WSL is available, run `wsl` commands to verify Python, ROS 2 package discovery, and `colcon build`.
- [ ] If ROS 2 and Gazebo are available in WSL, launch the stack in headless mode and collect evidence from topic list and node list.
- [ ] Document any environment blockers directly in the spec if full runtime verification cannot complete.
