# Round 3 Rover Design

**Goal**

Build a lean ROS 2 autonomy stack for a differential-drive rover that completes the Round 3 mission using RGB vision for navigation, AprilTag detection for mission events, and 2D LiDAR only for collision avoidance.

**Constraints**

- ROS 2 Humble/Jazzy compatible
- Runs in WSL 2 Ubuntu with Gazebo
- Total memory budget effectively limited by 8 GB shared with WSL and Gazebo
- No pre-built map, SLAM, Nav2, or lidar-led navigation
- Logging order and mission sequence are score critical
- Tag 1 to Tag 2 retrace is allowed; other accidental retrace is penalized
- A runtime skip switch must allow bypassing the Tag 1 return sequence for speed

**Architecture**

The runtime stack uses one third-party detector and four custom nodes:

- `apriltag_ros`: detects tags from the primary RGB camera
- `vision_processor`: fuses camera image and tag detections, tracks green/orange directional markers, estimates arrow heading, detects stop, and emits tile/tag events
- `state_machine_controller`: owns mission sequencing, target color mode, and raw motion generation
- `safety_controller`: consumes LiDAR and raw velocity, overrides unsafe commands, and is the only publisher to `/cmd_vel`
- `run_logger`: writes `run_log.csv` and PNG snapshots for scoreable events

The design intentionally avoids heavyweight planners and extra ROS graph complexity. The vision node performs a single BGR-to-HSV conversion per frame, processes only relevant ROIs, and publishes compact semantic events.

**Node Contracts**

`vision_processor`
- Inputs: `/camera/image_raw`, `/camera/camera_info`, `/tag_detections`, `/vision/target_color`
- Outputs: `/vision/tag_event`, `/vision/path_tracking`, `/vision/tile_event`, optional `/vision/debug_image`
- Responsibilities:
  - normalize tag detections into mission events
  - track target color markers with HSV segmentation
  - isolate arrowhead geometry and compute yaw error
  - count total ArtPark tiles and directional tiles with temporal gating
  - detect STOP indicator in orange phase

`state_machine_controller`
- Inputs: `/vision/tag_event`, `/vision/path_tracking`, `/vision/tile_event`, `/safety/clearance`
- Outputs: `/nav/cmd_vel_raw`, `/vision/target_color`, `/mission/state`, `/mission/log_event`
- Responsibilities:
  - execute mission sequence exactly
  - support `skip_tag1_return`
  - gate allowed retrace only for Tag 1 return
  - command U-turn maneuvers at required tags
  - bias motion toward centered tag approach or color-arrow following

`safety_controller`
- Inputs: `/scan`, `/nav/cmd_vel_raw`
- Outputs: `/cmd_vel`, `/safety/clearance`, `/safety/override_active`
- Responsibilities:
  - enforce lidar-only collision prevention
  - stop forward motion if front distance < 0.2 m
  - add a repulsive angular escape direction away from the closest wall

`run_logger`
- Inputs: `/vision/tag_event`, `/vision/tile_event`, `/mission/log_event`, `/camera/image_raw`, `/mission/state`
- Outputs: filesystem only
- Responsibilities:
  - append ordered rows to `run_log.csv`
  - save PNG snapshots for all tag events
  - maintain tile counters and directional tile counters
  - preserve event ordering to avoid scoring penalties

**Mission State Machine**

Primary states:

1. `State_Start`
2. `State_Tag2_Found`
3. `State_Tag1_DeadEnd`
4. `State_Return_Tag2`
5. `State_Tag3_GreenPath`
6. `State_Tag4_UTurn`
7. `State_Tag5_OrangePath`
8. `State_FinalGoal`
9. `State_Stop`

Transitions are triggered by normalized tag commits and path-tracking events rather than raw detections. The state machine always knows the currently allowed target color and whether retrace is legal.

Special logic:

- `skip_tag1_return=false`: execute full legal U-turn and return to Tag 2
- `skip_tag1_return=true`: skip `State_Return_Tag2` and jump to the next exploration state after logging the tradeoff
- `allow_retrace_window=true` only during the legal Tag 1 to Tag 2 return

**Perception Design**

AprilTags:
- Use `apriltag_ros` for detection
- Map detector IDs to mission labels via YAML
- Debounce repeated detections using time and apparent size/range thresholds

Color and arrow tracking:
- Single HSV conversion per frame
- Lower image ROI for floor/path processing
- Separate green and orange thresholds from YAML
- Morphological open/close plus connected-component filtering
- Arrowhead isolation using contour approximation, convex hull defects, and tip-point scoring
- Heading from tip direction relative to image centerline

Tile counting:
- Count only on new commits after temporal separation and image-space displacement
- Maintain:
  - `total_tiles_seen`
  - `green_direction_tiles_seen`
  - `orange_direction_tiles_seen`

STOP detection:
- In orange-follow phase, look for stop indicator signature in the ROI
- Stop only after a stable multi-frame confirmation to reduce false positives

**Topic Layout**

Core topics:

- `/camera/image_raw`
- `/camera/camera_info`
- `/scan`
- `/tag_detections`
- `/vision/tag_event`
- `/vision/path_tracking`
- `/vision/tile_event`
- `/vision/target_color`
- `/nav/cmd_vel_raw`
- `/safety/clearance`
- `/safety/override_active`
- `/cmd_vel`
- `/mission/state`
- `/mission/log_event`

Optional debug topics:

- `/vision/debug_image`

**Package Layout**

Under `gaws_ws/src/`:

- `grid_world/` untouched world assets
- `rover_bringup/` launch files and configs
- `rover_robot/` robot model and Gazebo spawn support
- `rover_autonomy/` vision, state machine, safety, shared utilities
- `rover_logging/` CSV and PNG logger
- `rover_msgs/` only if standard messages prove too awkward

**WSL and Gazebo Strategy**

Default mode is headless or near-headless:

- Gazebo server preferred for frequent tests
- reduced camera resolution and FPS during development
- no OpenCV GUI unless `debug_gui:=true`
- GUI code must also check for `DISPLAY`

Recommended launch arguments:

- `headless:=true`
- `debug_gui:=false`
- `use_rviz:=false`
- `camera_width:=640`
- `camera_height:=360`
- `camera_fps:=10`
- `skip_tag1_return:=false`
- `use_sim_time:=true`

**Verification Strategy**

Verification occurs at three levels:

1. static validation:
   - package manifests
   - Python syntax
   - launch file syntax
2. workspace validation:
   - `colcon build`
   - package discovery
3. runtime validation in WSL:
   - launch Gazebo headless
   - launch full stack
   - confirm topic graph
   - confirm tag and path events are produced
   - confirm safety override blocks forward motion under close wall conditions

**Current Verification Status**

As of 2026-04-17 in the present WSL environment:

- helper assertions passed under `python3`
- Python syntax checks passed for autonomy, logging, and launch files
- full ROS/Gazebo validation is currently blocked because the WSL instance does not yet expose `colcon`, `ros2`, `gz`, `pytest`, `cv2`, or the ROS runtime dependencies required for `apriltag_ros` and `cv_bridge`

**Why This Design**

This design is the best fit for the user-selected priorities:

- lean competition-first runtime
- hybrid `apriltag_ros + OpenCV`
- low ROS graph overhead under WSL
- easy-to-toggle strategic skip for Tag 1 return
- score-focused logging separated from control
