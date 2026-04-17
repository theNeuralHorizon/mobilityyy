from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from enum import Enum

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from sensor_msgs.msg import Imu, LaserScan
    from std_msgs.msg import Bool, String
except ImportError:  # pragma: no cover
    rclpy = None
    Node = object
    Twist = None
    Imu = None
    LaserScan = None
    Bool = None
    String = None


class MissionPhase(Enum):
    STATE_START = "State_Start"
    STATE_TAG2_FOUND = "State_Tag2_Found"
    STATE_TAG1_DEAD_END = "State_Tag1_DeadEnd"
    STATE_RETURN_TAG2 = "State_Return_Tag2"
    STATE_TAG3_GREEN_PATH = "State_Tag3_GreenPath"
    STATE_TAG4_UTURN = "State_Tag4_UTurn"
    STATE_TAG5_ORANGE_PATH = "State_Tag5_OrangePath"
    STATE_FINAL_GOAL = "State_FinalGoal"
    STATE_STOP = "State_Stop"


EXPLORE_PHASES = frozenset({
    MissionPhase.STATE_START,
    MissionPhase.STATE_TAG2_FOUND,
    MissionPhase.STATE_RETURN_TAG2,
})

FOLLOW_PHASES = frozenset({
    MissionPhase.STATE_TAG3_GREEN_PATH,
    MissionPhase.STATE_TAG5_ORANGE_PATH,
    MissionPhase.STATE_FINAL_GOAL,
})


def advance_phase_on_tag(
    phase: MissionPhase,
    mission_label: int,
    skip_tag1_return: bool,
) -> MissionPhase:
    if phase is MissionPhase.STATE_START and mission_label == 2:
        return MissionPhase.STATE_TAG2_FOUND
    if phase is MissionPhase.STATE_TAG2_FOUND and mission_label == 1:
        return MissionPhase.STATE_TAG1_DEAD_END
    if phase is MissionPhase.STATE_TAG1_DEAD_END and skip_tag1_return:
        return MissionPhase.STATE_TAG3_GREEN_PATH if mission_label == 3 else MissionPhase.STATE_TAG1_DEAD_END
    if phase is MissionPhase.STATE_TAG1_DEAD_END and mission_label == 2:
        return MissionPhase.STATE_RETURN_TAG2
    if phase in (MissionPhase.STATE_RETURN_TAG2, MissionPhase.STATE_TAG1_DEAD_END) and mission_label == 3:
        return MissionPhase.STATE_TAG3_GREEN_PATH
    if phase is MissionPhase.STATE_TAG3_GREEN_PATH and mission_label == 4:
        return MissionPhase.STATE_TAG4_UTURN
    if phase is MissionPhase.STATE_TAG4_UTURN and mission_label == 5:
        return MissionPhase.STATE_TAG5_ORANGE_PATH
    return phase


@dataclass
class ControlState:
    phase: MissionPhase = MissionPhase.STATE_START
    target_color: str = "none"
    skip_tag1_return: bool = False
    latest_clearance_ok: bool = True
    last_yaw_error: float = 0.0
    stop_detected: bool = False
    # U-turn tracking
    uturn_start_yaw: float | None = None
    uturn_accumulated: float = 0.0
    uturn_last_yaw: float | None = None
    uturn_start_time: float = 0.0
    # STOP multi-frame confirmation
    stop_confirm_count: int = 0


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def compute_follow_command(target_detected: bool, yaw_error: float, linear_speed: float) -> tuple[float, float]:
    if not target_detected:
        return 0.0, 0.35
    return linear_speed, _clamp(-0.9 * yaw_error, -1.0, 1.0)


def compute_wall_follow_command(
    front_min: float,
    left_min: float,
    right_min: float,
    linear_speed: float,
    wall_follow_distance: float = 0.35,
    front_threshold: float = 0.4,
) -> tuple[float, float]:
    """Right-hand wall-following: stay near the right wall.

    - Corner (front + right blocked) -> spin left hard
    - Front blocked -> turn left
    - Right wall present and at good distance -> drive straight
    - Right side too close -> veer left slightly
    - Right side too far / open -> turn right to seek wall
    """
    if front_min < front_threshold and right_min < wall_follow_distance:
        # Corner: both front and right blocked — spin left in place
        return 0.0, 0.8
    if front_min < front_threshold:
        # Wall ahead — turn left
        return 0.05, 0.6
    if right_min < wall_follow_distance * 0.6:
        # Too close to right wall — veer left
        return linear_speed * 0.8, 0.3
    if right_min < wall_follow_distance * 1.4:
        # Right wall at good distance — drive straight
        return linear_speed, 0.0
    # Right side open — turn right to seek wall
    return linear_speed * 0.7, -0.4


def _sector_min(ranges: list[float], start: int, end: int) -> float:
    valid = [v for v in ranges[start:end] if math.isfinite(v) and v > 0.0]
    return min(valid) if valid else float("inf")


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Extract yaw from quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class StateMachineController(Node):
    STOP_CONFIRM_THRESHOLD = 5
    UTURN_TIMEOUT_SEC = 8.0

    def __init__(self) -> None:
        if rclpy is None:  # pragma: no cover
            raise RuntimeError("rclpy is required to run the state machine node")
        super().__init__("state_machine_controller")
        self.declare_parameter("skip_tag1_return", False)
        self.declare_parameter("explore_linear_speed", 0.18)
        self.declare_parameter("follow_linear_speed", 0.14)
        self.declare_parameter("wall_follow_distance", 0.35)
        self.state = ControlState(skip_tag1_return=bool(self.get_parameter("skip_tag1_return").value))
        self.explore_linear_speed = float(self.get_parameter("explore_linear_speed").value)
        self.follow_linear_speed = float(self.get_parameter("follow_linear_speed").value)
        self.wall_follow_distance = float(self.get_parameter("wall_follow_distance").value)
        self.last_path_tracking = {"detected": False, "yaw_error": 0.0, "stop_detected": False}

        # Sensor state for wall-following
        self.front_min = float("inf")
        self.left_min = float("inf")
        self.right_min = float("inf")
        self.current_yaw = 0.0

        self.cmd_pub = self.create_publisher(Twist, "/nav/cmd_vel_raw", 10)
        self.color_pub = self.create_publisher(String, "/vision/target_color", 10)
        self.phase_pub = self.create_publisher(String, "/mission/state", 10)
        self.log_pub = self.create_publisher(String, "/mission/log_event", 10)

        self.create_subscription(String, "/vision/tag_event", self._on_tag_event, 10)
        self.create_subscription(String, "/vision/path_tracking", self._on_path_tracking, 10)
        self.create_subscription(Bool, "/safety/clearance", self._on_clearance, 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.create_subscription(Imu, "/imu", self._on_imu, 10)
        self.timer = self.create_timer(0.1, self._tick)
        self._publish_phase()

    def _on_scan(self, msg: LaserScan) -> None:
        ranges = list(msg.ranges)
        count = len(ranges)
        if count == 0:
            return
        quarter = max(count // 4, 1)
        self.front_min = _sector_min(ranges, 0, quarter)
        self.left_min = _sector_min(ranges, quarter, quarter * 2)
        self.right_min = _sector_min(ranges, quarter * 3, count)

    def _on_imu(self, msg: Imu) -> None:
        q = msg.orientation
        self.current_yaw = _yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def _on_tag_event(self, msg: String) -> None:
        event = json.loads(msg.data)
        label = int(event["mission_label"])
        old_phase = self.state.phase
        self.state.phase = advance_phase_on_tag(self.state.phase, label, self.state.skip_tag1_return)
        if self.state.phase is MissionPhase.STATE_TAG3_GREEN_PATH:
            self.state.target_color = "green"
        elif self.state.phase in (MissionPhase.STATE_TAG5_ORANGE_PATH, MissionPhase.STATE_FINAL_GOAL):
            self.state.target_color = "orange"
        # Initialize U-turn tracking when entering U-turn state
        if self.state.phase is MissionPhase.STATE_TAG4_UTURN and old_phase is not MissionPhase.STATE_TAG4_UTURN:
            self.state.uturn_start_yaw = self.current_yaw
            self.state.uturn_accumulated = 0.0
            self.state.uturn_last_yaw = self.current_yaw
            self.state.uturn_start_time = time.time()
        self.log_pub.publish(String(data=json.dumps({
            "event": "tag_transition",
            "mission_label": label,
            "phase": self.state.phase.value,
            "decision_taken": event.get("decision_taken", ""),
        })))
        self._publish_phase()

    def _on_path_tracking(self, msg: String) -> None:
        self.last_path_tracking = json.loads(msg.data)
        self.state.last_yaw_error = float(self.last_path_tracking.get("yaw_error", 0.0))
        raw_stop = bool(self.last_path_tracking.get("stop_detected", False))

        # Multi-frame STOP confirmation
        if raw_stop:
            self.state.stop_confirm_count += 1
        else:
            self.state.stop_confirm_count = 0

        confirmed_stop = self.state.stop_confirm_count >= self.STOP_CONFIRM_THRESHOLD
        self.state.stop_detected = confirmed_stop

        if self.state.phase is MissionPhase.STATE_TAG5_ORANGE_PATH:
            self.state.phase = MissionPhase.STATE_FINAL_GOAL
            self._publish_phase()
        if self.state.phase is MissionPhase.STATE_FINAL_GOAL and confirmed_stop:
            self.state.phase = MissionPhase.STATE_STOP
            self.log_pub.publish(String(data=json.dumps({"event": "stop_detected"})))
            self._publish_phase()

    def _on_clearance(self, msg: Bool) -> None:
        self.state.latest_clearance_ok = bool(msg.data)

    def _publish_phase(self) -> None:
        self.phase_pub.publish(String(data=self.state.phase.value))
        self.color_pub.publish(String(data=self.state.target_color))

    def _tick(self) -> None:
        cmd = Twist()
        # NOTE: Do NOT gate on clearance here. The safety controller
        # already overrides unsafe commands on /cmd_vel. If we send
        # zero when clearance is False, the safety controller has
        # nothing useful to work with and the robot freezes.

        if self.state.phase in EXPLORE_PHASES:
            # Wall-following exploration
            cmd.linear.x, cmd.angular.z = compute_wall_follow_command(
                self.front_min,
                self.left_min,
                self.right_min,
                self.explore_linear_speed,
                self.wall_follow_distance,
            )
        elif self.state.phase is MissionPhase.STATE_TAG1_DEAD_END:
            # Dead end — sharper right bias to reverse
            cmd.linear.x = self.explore_linear_speed
            cmd.angular.z = -0.5
        elif self.state.phase is MissionPhase.STATE_TAG4_UTURN:
            # IMU-tracked U-turn with timeout
            cmd.linear.x = 0.0
            cmd.angular.z = 0.7
            if self.state.uturn_last_yaw is not None:
                delta = _normalize_angle(self.current_yaw - self.state.uturn_last_yaw)
                self.state.uturn_accumulated += abs(delta)
                self.state.uturn_last_yaw = self.current_yaw
            elapsed = time.time() - self.state.uturn_start_time
            if self.state.uturn_accumulated >= math.pi or elapsed > self.UTURN_TIMEOUT_SEC:
                # U-turn complete — resume exploration to find Tag 5
                self.state.phase = MissionPhase.STATE_TAG5_ORANGE_PATH
                self.state.target_color = "orange"
                self._publish_phase()
                cmd.linear.x = self.explore_linear_speed
                cmd.angular.z = 0.0
        elif self.state.phase in FOLLOW_PHASES:
            detected = bool(self.last_path_tracking.get("detected", False))
            linear_x, angular_z = compute_follow_command(
                detected,
                self.state.last_yaw_error,
                self.follow_linear_speed,
            )
            cmd.linear.x = linear_x
            cmd.angular.z = angular_z

        self.cmd_pub.publish(cmd)


def main(args=None) -> None:  # pragma: no cover
    if rclpy is None:
        raise RuntimeError("rclpy is required to run this node")
    rclpy.init(args=args)
    node = StateMachineController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
