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
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool, String
except ImportError:  # pragma: no cover
    rclpy = None
    Node = object
    Twist = None
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
    maneuver_until: float = 0.0
    maneuver_linear_x: float = 0.0
    maneuver_angular_z: float = 0.0
    last_tag_event_monotonic: float = 0.0
    first_hint_consumed: bool = False
    follow_side: str = "right"
    scan_until: float = 0.0
    scan_angular_z: float = 0.0
    last_scan_monotonic: float = 0.0
    maneuver_queue: list[tuple[float, float, float]] = field(default_factory=list)
    color_scan_direction: float = 1.0


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def compute_follow_command(target_detected: bool, yaw_error: float, linear_speed: float) -> tuple[float, float]:
    if not target_detected:
        remembered_bias = -0.5 * yaw_error if abs(yaw_error) > 1e-3 else 0.18
        return max(0.06, linear_speed * 0.55), _clamp(remembered_bias, -0.45, 0.45)
    return linear_speed, _clamp(-0.9 * yaw_error, -1.0, 1.0)


def _safe_min(values: list[float], default: float = 10.0) -> float:
    finite = [float(v) for v in values if math.isfinite(v) and v > 0.01]
    return min(finite) if finite else default


def regions_from_ranges(ranges: list[float]) -> dict[str, float]:
    if not ranges:
        return {"right": 10.0, "fright": 10.0, "front": 10.0, "fleft": 10.0, "left": 10.0}

    def sector(lo: int, hi: int) -> float:
        lo = max(0, min(len(ranges) - 1, lo))
        hi = max(0, min(len(ranges) - 1, hi))
        if lo > hi:
            lo, hi = hi, lo
        return _safe_min(ranges[lo : hi + 1])

    return {
        "right": sector(70, 110),
        "fright": sector(110, 150),
        "front": sector(150, 210),
        "fleft": sector(210, 250),
        "left": sector(250, 290),
    }


def wall_follow_command(regions: dict[str, float], follow_side: str = "right") -> tuple[float, float]:
    front = float(regions.get("front", 10.0))
    right = float(regions.get("right", 10.0))
    fright = float(regions.get("fright", 10.0))
    left = float(regions.get("left", 10.0))
    fleft = float(regions.get("fleft", 10.0))

    if front < 0.22:
        return 0.0, 0.65 if follow_side == "right" else -0.65
    if front < 0.40:
        return 0.08, 0.45 if follow_side == "right" else -0.45

    target_wall = right if follow_side == "right" else left
    forward_wall = fright if follow_side == "right" else fleft
    if target_wall > 1.0 and forward_wall > 1.0:
        return 0.16, -0.20 if follow_side == "right" else 0.20
    error = 0.38 - target_wall
    if follow_side == "left":
        error = -error
    angular = _clamp(1.4 * error, -0.45, 0.45)
    return 0.18, angular


def should_start_scan(
    phase: MissionPhase,
    regions: dict[str, float],
    seconds_since_last_tag: float,
    seconds_since_last_scan: float,
) -> bool:
    if phase not in (
        MissionPhase.STATE_START,
        MissionPhase.STATE_RETURN_TAG2,
    ):
        return False
    min_scan_gap = 4.0 if phase is MissionPhase.STATE_RETURN_TAG2 else 6.0
    if seconds_since_last_scan < min_scan_gap or seconds_since_last_tag < 3.5:
        return False
    front = float(regions.get("front", 10.0))
    right = float(regions.get("right", 10.0))
    fright = float(regions.get("fright", 10.0))
    left = float(regions.get("left", 10.0))
    fleft = float(regions.get("fleft", 10.0))
    side_opening = max(right, fright, left, fleft) > 1.0
    approaching_corner = front < 0.45
    return side_opening or approaching_corner


def should_start_color_scan(
    phase: MissionPhase,
    seconds_since_last_tag: float,
    seconds_since_last_scan: float,
) -> bool:
    return (
        phase is MissionPhase.STATE_TAG4_UTURN
        and seconds_since_last_tag > 6.0
        and seconds_since_last_scan > 5.0
    )


def should_log_tag_event(phase: MissionPhase, mission_label: int, hint_consumed: bool) -> bool:
    if phase is MissionPhase.STATE_START and int(mission_label) == 2 and not hint_consumed:
        return False
    return int(mission_label) in {1, 2, 3, 4, 5}


def decision_for_tag_event(phase: MissionPhase, mission_label: int, hint_consumed: bool) -> str:
    mission_label = int(mission_label)
    if phase is MissionPhase.STATE_START and mission_label == 2 and not hint_consumed:
        return "TAKE_RIGHT_HINT"
    if mission_label == 2:
        return "TURN_LEFT"
    if mission_label == 1:
        return "TURN_LEFT"
    if mission_label == 3:
        return "START_GREEN_AND_U_TURN"
    if mission_label == 4:
        return "U_TURN"
    if mission_label == 5:
        return "START_ORANGE"
    return "NO_ACTION"


def phase_entry_maneuver_steps(phase: MissionPhase) -> list[tuple[float, float, float]]:
    if phase is MissionPhase.STATE_TAG2_FOUND:
        return [(1.5, 0.0, -0.65), (1.2, 0.14, 0.0)]
    if phase is MissionPhase.STATE_RETURN_TAG2:
        return [(1.2, 0.10, 0.20)]
    if phase is MissionPhase.STATE_TAG3_GREEN_PATH:
        return [(2.6, 0.0, 0.85), (1.5, 0.13, 0.0)]
    if phase is MissionPhase.STATE_TAG1_DEAD_END:
        return [(2.5, 0.0, 0.85), (1.0, 0.12, 0.0)]
    if phase is MissionPhase.STATE_TAG4_UTURN:
        return [(2.5, 0.0, 0.85), (2.0, 0.15, 0.0)]
    return []


def should_handle_tag_event(
    seen_phase_tag_pairs: set[tuple[str, int]],
    phase: MissionPhase,
    tag_id: int,
) -> bool:
    key = (phase.value, int(tag_id))
    if key in seen_phase_tag_pairs:
        return False
    seen_phase_tag_pairs.add(key)
    return True


def should_defer_phase_seen_tracking(state: ControlState, mission_label: int) -> bool:
    return (
        state.phase is MissionPhase.STATE_START
        and int(mission_label) == 2
        and not state.first_hint_consumed
    )


class StateMachineController(Node):
    def __init__(self) -> None:
        if rclpy is None:  # pragma: no cover
            raise RuntimeError("rclpy is required to run the state machine node")
        super().__init__("state_machine_controller")
        self.declare_parameter("skip_tag1_return", False)
        self.declare_parameter("explore_linear_speed", 0.18)
        self.declare_parameter("follow_linear_speed", 0.14)
        self.declare_parameter("search_sweep_gain", 0.28)
        self.declare_parameter("search_sweep_frequency", 0.9)
        self.state = ControlState(skip_tag1_return=bool(self.get_parameter("skip_tag1_return").value))
        self.explore_linear_speed = float(self.get_parameter("explore_linear_speed").value)
        self.follow_linear_speed = float(self.get_parameter("follow_linear_speed").value)
        self.search_sweep_gain = float(self.get_parameter("search_sweep_gain").value)
        self.search_sweep_frequency = float(self.get_parameter("search_sweep_frequency").value)
        self.last_path_tracking = {"detected": False, "yaw_error": 0.0, "stop_detected": False}
        self.latest_scan_regions = {"right": 10.0, "fright": 10.0, "front": 10.0, "fleft": 10.0, "left": 10.0}
        self.seen_phase_tag_pairs: set[tuple[str, int]] = set()
        self.state.last_scan_monotonic = time.monotonic()
        self.state.scan_until = self.state.last_scan_monotonic + 2.2
        self.state.scan_angular_z = -0.45

        self.cmd_pub = self.create_publisher(Twist, "/nav/cmd_vel_raw", 10)
        self.color_pub = self.create_publisher(String, "/vision/target_color", 10)
        self.phase_pub = self.create_publisher(String, "/mission/state", 10)
        self.log_pub = self.create_publisher(String, "/mission/log_event", 10)

        self.create_subscription(String, "/vision/tag_event", self._on_tag_event, 10)
        self.create_subscription(String, "/vision/path_tracking", self._on_path_tracking, 10)
        self.create_subscription(Bool, "/safety/clearance", self._on_clearance, 10)
        self.create_subscription(LaserScan, "/r1_mini/lidar", self._on_scan, 10)
        self.timer = self.create_timer(0.1, self._tick)
        self._publish_phase()

    def _on_tag_event(self, msg: String) -> None:
        event = json.loads(msg.data)
        phase_before = self.state.phase
        tag_id = int(event["tag_id"])
        label = int(event["mission_label"])
        if should_defer_phase_seen_tracking(self.state, label):
            self.state.first_hint_consumed = True
            self.state.last_tag_event_monotonic = time.monotonic()
            return
        if not should_handle_tag_event(self.seen_phase_tag_pairs, phase_before, tag_id):
            return
        decision_taken = decision_for_tag_event(
            phase_before,
            mission_label=label,
            hint_consumed=self.state.first_hint_consumed,
        )
        should_log = should_log_tag_event(
            phase_before,
            mission_label=label,
            hint_consumed=self.state.first_hint_consumed,
        )

        self.state.last_tag_event_monotonic = time.monotonic()
        self.state.phase = advance_phase_on_tag(phase_before, label, self.state.skip_tag1_return)
        if self.state.phase is MissionPhase.STATE_TAG3_GREEN_PATH:
            self.state.target_color = "green"
        elif self.state.phase in (MissionPhase.STATE_TAG5_ORANGE_PATH, MissionPhase.STATE_FINAL_GOAL):
            self.state.target_color = "orange"
        if self.state.phase is not phase_before:
            self._start_phase_maneuver(self.state.phase)
            if should_log:
                self.log_pub.publish(String(data=json.dumps({
                    "event": "tag_log",
                    "tag_id": tag_id,
                    "mission_label": label,
                    "decision_taken": decision_taken,
                    "unix_timestamp": int(event.get("unix_timestamp", 0)),
                    "phase_before": phase_before.value,
                    "phase_after": self.state.phase.value,
                })))
        self._publish_phase()

    def _on_path_tracking(self, msg: String) -> None:
        self.last_path_tracking = json.loads(msg.data)
        self.state.last_yaw_error = float(self.last_path_tracking.get("yaw_error", 0.0))
        self.state.stop_detected = bool(self.last_path_tracking.get("stop_detected", False))
        if self.state.phase is MissionPhase.STATE_TAG5_ORANGE_PATH:
            self.state.phase = MissionPhase.STATE_FINAL_GOAL
            self._publish_phase()
        if self.state.phase is MissionPhase.STATE_FINAL_GOAL and self.state.stop_detected:
            self.state.phase = MissionPhase.STATE_STOP
            self.log_pub.publish(String(data=json.dumps({"event": "stop_detected"})))
            self._publish_phase()

    def _on_clearance(self, msg: Bool) -> None:
        self.state.latest_clearance_ok = bool(msg.data)

    def _on_scan(self, msg: LaserScan) -> None:
        self.latest_scan_regions = regions_from_ranges(list(msg.ranges))

    def _publish_phase(self) -> None:
        self.phase_pub.publish(String(data=self.state.phase.value))
        self.color_pub.publish(String(data=self.state.target_color))

    def _start_phase_maneuver(self, phase: MissionPhase) -> None:
        self.state.maneuver_queue = list(phase_entry_maneuver_steps(phase))
        self._advance_maneuver()

    def _advance_maneuver(self) -> None:
        if not self.state.maneuver_queue:
            self.state.maneuver_until = 0.0
            self.state.maneuver_linear_x = 0.0
            self.state.maneuver_angular_z = 0.0
            return
        duration_s, linear_x, angular_z = self.state.maneuver_queue.pop(0)
        if duration_s <= 0.0:
            self._advance_maneuver()
            return
        self.state.maneuver_until = time.monotonic() + duration_s
        self.state.maneuver_linear_x = linear_x
        self.state.maneuver_angular_z = angular_z

    def _tick(self) -> None:
        cmd = Twist()
        if not self.state.latest_clearance_ok:
            self.cmd_pub.publish(cmd)
            return
        if self.state.maneuver_until > 0.0 and time.monotonic() >= self.state.maneuver_until:
            self._advance_maneuver()
        if time.monotonic() < self.state.maneuver_until:
            cmd.linear.x = self.state.maneuver_linear_x
            cmd.angular.z = self.state.maneuver_angular_z
            self.cmd_pub.publish(cmd)
            return
        if time.monotonic() < self.state.scan_until:
            cmd.angular.z = self.state.scan_angular_z
            self.cmd_pub.publish(cmd)
            return
        if self.state.phase in (
            MissionPhase.STATE_START,
            MissionPhase.STATE_TAG2_FOUND,
            MissionPhase.STATE_TAG1_DEAD_END,
            MissionPhase.STATE_RETURN_TAG2,
        ):
            seconds_since_last_tag = (
                time.monotonic() - self.state.last_tag_event_monotonic
                if self.state.last_tag_event_monotonic > 0.0
                else 999.0
            )
            seconds_since_last_scan = (
                time.monotonic() - self.state.last_scan_monotonic
                if self.state.last_scan_monotonic > 0.0
                else 999.0
            )
            if should_start_scan(
                self.state.phase,
                self.latest_scan_regions,
                seconds_since_last_tag=seconds_since_last_tag,
                seconds_since_last_scan=seconds_since_last_scan,
            ):
                self.state.last_scan_monotonic = time.monotonic()
                self.state.scan_until = self.state.last_scan_monotonic + 1.7
                if self.state.phase is MissionPhase.STATE_TAG1_DEAD_END:
                    self.state.scan_angular_z = 0.55
                else:
                    open_right = max(
                        float(self.latest_scan_regions.get("right", 10.0)),
                        float(self.latest_scan_regions.get("fright", 10.0)),
                    )
                    open_left = max(
                        float(self.latest_scan_regions.get("left", 10.0)),
                        float(self.latest_scan_regions.get("fleft", 10.0)),
                    )
                    self.state.scan_angular_z = -0.55 if open_right >= open_left else 0.55
                cmd.angular.z = self.state.scan_angular_z
                self.cmd_pub.publish(cmd)
                return
            follow_side = "left" if self.state.phase is MissionPhase.STATE_TAG1_DEAD_END else self.state.follow_side
            linear_x, angular_z = wall_follow_command(self.latest_scan_regions, follow_side=follow_side)
            cmd.linear.x = linear_x
            cmd.angular.z = angular_z
            if self.state.phase is MissionPhase.STATE_START and self.state.last_tag_event_monotonic == 0.0:
                cmd.linear.x = min(cmd.linear.x, 0.10)
                if bool(self.last_path_tracking.get("tag_candidate_detected", False)):
                    cmd.angular.z = _clamp(
                        -0.9 * float(self.last_path_tracking.get("tag_candidate_yaw_error", 0.0)),
                        -0.5,
                        0.5,
                    )
            elif self.state.phase is MissionPhase.STATE_TAG1_DEAD_END:
                cmd.angular.z = min(cmd.angular.z, -0.15)
            elif time.monotonic() - self.state.last_tag_event_monotonic > 2.0:
                if bool(self.last_path_tracking.get("tag_candidate_detected", False)):
                    cmd.angular.z = _clamp(
                        -0.9 * float(self.last_path_tracking.get("tag_candidate_yaw_error", 0.0)),
                        -0.55,
                        0.55,
                    )
        elif self.state.phase in (
            MissionPhase.STATE_TAG3_GREEN_PATH,
            MissionPhase.STATE_TAG4_UTURN,
            MissionPhase.STATE_TAG5_ORANGE_PATH,
            MissionPhase.STATE_FINAL_GOAL,
        ):
            seconds_since_last_tag = (
                time.monotonic() - self.state.last_tag_event_monotonic
                if self.state.last_tag_event_monotonic > 0.0
                else 999.0
            )
            seconds_since_last_scan = (
                time.monotonic() - self.state.last_scan_monotonic
                if self.state.last_scan_monotonic > 0.0
                else 999.0
            )
            if should_start_color_scan(
                self.state.phase,
                seconds_since_last_tag=seconds_since_last_tag,
                seconds_since_last_scan=seconds_since_last_scan,
            ):
                self.state.last_scan_monotonic = time.monotonic()
                self.state.scan_until = self.state.last_scan_monotonic + 0.9
                self.state.color_scan_direction *= -1.0
                self.state.scan_angular_z = 0.42 * self.state.color_scan_direction
                cmd.angular.z = self.state.scan_angular_z
                self.cmd_pub.publish(cmd)
                return
            detected = bool(self.last_path_tracking.get("detected", False))
            linear_x, angular_z = compute_follow_command(
                detected,
                self.state.last_yaw_error,
                self.follow_linear_speed,
            )
            if not detected and bool(self.last_path_tracking.get("tag_candidate_detected", False)):
                linear_x = 0.08
                angular_z = _clamp(
                    -0.9 * float(self.last_path_tracking.get("tag_candidate_yaw_error", 0.0)),
                    -0.45,
                    0.45,
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
