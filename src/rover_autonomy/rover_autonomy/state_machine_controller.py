from __future__ import annotations

import json
import math
from dataclasses import dataclass
from enum import Enum

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from std_msgs.msg import Bool, String
except ImportError:  # pragma: no cover
    rclpy = None
    Node = object
    Twist = None
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


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def compute_follow_command(target_detected: bool, yaw_error: float, linear_speed: float) -> tuple[float, float]:
    if not target_detected:
        return 0.0, 0.35
    return linear_speed, _clamp(-0.9 * yaw_error, -1.0, 1.0)


class StateMachineController(Node):
    def __init__(self) -> None:
        if rclpy is None:  # pragma: no cover
            raise RuntimeError("rclpy is required to run the state machine node")
        super().__init__("state_machine_controller")
        self.declare_parameter("skip_tag1_return", False)
        self.declare_parameter("explore_linear_speed", 0.18)
        self.declare_parameter("follow_linear_speed", 0.14)
        self.state = ControlState(skip_tag1_return=bool(self.get_parameter("skip_tag1_return").value))
        self.explore_linear_speed = float(self.get_parameter("explore_linear_speed").value)
        self.follow_linear_speed = float(self.get_parameter("follow_linear_speed").value)
        self.last_path_tracking = {"detected": False, "yaw_error": 0.0, "stop_detected": False}

        self.cmd_pub = self.create_publisher(Twist, "/nav/cmd_vel_raw", 10)
        self.color_pub = self.create_publisher(String, "/vision/target_color", 10)
        self.phase_pub = self.create_publisher(String, "/mission/state", 10)
        self.log_pub = self.create_publisher(String, "/mission/log_event", 10)

        self.create_subscription(String, "/vision/tag_event", self._on_tag_event, 10)
        self.create_subscription(String, "/vision/path_tracking", self._on_path_tracking, 10)
        self.create_subscription(Bool, "/safety/clearance", self._on_clearance, 10)
        self.timer = self.create_timer(0.1, self._tick)
        self._publish_phase()

    def _on_tag_event(self, msg: String) -> None:
        event = json.loads(msg.data)
        label = int(event["mission_label"])
        self.state.phase = advance_phase_on_tag(self.state.phase, label, self.state.skip_tag1_return)
        if self.state.phase is MissionPhase.STATE_TAG3_GREEN_PATH:
            self.state.target_color = "green"
        elif self.state.phase in (MissionPhase.STATE_TAG5_ORANGE_PATH, MissionPhase.STATE_FINAL_GOAL):
            self.state.target_color = "orange"
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

    def _publish_phase(self) -> None:
        self.phase_pub.publish(String(data=self.state.phase.value))
        self.color_pub.publish(String(data=self.state.target_color))

    def _tick(self) -> None:
        cmd = Twist()
        if not self.state.latest_clearance_ok:
            self.cmd_pub.publish(cmd)
            return
        if self.state.phase in (
            MissionPhase.STATE_START,
            MissionPhase.STATE_TAG2_FOUND,
            MissionPhase.STATE_TAG1_DEAD_END,
            MissionPhase.STATE_RETURN_TAG2,
        ):
            cmd.linear.x = self.explore_linear_speed
            cmd.angular.z = -0.35 if self.state.phase is MissionPhase.STATE_TAG1_DEAD_END else 0.0
        elif self.state.phase in (
            MissionPhase.STATE_TAG3_GREEN_PATH,
            MissionPhase.STATE_TAG4_UTURN,
            MissionPhase.STATE_TAG5_ORANGE_PATH,
            MissionPhase.STATE_FINAL_GOAL,
        ):
            detected = bool(self.last_path_tracking.get("detected", False))
            linear_x, angular_z = compute_follow_command(
                detected,
                self.state.last_yaw_error,
                self.follow_linear_speed,
            )
            cmd.linear.x = linear_x
            cmd.angular.z = angular_z
            if self.state.phase is MissionPhase.STATE_TAG4_UTURN:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.7
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
