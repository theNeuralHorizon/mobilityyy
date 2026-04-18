from __future__ import annotations

import math

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool
except ImportError:  # pragma: no cover
    rclpy = None
    Node = object
    Twist = None
    LaserScan = None
    Bool = None


def apply_safety_override(
    requested_linear_x: float,
    requested_angular_z: float,
    front_min_m: float,
    left_min_m: float,
    right_min_m: float,
    hard_stop_distance_m: float,
) -> tuple[float, float, bool]:
    if 0.0 < front_min_m < hard_stop_distance_m:
        turn_sign = 1.0 if left_min_m >= right_min_m else -1.0
        return 0.0, 0.6 * turn_sign, True
    return requested_linear_x, requested_angular_z, False


def _sector_min(ranges: list[float], start: int, end: int) -> float:
    valid = [value for value in ranges[start:end] if math.isfinite(value) and value > 0.0]
    return min(valid) if valid else float("inf")


def _window_min(ranges: list[float], center_index: int, half_width: int) -> float:
    count = len(ranges)
    values = []
    for offset in range(-half_width, half_width + 1):
        index = (center_index + offset) % count
        value = ranges[index]
        if math.isfinite(value) and value > 0.0:
            values.append(value)
    return min(values) if values else float("inf")


class SafetyController(Node):
    def __init__(self) -> None:
        if rclpy is None:  # pragma: no cover
            raise RuntimeError("rclpy is required to run the safety node")
        super().__init__("safety_controller")
        self.declare_parameter("hard_stop_distance_m", 0.2)
        self.declare_parameter("scan_topic", "/r1_mini/lidar")
        self.hard_stop_distance_m = float(self.get_parameter("hard_stop_distance_m").value)
        scan_topic = str(self.get_parameter("scan_topic").value)
        self.latest_scan = None
        self.requested_linear_x = 0.0
        self.requested_angular_z = 0.0

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.clearance_pub = self.create_publisher(Bool, "/safety/clearance", 10)
        self.override_pub = self.create_publisher(Bool, "/safety/override_active", 10)

        self.create_subscription(Twist, "/nav/cmd_vel_raw", self._on_cmd, 10)
        self.create_subscription(LaserScan, scan_topic, self._on_scan, 10)
        self.timer = self.create_timer(0.05, self._tick)

    def _on_cmd(self, msg: Twist) -> None:
        self.requested_linear_x = float(msg.linear.x)
        self.requested_angular_z = float(msg.angular.z)

    def _on_scan(self, msg: LaserScan) -> None:
        self.latest_scan = list(msg.ranges)

    def _tick(self) -> None:
        cmd = Twist()
        if not self.latest_scan:
            self.cmd_pub.publish(cmd)
            self.clearance_pub.publish(Bool(data=True))
            self.override_pub.publish(Bool(data=False))
            return
        count = len(self.latest_scan)
        center = count // 2
        quarter = count // 4
        half_width = max(count // 18, 1)
        front = _window_min(self.latest_scan, center, half_width)
        left = _window_min(self.latest_scan, (center + quarter) % count, half_width)
        right = _window_min(self.latest_scan, (center - quarter) % count, half_width)
        linear_x, angular_z, active = apply_safety_override(
            self.requested_linear_x,
            self.requested_angular_z,
            front,
            left,
            right,
            self.hard_stop_distance_m,
        )
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_pub.publish(cmd)
        self.clearance_pub.publish(Bool(data=not active))
        self.override_pub.publish(Bool(data=active))


def main(args=None) -> None:  # pragma: no cover
    if rclpy is None:
        raise RuntimeError("rclpy is required to run this node")
    rclpy.init(args=args)
    node = SafetyController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
