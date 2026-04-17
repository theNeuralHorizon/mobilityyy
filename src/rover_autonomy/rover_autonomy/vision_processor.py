from __future__ import annotations

import json
import time

import cv2
import numpy as np

from .arrow_geometry import estimate_arrow_yaw_error, find_largest_arrow_contour
from .hsv_utils import lower_roi, make_mask
from .tag_mapper import TagMapper

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
except ImportError:  # pragma: no cover
    rclpy = None
    Node = object
    Image = None
    String = None

try:
    from cv_bridge import CvBridge
except ImportError:  # pragma: no cover
    CvBridge = None

try:
    from apriltag_msgs.msg import AprilTagDetectionArray
except ImportError:  # pragma: no cover
    AprilTagDetectionArray = None


class VisionProcessor(Node):
    def __init__(self) -> None:
        if rclpy is None:  # pragma: no cover
            raise RuntimeError("rclpy is required to run the vision node")
        if CvBridge is None:  # pragma: no cover
            raise RuntimeError("cv_bridge is required to run the vision node")
        super().__init__("vision_processor")
        self.declare_parameter("green_low", [35, 80, 60])
        self.declare_parameter("green_high", [90, 255, 255])
        self.declare_parameter("orange_low", [5, 120, 120])
        self.declare_parameter("orange_high", [25, 255, 255])
        self.declare_parameter("tag_id_to_label", [0, 1, 2, 3, 4])
        self.declare_parameter("debug_gui", False)
        self.bridge = CvBridge()
        self.target_color = "none"
        self.latest_image = None
        self.last_tile_commit_time = 0.0
        self.tile_counter = 0
        self.direction_tile_counter = {"green": 0, "orange": 0}
        self.last_tag_publish_times: dict[int, float] = {}
        self.stop_frame_count = 0
        self.stop_confirm_threshold = 5

        tag_labels = self.get_parameter("tag_id_to_label").value
        mapping = {idx: int(label) for idx, label in enumerate(tag_labels)}
        self.mapper = TagMapper(mapping)

        self.tag_pub = self.create_publisher(String, "/vision/tag_event", 10)
        self.path_pub = self.create_publisher(String, "/vision/path_tracking", 10)
        self.tile_pub = self.create_publisher(String, "/vision/tile_event", 10)
        self.debug_pub = self.create_publisher(Image, "/vision/debug_image", 5)

        self.create_subscription(Image, "/camera/image_raw", self._on_image, 10)
        if AprilTagDetectionArray is not None:
            self.create_subscription(AprilTagDetectionArray, "/tag_detections_native", self._on_native_tag_detections, 10)
        self.create_subscription(String, "/tag_detections", self._on_tag_detections, 10)
        self.create_subscription(String, "/vision/target_color", self._on_target_color, 10)
        self.timer = self.create_timer(0.1, self._tick)

    def _on_target_color(self, msg: String) -> None:
        self.target_color = msg.data.strip().lower()

    def _on_image(self, msg: Image) -> None:
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def _on_tag_detections(self, msg: String) -> None:
        payload = json.loads(msg.data)
        detections = payload.get("detections", [])
        self._publish_tag_events(detections)

    def _on_native_tag_detections(self, msg: AprilTagDetectionArray) -> None:
        detections = []
        for detection in msg.detections:
            pose = detection.pose.pose.pose.position
            distance_m = float((pose.x ** 2 + pose.y ** 2 + pose.z ** 2) ** 0.5)
            bearing_rad = float(np.arctan2(pose.x, pose.z if abs(pose.z) > 1e-6 else 1e-6))
            detections.append({
                "id": int(detection.id),
                "distance_m": distance_m,
                "bearing_rad": bearing_rad,
            })
        self._publish_tag_events(detections)

    def _publish_tag_events(self, detections: list[dict]) -> None:
        now = time.time()
        for detection in detections:
            tag_id = int(detection["id"])
            if now - self.last_tag_publish_times.get(tag_id, 0.0) < 1.0:
                continue
            event = self.mapper.build_event(
                tag_id=tag_id,
                distance_m=float(detection.get("distance_m", 0.0)),
                bearing_rad=float(detection.get("bearing_rad", 0.0)),
            )
            if event is None:
                self.get_logger().warn(f"Unmapped tag ID {tag_id}, skipping")
                continue
            event["unix_timestamp"] = int(now)
            self.tag_pub.publish(String(data=json.dumps(event)))
            self.last_tag_publish_times[tag_id] = now

    def _tick(self) -> None:
        if self.latest_image is None:
            return
        frame = self.latest_image
        roi, row_offset = lower_roi(frame)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        green_low = self.get_parameter("green_low").value
        green_high = self.get_parameter("green_high").value
        orange_low = self.get_parameter("orange_low").value
        orange_high = self.get_parameter("orange_high").value
        green_mask = make_mask(hsv, green_low, green_high)
        orange_mask = make_mask(hsv, orange_low, orange_high)

        active_color = self.target_color if self.target_color in ("green", "orange") else "green"
        active_mask = green_mask if active_color == "green" else orange_mask
        contour = find_largest_arrow_contour(active_mask)
        detected = contour is not None and cv2.contourArea(contour) > 300.0
        yaw_error = estimate_arrow_yaw_error(contour, roi.shape[1]) if detected else 0.0

        # Ratio-based STOP detection with multi-frame confirmation
        roi_area = max(orange_mask.shape[0] * orange_mask.shape[1], 1)
        orange_ratio = cv2.countNonZero(orange_mask) / roi_area
        if self.target_color == "orange" and orange_ratio > 0.15:
            self.stop_frame_count += 1
        else:
            self.stop_frame_count = 0
        stop_detected = self.stop_frame_count >= self.stop_confirm_threshold

        if detected and time.time() - self.last_tile_commit_time > 0.75:
            self.tile_counter += 1
            self.direction_tile_counter[active_color] += 1
            self.last_tile_commit_time = time.time()
            self.tile_pub.publish(String(data=json.dumps({
                "unix_timestamp": int(self.last_tile_commit_time),
                "total_tiles_seen": self.tile_counter,
                "green_direction_tiles_seen": self.direction_tile_counter["green"],
                "orange_direction_tiles_seen": self.direction_tile_counter["orange"],
                "active_color": active_color,
            })))

        self.path_pub.publish(String(data=json.dumps({
            "target_color": self.target_color,
            "detected": detected,
            "yaw_error": yaw_error,
            "forward_confidence": 0.8 if detected else 0.0,
            "direction_tile_seen": detected,
            "stop_detected": stop_detected,
            "roi_row_offset": row_offset,
        })))

        if self.debug_pub.get_subscription_count() > 0:
            debug_frame = frame.copy()
            cv2.line(debug_frame, (debug_frame.shape[1] // 2, 0), (debug_frame.shape[1] // 2, debug_frame.shape[0]), (255, 255, 255), 2)
            if detected:
                shifted = contour.copy()
                shifted[:, 0, 1] += row_offset
                cv2.drawContours(debug_frame, [shifted], -1, (0, 255, 0), 2)
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_frame, encoding="bgr8"))


def main(args=None) -> None:  # pragma: no cover
    if rclpy is None:
        raise RuntimeError("rclpy is required to run this node")
    rclpy.init(args=args)
    node = VisionProcessor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
