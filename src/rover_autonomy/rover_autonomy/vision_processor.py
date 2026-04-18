from __future__ import annotations

import json
import time
from pathlib import Path

import cv2
import numpy as np

from ament_index_python.packages import get_package_share_directory

from .arrow_geometry import estimate_arrow_yaw_error, estimate_mask_yaw_error, find_largest_arrow_contour
from .hsv_utils import lower_roi, make_mask
from .tag_mapper import TagMapper
from .tag_template_matcher import load_tag_templates, match_tag_patch

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
        self.declare_parameter("camera_image_topic", "/r1_mini/camera/image_raw")
        self.bridge = CvBridge()
        self.target_color = "none"
        self.latest_image = None
        self._aruco_detector = None
        self._aruco_params = None
        self.last_tile_commit_time = 0.0
        self.tile_counter = 0
        self.direction_tile_counter = {"green": 0, "orange": 0}
        self.last_tag_publish_times: dict[int, float] = {}
        self.tag_templates = {}

        if hasattr(cv2, "aruco"):
            april_dict_id = getattr(cv2.aruco, "DICT_APRILTAG_36h11", None)
            if april_dict_id is None:
                april_dict_id = getattr(cv2.aruco, "DICT_APRILTAG_36H11", None)
            if april_dict_id is None:
                april_dict_id = cv2.aruco.DICT_4X4_50
            aruco_dict = cv2.aruco.getPredefinedDictionary(april_dict_id)
            if hasattr(cv2.aruco, "ArucoDetector"):
                self._aruco_detector = cv2.aruco.ArucoDetector(
                    aruco_dict,
                    cv2.aruco.DetectorParameters(),
                )
            else:
                self._aruco_params = cv2.aruco.DetectorParameters_create()
                self._aruco_detector = aruco_dict
        try:
            template_dir = Path(get_package_share_directory("grid_world")) / "materials" / "textures" / "apriltags"
            self.tag_templates = load_tag_templates(template_dir)
        except Exception:
            self.tag_templates = {}

        tag_labels = self.get_parameter("tag_id_to_label").value
        mapping = {idx: int(label) for idx, label in enumerate(tag_labels)}
        self.mapper = TagMapper(mapping)

        self.tag_pub = self.create_publisher(String, "/vision/tag_event", 10)
        self.path_pub = self.create_publisher(String, "/vision/path_tracking", 10)
        self.tile_pub = self.create_publisher(String, "/vision/tile_event", 10)
        self.debug_pub = self.create_publisher(Image, "/vision/debug_image", 5)

        camera_image_topic = str(self.get_parameter("camera_image_topic").value)
        self.create_subscription(Image, camera_image_topic, self._on_image, 10)
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
        image_width = self.latest_image.shape[1] if self.latest_image is not None else 640
        for detection in msg.detections:
            normalized_offset = (float(detection.centre.x) - (image_width / 2.0)) / max(image_width / 2.0, 1.0)
            bearing_rad = float(normalized_offset * (np.pi / 4.0))
            detections.append({
                "id": int(detection.id),
                "distance_m": 0.0,
                "bearing_rad": bearing_rad,
            })
        self._publish_tag_events(detections)

    def _publish_tag_events(self, detections: list[dict]) -> None:
        now = time.time()
        for detection in detections:
            tag_id = int(detection["id"])
            if now - self.last_tag_publish_times.get(tag_id, 0.0) < 1.0:
                continue
            try:
                event = self.mapper.build_event(
                    tag_id=tag_id,
                    distance_m=float(detection.get("distance_m", 0.0)),
                    bearing_rad=float(detection.get("bearing_rad", 0.0)),
                )
            except (KeyError, IndexError):
                self.get_logger().warn(f"Skipping unknown tag id {tag_id}")
                continue
            event["unix_timestamp"] = int(now)
            self.tag_pub.publish(String(data=json.dumps(event)))
            self.last_tag_publish_times[tag_id] = now

    def _detect_aruco_tags(self, frame: np.ndarray) -> list[dict]:
        if self._aruco_detector is None:
            return []
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)
        if self._aruco_params is None:
            corners, ids, _ = self._aruco_detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray,
                self._aruco_detector,
                parameters=self._aruco_params,
            )
        if ids is None:
            return self._detect_template_tags(frame)
        image_width = frame.shape[1]
        detections = []
        for idx, marker_id in enumerate(ids.flatten().tolist()):
            corner_set = corners[idx][0]
            widths = corner_set[:, 0].max() - corner_set[:, 0].min()
            heights = corner_set[:, 1].max() - corner_set[:, 1].min()
            marker_size = max(widths, heights)
            if marker_size < 10.0:
                continue
            center = corner_set.mean(axis=0)
            normalized_offset = (float(center[0]) - (image_width / 2.0)) / max(image_width / 2.0, 1.0)
            detections.append({
                "id": int(marker_id),
                "distance_m": 0.0,
                "bearing_rad": float(normalized_offset * (np.pi / 4.0)),
            })
        return detections

    def _order_quad(self, points: np.ndarray) -> np.ndarray:
        pts = points.astype(np.float32)
        s = pts.sum(axis=1)
        diff = np.diff(pts, axis=1)
        ordered = np.zeros((4, 2), dtype=np.float32)
        ordered[0] = pts[np.argmin(s)]
        ordered[2] = pts[np.argmax(s)]
        ordered[1] = pts[np.argmin(diff)]
        ordered[3] = pts[np.argmax(diff)]
        return ordered

    def _detect_template_tags(self, frame: np.ndarray) -> list[dict]:
        if len(self.tag_templates) == 0:
            return []
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)
        edges = cv2.Canny(gray, 70, 180)
        contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        image_width = frame.shape[1]
        detections = []
        seen_ids: set[int] = set()
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 120.0 or area > 12000.0:
                continue
            perimeter = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.06 * perimeter, True)
            if len(approx) != 4:
                continue
            x, y, w, h = cv2.boundingRect(approx)
            if w < 12 or h < 12 or y > int(frame.shape[0] * 0.8):
                continue
            aspect = w / float(h)
            if aspect < 0.5 or aspect > 1.6:
                continue
            ordered = self._order_quad(approx.reshape(4, 2))
            target = np.array([[0, 0], [127, 0], [127, 127], [0, 127]], dtype=np.float32)
            matrix = cv2.getPerspectiveTransform(ordered, target)
            warped = cv2.warpPerspective(gray, matrix, (128, 128))
            tag_id, score = match_tag_patch(warped, self.tag_templates)
            if tag_id is None or score > 0.22 or tag_id in seen_ids:
                continue
            seen_ids.add(tag_id)
            center_x = x + (w / 2.0)
            normalized_offset = (float(center_x) - (image_width / 2.0)) / max(image_width / 2.0, 1.0)
            detections.append({
                "id": int(tag_id),
                "distance_m": 0.0,
                "bearing_rad": float(normalized_offset * (np.pi / 4.0)),
            })
        return detections

    def _detect_tag_candidate(self, frame: np.ndarray) -> tuple[bool, float]:
        height, width = frame.shape[:2]
        roi = frame[: max(int(height * 0.65), 1), :]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 70, 180)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best_score = 0.0
        best_center_x = 0.0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 80.0 or area > 6000.0:
                continue
            perimeter = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.08 * perimeter, True)
            if len(approx) != 4:
                continue
            x, y, w, h = cv2.boundingRect(approx)
            if w < 10 or h < 10:
                continue
            aspect = w / float(h)
            if aspect < 0.6 or aspect > 1.4:
                continue
            patch = gray[y : y + h, x : x + w]
            if patch.size == 0:
                continue
            contrast = float(np.std(patch))
            if contrast < 25.0:
                continue
            score = area * contrast
            if score > best_score:
                best_score = score
                best_center_x = x + (w / 2.0)
        if best_score <= 0.0:
            return False, 0.0
        yaw_error = (best_center_x - (width / 2.0)) / max(width / 2.0, 1.0)
        return True, float(yaw_error)

    def _tick(self) -> None:
        if self.latest_image is None:
            return
        frame = self.latest_image
        self._publish_tag_events(self._detect_aruco_tags(frame))
        tag_candidate_detected, tag_candidate_yaw_error = self._detect_tag_candidate(frame)
        roi, row_offset = lower_roi(frame)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        green_low = self.get_parameter("green_low").value
        green_high = self.get_parameter("green_high").value
        orange_low = self.get_parameter("orange_low").value
        orange_high = self.get_parameter("orange_high").value
        green_mask = make_mask(hsv, green_low, green_high)
        orange_mask = make_mask(hsv, orange_low, orange_high)

        active_color = self.target_color if self.target_color in ("green", "orange") else "none"
        active_mask = None
        contour = None
        detected = False
        yaw_error = 0.0
        stop_detected = False
        if active_color == "green":
            active_mask = green_mask
        elif active_color == "orange":
            active_mask = orange_mask
            stop_detected = bool(cv2.countNonZero(orange_mask) > 7000)
        if active_mask is not None:
            contour = find_largest_arrow_contour(active_mask)
            if contour is not None and cv2.contourArea(contour) > 300.0:
                detected = True
                yaw_error = estimate_arrow_yaw_error(contour, roi.shape[1])
            elif cv2.countNonZero(active_mask) > 700:
                detected = True
                yaw_error = estimate_mask_yaw_error(active_mask, roi.shape[1])

        if active_color != "none" and detected and time.time() - self.last_tile_commit_time > 0.75:
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
            "tag_candidate_detected": tag_candidate_detected,
            "tag_candidate_yaw_error": tag_candidate_yaw_error,
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
