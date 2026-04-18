from __future__ import annotations

import csv
import json
import os
from pathlib import Path

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None

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


def build_tag_png_name(unix_timestamp: int) -> str:
    return f"{int(unix_timestamp)}.png"


def build_tag_log_row(unix_timestamp: int, tag_id: int, decision: str, image_name: str) -> list[str]:
    return [str(int(unix_timestamp)), str(int(tag_id)), str(decision), str(image_name)]


class RunLogger(Node):
    def __init__(self) -> None:
        if rclpy is None:  # pragma: no cover
            raise RuntimeError("rclpy is required to run the logger node")
        if CvBridge is None:  # pragma: no cover
            raise RuntimeError("cv_bridge is required to run the logger node")
        super().__init__("run_logger")
        self.declare_parameter("output_dir", str(Path.home() / "rover_run_logs"))
        self.declare_parameter("camera_image_topic", "/r1_mini/camera/image_raw")
        self.output_dir = Path(self.get_parameter("output_dir").value)
        camera_image_topic = str(self.get_parameter("camera_image_topic").value)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.images_dir = self.output_dir / "images"
        self.images_dir.mkdir(parents=True, exist_ok=True)
        self.csv_path = self.output_dir / "run_log.csv"
        self.bridge = CvBridge()
        self.latest_image = None

        with self.csv_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(["Timestamp", "Tag_ID", "Decision_Taken", "Image_File"])

        self.create_subscription(Image, camera_image_topic, self._on_image, 10)
        self.create_subscription(String, "/vision/tag_event", self._on_tag_event, 10)
        self.create_subscription(String, "/vision/path_tracking", self._on_path_tracking, 10)
        self.create_subscription(String, "/vision/tile_event", self._on_tile_event, 10)
        self.create_subscription(String, "/mission/state", self._on_mission_state, 10)
        self.create_subscription(String, "/mission/log_event", self._on_log_event, 10)

    def _on_image(self, msg: Image) -> None:
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def _append_row(self, row: list[str]) -> None:
        with self.csv_path.open("a", newline="", encoding="utf-8") as handle:
            csv.writer(handle).writerow(row)

    def _save_image(self, unix_timestamp: int) -> str:
        image_name = build_tag_png_name(unix_timestamp)
        if self.latest_image is not None and cv2 is not None:
            cv2.imwrite(str(self.images_dir / image_name), self.latest_image)
        return image_name

    def _append_jsonl(self, filename: str, payload: dict) -> None:
        with (self.output_dir / filename).open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(payload) + os.linesep)

    def _on_tag_event(self, msg: String) -> None:
        payload = json.loads(msg.data)
        self._append_jsonl("tag_events.jsonl", payload)

    def _on_path_tracking(self, msg: String) -> None:
        payload = json.loads(msg.data)
        payload["logged_at_ns"] = int(self.get_clock().now().nanoseconds)
        self._append_jsonl("path_tracking.jsonl", payload)

    def _on_tile_event(self, msg: String) -> None:
        payload = json.loads(msg.data)
        self._append_jsonl("tile_counts.jsonl", payload)

    def _on_mission_state(self, msg: String) -> None:
        payload = {
            "logged_at_ns": int(self.get_clock().now().nanoseconds),
            "state": msg.data,
        }
        self._append_jsonl("mission_state.jsonl", payload)

    def _on_log_event(self, msg: String) -> None:
        payload = json.loads(msg.data)
        if payload.get("event") == "tag_log":
            unix_timestamp = int(payload["unix_timestamp"])
            image_name = self._save_image(unix_timestamp)
            self._append_row(build_tag_log_row(
                unix_timestamp=unix_timestamp,
                tag_id=int(payload["mission_label"]),
                decision=str(payload["decision_taken"]),
                image_name=image_name,
            ))
        self._append_jsonl("mission_events.jsonl", payload)


def main(args=None) -> None:  # pragma: no cover
    if rclpy is None:
        raise RuntimeError("rclpy is required to run this node")
    rclpy.init(args=args)
    node = RunLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
