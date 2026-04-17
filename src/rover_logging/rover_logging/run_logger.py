from __future__ import annotations

import csv
import json
import os
from datetime import datetime, timezone
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


def _make_run_dir() -> Path:
    """Create a timestamped run directory under ~/artpark_runs/."""
    stamp = datetime.now(tz=timezone.utc).strftime("%Y%m%dT%H%M%S")
    run_dir = Path.home() / "artpark_runs" / stamp
    run_dir.mkdir(parents=True, exist_ok=True)
    return run_dir


def build_tag_png_name(unix_timestamp: int, seq: int = 0, label: int = 0) -> str:
    return f"{int(unix_timestamp)}_{seq:03d}_tag{label}.png"


def build_tag_log_row(unix_timestamp: int, tag_id: int, decision: str, image_name: str) -> list[str]:
    return [str(int(unix_timestamp)), str(int(tag_id)), str(decision), str(image_name)]


class RunLogger(Node):
    def __init__(self) -> None:
        if rclpy is None:  # pragma: no cover
            raise RuntimeError("rclpy is required to run the logger node")
        if CvBridge is None:  # pragma: no cover
            raise RuntimeError("cv_bridge is required to run the logger node")
        super().__init__("run_logger")

        self.output_dir = _make_run_dir()
        self.images_dir = self.output_dir / "images"
        self.images_dir.mkdir(parents=True, exist_ok=True)
        self.csv_path = self.output_dir / "judge_scorecard.csv"
        self.thought_log_path = self.output_dir / "thought_log.jsonl"
        self.bridge = CvBridge()
        self.latest_image = None
        self.image_seq = 0

        with self.csv_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(["Timestamp", "Tag_ID", "Decision_Taken", "Image_File"])

        self.create_subscription(Image, "/camera/image_raw", self._on_image, 10)
        self.create_subscription(String, "/vision/tag_event", self._on_tag_event, 10)
        self.create_subscription(String, "/vision/tile_event", self._on_tile_event, 10)
        self.create_subscription(String, "/mission/log_event", self._on_log_event, 10)

        self.get_logger().info(f"Logging to {self.output_dir}")

    def _on_image(self, msg: Image) -> None:
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def _append_row(self, row: list[str]) -> None:
        with self.csv_path.open("a", newline="", encoding="utf-8") as handle:
            csv.writer(handle).writerow(row)

    def _save_image(self, unix_timestamp: int, label: int = 0) -> str:
        self.image_seq += 1
        image_name = build_tag_png_name(unix_timestamp, self.image_seq, label)
        if self.latest_image is not None and cv2 is not None:
            cv2.imwrite(str(self.images_dir / image_name), self.latest_image)
        return image_name

    def _on_tag_event(self, msg: String) -> None:
        payload = json.loads(msg.data)
        unix_timestamp = int(payload["unix_timestamp"])
        mission_label = int(payload["mission_label"])
        image_name = self._save_image(unix_timestamp, mission_label)
        self._append_row(build_tag_log_row(
            unix_timestamp=unix_timestamp,
            tag_id=mission_label,
            decision=str(payload["decision_taken"]),
            image_name=image_name,
        ))

    def _on_tile_event(self, msg: String) -> None:
        payload = json.loads(msg.data)
        with (self.output_dir / "tile_counts.jsonl").open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(payload) + os.linesep)

    def _on_log_event(self, msg: String) -> None:
        with self.thought_log_path.open("a", encoding="utf-8") as handle:
            handle.write(msg.data + os.linesep)


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
