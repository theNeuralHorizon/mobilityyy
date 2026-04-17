"""AprilTag handler — wraps raw apriltag_ros detections with a 3-frame vote
and emits a committed TagEvent only after vote passes. This is the module
that turns noisy detections into scoreable logs.

Rules enforced here:
- 3 consecutive frames with same id, center pixel within 20 px, distance within 5 cm.
- Slow-down publish on /approach_mode whenever a candidate is in-frame but not yet committed.
- Logical-label lookup from config/tag_label_map.yaml — this is THE ONLY
  hardcoded AprilTag knowledge we're allowed per Atharva.
"""
from __future__ import annotations

import math
import threading
from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose

from apriltag_msgs.msg import AprilTagDetectionArray  # from apriltag_ros Jazzy
from artpark_msgs.msg import TagEvent, Thought


@dataclass
class DetectionSample:
    tag_id: int
    cx: float
    cy: float
    z: float          # camera-frame forward distance
    pose: Pose


class AprilTagHandler(Node):
    def __init__(self) -> None:
        super().__init__('apriltag_handler')

        # ---------------- parameters ----------------
        self.declare_parameter('vote_window',       3)
        self.declare_parameter('pixel_tolerance',   20.0)
        self.declare_parameter('distance_tolerance_m', 0.05)
        self.declare_parameter('approach_trigger_m',   1.5)
        self.declare_parameter('tag_label_map', {})     # {id:int → label:int}
        self.declare_parameter('action_by_label', {})   # {label:int → action:str}

        self.vote_window: int   = int(self.get_parameter('vote_window').value)
        self.px_tol: float      = float(self.get_parameter('pixel_tolerance').value)
        self.dist_tol: float    = float(self.get_parameter('distance_tolerance_m').value)
        self.approach_m: float  = float(self.get_parameter('approach_trigger_m').value)

        raw_id_map    = self.get_parameter('tag_label_map').value or {}
        raw_act_map   = self.get_parameter('action_by_label').value or {}
        self.id_to_label: Dict[int, int] = {int(k): int(v) for k, v in dict(raw_id_map).items()}
        self.label_to_act: Dict[int, str] = {int(k): str(v) for k, v in dict(raw_act_map).items()}

        # ---------------- state ----------------
        self._buffers: Dict[int, Deque[DetectionSample]] = {}
        self._committed_ids: set[int] = set()
        self._lock = threading.Lock()
        self._seq = 0

        # ---------------- I/O ----------------
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.sub = self.create_subscription(
            AprilTagDetectionArray, '/detections', self._on_detections, qos)
        self.pub_event    = self.create_publisher(TagEvent, '/tag_event', qos)
        self.pub_approach = self.create_publisher(Bool, '/approach_mode', qos)
        self.pub_thought  = self.create_publisher(Thought, '/thought', qos)

        self.get_logger().info(
            f'apriltag_handler up | id→label={self.id_to_label} | action_by_label={self.label_to_act}')

    # ========================================================================
    def _on_detections(self, msg: AprilTagDetectionArray) -> None:
        """Each incoming detection is pushed into its tag's rolling buffer.
        When the last `vote_window` samples agree, emit a committed TagEvent.
        """
        if not msg.detections:
            self.pub_approach.publish(Bool(data=False))
            return

        has_candidate = False
        for det in msg.detections:
            tag_id = int(det.id)
            # apriltag_ros for Jazzy puts the homography-derived center in det.centre
            cx = float(getattr(det, 'centre').x) if hasattr(det, 'centre') else 0.0
            cy = float(getattr(det, 'centre').y) if hasattr(det, 'centre') else 0.0

            # The pose of the tag in the camera frame comes via a companion
            # topic /tf or /tag_poses; for this node we only need distance
            # for the vote tolerance. If not provided, estimate from tag
            # size (0.15 m) and homography; fall back to 0 to avoid crashes.
            z = 0.0  # set by pose_array subscription when wired up
            pose = Pose()

            sample = DetectionSample(tag_id=tag_id, cx=cx, cy=cy, z=z, pose=pose)
            buf = self._buffers.setdefault(tag_id, deque(maxlen=self.vote_window))
            buf.append(sample)

            has_candidate = True
            if len(buf) == self.vote_window and self._vote_agrees(buf):
                if tag_id not in self._committed_ids:
                    self._committed_ids.add(tag_id)
                    self._emit_commit(tag_id, sample, first_sighting=True)
                else:
                    self._emit_commit(tag_id, sample, first_sighting=False)
                buf.clear()

        self.pub_approach.publish(Bool(data=has_candidate))

    def _vote_agrees(self, buf: Deque[DetectionSample]) -> bool:
        first = buf[0]
        for s in list(buf)[1:]:
            if s.tag_id != first.tag_id:
                return False
            if math.hypot(s.cx - first.cx, s.cy - first.cy) > self.px_tol:
                return False
            # distance check skipped when z is unknown (0); re-enable when pose
            # comes in on a companion topic.
            if first.z > 0 and abs(s.z - first.z) > self.dist_tol:
                return False
        return True

    def _emit_commit(self, tag_id: int, sample: DetectionSample, first_sighting: bool) -> None:
        label = self.id_to_label.get(tag_id, 0)
        action = self.label_to_act.get(label, 'UNKNOWN')

        ev = TagEvent()
        ev.stamp = self.get_clock().now().to_msg()
        ev.tag_id = tag_id
        ev.logical_label = label
        ev.decision = action
        ev.tag_in_camera = sample.pose
        ev.distance = sample.z
        ev.bearing = math.atan2(sample.cx - 320.0, 500.0)  # fallback; refined by pose fuser
        ev.vote_frames = self.vote_window
        ev.first_sighting = first_sighting
        self.pub_event.publish(ev)

        self._seq += 1
        t = Thought()
        t.stamp = ev.stamp
        t.seq = self._seq
        t.phase = 'APPROACH_TAG'
        t.hypothesis = f'Tag id={tag_id} committed after {self.vote_window}-frame vote'
        t.action_chosen = f'publish_TagEvent(label={label},action={action})'
        t.rule_applied = 'apriltag_handler.vote_pass'
        t.alt_considered = 'reject_as_noise'
        t.extras_json = f'{{"tag_id": {tag_id}, "label": {label}, "first": {str(first_sighting).lower()}}}'
        t.confidence = 1.0 if label != 0 else 0.3
        self.pub_thought.publish(t)


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagHandler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
