"""Floor logo detector — reads the downward-facing camera, crops to the
current tile's bbox (estimated from LiDAR wall distances + odom), runs HSV
thresholds for green and orange, and computes per-edge pixel counts.

The edge-sample output is consumed by the state machine, which applies the
rules (exclude entry edge, break ties, LiDAR veto) and picks an exit. This
node does NOT make motion decisions; it is a pure perception module.
"""
from __future__ import annotations

import time
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from artpark_msgs.msg import EdgeSample, Thought


class FloorLogoDetector(Node):
    def __init__(self) -> None:
        super().__init__('floor_logo_detector')

        # ---- HSV thresholds (calibrate on physical arena) ----
        self.declare_parameter('hsv_green_low',  [35,  80,  60])
        self.declare_parameter('hsv_green_high', [85, 255, 255])
        self.declare_parameter('hsv_orange_low',  [5, 130, 120])
        self.declare_parameter('hsv_orange_high',[22, 255, 255])

        # Fraction of image used per edge strip (5% inset, 15% depth).
        self.declare_parameter('edge_inset',  0.05)
        self.declare_parameter('edge_depth',  0.15)
        self.declare_parameter('publish_rate_hz', 8.0)

        self.gl = np.array(self.get_parameter('hsv_green_low').value,  dtype=np.uint8)
        self.gh = np.array(self.get_parameter('hsv_green_high').value, dtype=np.uint8)
        self.ol = np.array(self.get_parameter('hsv_orange_low').value, dtype=np.uint8)
        self.oh = np.array(self.get_parameter('hsv_orange_high').value, dtype=np.uint8)
        self.inset = float(self.get_parameter('edge_inset').value)
        self.depth = float(self.get_parameter('edge_depth').value)

        self.bridge = CvBridge()
        self._latest: Optional[np.ndarray] = None
        self._last_publish = 0.0

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(
            Image, '/floor_cam/image_raw', self._on_image, qos)
        self.pub = self.create_publisher(EdgeSample, '/edge_sample',
                                         QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.pub_thought = self.create_publisher(Thought, '/thought',
                                                  QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.get_logger().info('floor_logo_detector up')

        self.timer = self.create_timer(
            1.0 / float(self.get_parameter('publish_rate_hz').value), self._tick)
        self._seq = 0

    # ------------------------------------------------------------
    def _on_image(self, msg: Image) -> None:
        try:
            self._latest = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'cv_bridge failure: {exc}')

    # ------------------------------------------------------------
    def _tick(self) -> None:
        if self._latest is None:
            return

        img = self._latest
        h, w = img.shape[:2]
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        mask_g = cv2.inRange(hsv, self.gl, self.gh)
        mask_o = cv2.inRange(hsv, self.ol, self.oh)

        # Edge strip geometry (pixel-space)
        inset_px = int(self.inset * min(h, w))
        depth_px = int(self.depth * min(h, w))

        # North strip: top of image → robot's "forward" edge of tile.
        n_box = (inset_px, inset_px + depth_px, inset_px, w - inset_px)
        s_box = (h - inset_px - depth_px, h - inset_px, inset_px, w - inset_px)
        w_box = (inset_px, h - inset_px, inset_px, inset_px + depth_px)
        e_box = (inset_px, h - inset_px, w - inset_px - depth_px, w - inset_px)

        def count(mask, b):
            r0, r1, c0, c1 = b
            return int(cv2.countNonZero(mask[r0:r1, c0:c1]))

        gN, gS, gW, gE = count(mask_g, n_box), count(mask_g, s_box), count(mask_g, w_box), count(mask_g, e_box)
        oN, oS, oW, oE = count(mask_o, n_box), count(mask_o, s_box), count(mask_o, w_box), count(mask_o, e_box)

        now = time.monotonic()
        if now - self._last_publish < 1.0 / float(self.get_parameter('publish_rate_hz').value):
            return
        self._last_publish = now

        es = EdgeSample()
        es.stamp = self.get_clock().now().to_msg()
        es.green_n, es.green_s, es.green_e, es.green_w = gN, gS, gE, gW
        es.orange_n, es.orange_s, es.orange_e, es.orange_w = oN, oS, oE, oW
        es.entry_edge = ''          # filled by state machine from its memory
        es.recommended_exit = ''    # filled by state machine after applying rules
        es.confidence = 0.0
        self.pub.publish(es)

        self._seq += 1
        t = Thought()
        t.stamp = es.stamp
        t.seq = self._seq
        t.phase = 'PERCEPTION'
        t.hypothesis = f'green edges N={gN} S={gS} E={gE} W={gW} | orange N={oN} S={oS} E={oE} W={oW}'
        t.action_chosen = 'publish_edge_sample'
        t.rule_applied = 'floor_logo_detector.edge_hsv_sample'
        t.alt_considered = ''
        t.extras_json = (
            f'{{"green":{{"n":{gN},"s":{gS},"e":{gE},"w":{gW}}},'
            f'"orange":{{"n":{oN},"s":{oS},"e":{oE},"w":{oW}}}}}'
        )
        t.confidence = 0.8 if max(gN, gS, gE, gW, oN, oS, oE, oW) > 50 else 0.3
        self.pub_thought.publish(t)


def main(args=None):
    rclpy.init(args=args)
    node = FloorLogoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
