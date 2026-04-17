"""Top-level state machine.

Phases:
  INIT            - settle, bring sensors online
  EXPLORE         - wall-follow until first tag detected (no logging)
  APPROACH_TAG    - slow down, center on tag, wait for vote commit
  ACT_LEFT/RIGHT  - rotate ±90°
  ACT_UTURN       - rotate 180°
  GREEN_FOLLOW    - per-tile edge-sample → exit toward green
  ORANGE_FOLLOW   - same but orange
  SEEK_TAG_5      - bounded search when green arc ends before Tag 5 seen
  REACH_STOP      - solid-red tile or STOP pose → final halt

All scoreable events are surfaced on /tag_event (from apriltag_handler) and
/tile_event (from tile_tracker); the state machine mirrors them into
/scorecard for the logger. The state machine itself only owns /cmd_vel.
"""
from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Dict, Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32MultiArray, String

from artpark_msgs.msg import EdgeSample, TagEvent, TileEvent, Thought

from .edge_sampler import decide_exit, EDGES, OPPOSITE


class Phase(Enum):
    INIT = auto()
    EXPLORE = auto()
    APPROACH_TAG = auto()
    ACT_LEFT = auto()
    ACT_RIGHT = auto()
    ACT_UTURN = auto()
    GREEN_FOLLOW = auto()
    ORANGE_FOLLOW = auto()
    SEEK_TAG_5 = auto()
    REACH_STOP = auto()


@dataclass
class SMState:
    phase: Phase = Phase.INIT
    seq: int = 0
    entry_edge: Optional[str] = None     # for the CURRENT tile
    tile_rc: Optional[tuple[int, int]] = None
    obstacle_near: bool = False
    approach_mode: bool = False
    last_edge_sample: Optional[EdgeSample] = None
    tags_logged: set[int] = field(default_factory=set)
    committed_action: Optional[str] = None
    rotation_target_yaw: Optional[float] = None
    current_yaw: float = 0.0


class StateMachine(Node):
    def __init__(self) -> None:
        super().__init__('state_machine')

        # -------- parameters --------
        self.declare_parameter('v_explore',  0.20)
        self.declare_parameter('v_approach', 0.08)
        self.declare_parameter('v_follow',   0.15)
        self.declare_parameter('w_turn',     0.8)
        self.declare_parameter('idle_watchdog_s', 4.0)

        self.v_exp   = float(self.get_parameter('v_explore').value)
        self.v_app   = float(self.get_parameter('v_approach').value)
        self.v_fol   = float(self.get_parameter('v_follow').value)
        self.w_turn  = float(self.get_parameter('w_turn').value)
        self.idle_s  = float(self.get_parameter('idle_watchdog_s').value)

        # -------- state --------
        self.s = SMState()
        self._last_cmd_time = time.monotonic()

        # -------- I/O --------
        qos_rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        qos_be  = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(TagEvent,   '/tag_event',   self._on_tag,  qos_rel)
        self.create_subscription(EdgeSample, '/edge_sample', self._on_edge, qos_rel)
        self.create_subscription(TileEvent,  '/tile_event',  self._on_tile, qos_rel)
        self.create_subscription(Bool,       '/obstacle_near', self._on_obs, qos_rel)
        self.create_subscription(Bool,       '/approach_mode', self._on_app, qos_rel)
        self.create_subscription(Float32MultiArray, '/obstacle_octants',
                                  self._on_octants, qos_rel)
        self.create_subscription(Odometry,   '/odom', self._on_odom, qos_be)

        self.pub_cmd     = self.create_publisher(Twist, '/cmd_vel', qos_rel)
        self.pub_thought = self.create_publisher(Thought, '/thought', qos_rel)
        self.pub_score   = self.create_publisher(String, '/scorecard_event', qos_rel)

        # 20 Hz tick drives motion
        self.timer = self.create_timer(0.05, self._tick)

        self._octants = [float('inf')] * 8
        self.get_logger().info('state_machine up, entering INIT')

    # ================================================================
    # callbacks
    def _on_tag(self, msg: TagEvent) -> None:
        if msg.tag_id in self.s.tags_logged and not msg.first_sighting:
            return
        self.s.tags_logged.add(msg.tag_id)

        # Emit scorecard row (logger listens)
        row = {
            'type': 'TAG_LOG',
            'tag_id': msg.tag_id,
            'label': msg.logical_label,
            'decision': msg.decision,
            'distance': msg.distance,
            'bearing': msg.bearing,
        }
        self.pub_score.publish(String(data=json.dumps(row)))

        # Set next phase based on hardcoded action.
        action = msg.decision
        self._think(f'TagEvent committed: label={msg.logical_label}, action={action}',
                    rule='state_machine.on_tag_commit',
                    confidence=1.0)

        if action == 'LEFT':
            self._start_rotation(+math.pi / 2, Phase.ACT_LEFT)
        elif action == 'RIGHT':
            self._start_rotation(-math.pi / 2, Phase.ACT_RIGHT)
        elif action == 'U_TURN':
            self._start_rotation(math.pi, Phase.ACT_UTURN)
        elif action == 'GREEN':
            self.s.phase = Phase.GREEN_FOLLOW
            self.s.entry_edge = None  # fresh on mode entry
        elif action == 'ORANGE':
            self.s.phase = Phase.ORANGE_FOLLOW
            self.s.entry_edge = None

    def _on_edge(self, msg: EdgeSample) -> None:
        self.s.last_edge_sample = msg

    def _on_tile(self, msg: TileEvent) -> None:
        self.s.tile_rc = (msg.tile_row, msg.tile_col)
        self.s.entry_edge = msg.entry_edge if msg.entry_edge else self.s.entry_edge

    def _on_obs(self, msg: Bool) -> None:
        self.s.obstacle_near = msg.data

    def _on_app(self, msg: Bool) -> None:
        self.s.approach_mode = msg.data
        if self.s.approach_mode and self.s.phase == Phase.EXPLORE:
            self.s.phase = Phase.APPROACH_TAG

    def _on_octants(self, msg: Float32MultiArray) -> None:
        self._octants = list(msg.data)

    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        self.s.current_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    # ================================================================
    def _tick(self) -> None:
        ph = self.s.phase
        cmd = Twist()

        if ph == Phase.INIT:
            # Wait 1 second for TF/odom/sensors to settle, then explore.
            if time.monotonic() - self._last_cmd_time > 1.0:
                self.s.phase = Phase.EXPLORE
                self._think('INIT → EXPLORE (settle complete)',
                            rule='state_machine.init_timeout',
                            confidence=1.0)
            self._publish(cmd)
            return

        if ph == Phase.EXPLORE:
            # Simple wall-follow-ish: drive forward unless obstacle, else rotate right.
            if self.s.obstacle_near:
                cmd.angular.z = -self.w_turn * 0.8
                self._think('EXPLORE: obstacle ahead, rotating right',
                            rule='state_machine.explore_avoid', confidence=0.9)
            else:
                cmd.linear.x = self.v_exp
            self._publish(cmd)
            return

        if ph == Phase.APPROACH_TAG:
            if self.s.obstacle_near:
                # Final 0.2 m before tag — stop, let the vote fire.
                self._publish(Twist())
                return
            cmd.linear.x = self.v_app
            self._publish(cmd)
            return

        if ph in (Phase.ACT_LEFT, Phase.ACT_RIGHT, Phase.ACT_UTURN):
            self._run_rotation(cmd)
            return

        if ph == Phase.GREEN_FOLLOW:
            self._follow_color(cmd, color='green')
            return

        if ph == Phase.ORANGE_FOLLOW:
            self._follow_color(cmd, color='orange')
            return

        if ph == Phase.SEEK_TAG_5:
            if self.s.obstacle_near:
                cmd.angular.z = self.w_turn * 0.7
            else:
                cmd.linear.x = self.v_fol * 0.7
            self._publish(cmd)
            return

        if ph == Phase.REACH_STOP:
            self._publish(Twist())
            return

    # ================================================================
    def _start_rotation(self, delta_yaw: float, new_phase: Phase) -> None:
        target = self.s.current_yaw + delta_yaw
        # wrap
        target = math.atan2(math.sin(target), math.cos(target))
        self.s.rotation_target_yaw = target
        self.s.phase = new_phase
        self._think(f'starting rotation Δ={math.degrees(delta_yaw):.0f}° → target yaw {math.degrees(target):.0f}°',
                    rule='state_machine.rotation_start', confidence=1.0)

    def _run_rotation(self, cmd: Twist) -> None:
        if self.s.rotation_target_yaw is None:
            # degenerate; fall back to follow/explore
            self.s.phase = Phase.GREEN_FOLLOW if 3 in self.s.tags_logged else Phase.EXPLORE
            return
        diff = math.atan2(
            math.sin(self.s.rotation_target_yaw - self.s.current_yaw),
            math.cos(self.s.rotation_target_yaw - self.s.current_yaw),
        )
        if abs(diff) < math.radians(5):
            # done rotating; pick next phase
            if self.s.phase == Phase.ACT_UTURN:
                # after a U-turn we stay in GREEN_FOLLOW
                self.s.phase = Phase.GREEN_FOLLOW
            else:
                # after LEFT/RIGHT we resume EXPLORE until next tag
                self.s.phase = Phase.EXPLORE
            self.s.rotation_target_yaw = None
            self.s.entry_edge = None
            self._think(f'rotation complete → phase {self.s.phase.name}',
                        rule='state_machine.rotation_done', confidence=1.0)
            return
        cmd.angular.z = self.w_turn if diff > 0 else -self.w_turn
        self._publish(cmd)

    # ================================================================
    def _follow_color(self, cmd: Twist, color: str) -> None:
        es = self.s.last_edge_sample
        if es is None:
            cmd.linear.x = self.v_fol * 0.5
            self._publish(cmd)
            return

        if color == 'green':
            counts = {'N': es.green_n, 'S': es.green_s, 'E': es.green_e, 'W': es.green_w}
        else:
            counts = {'N': es.orange_n, 'S': es.orange_s, 'E': es.orange_e, 'W': es.orange_w}

        walls = self._walls_from_octants()
        decision = decide_exit(counts, entry_edge=self.s.entry_edge,
                               wall_blocked=walls,
                               min_pixels=50, near_tie_ratio=0.15)

        self._think(
            f'{color}_FOLLOW: counts={counts} entry={self.s.entry_edge} '
            f'walls={walls} → {decision.chosen or "NONE"} ({decision.reason})',
            rule=f'state_machine.{color}_follow.{decision.reason}',
            confidence=decision.confidence,
        )

        if not decision.chosen:
            if color == 'green':
                self.s.phase = Phase.SEEK_TAG_5
                self._think('green arc exhausted → SEEK_TAG_5',
                            rule='state_machine.green_exhausted', confidence=0.7)
            else:
                # orange exhausted — maybe we're at STOP already; check LiDAR
                if self.s.obstacle_near:
                    self.s.phase = Phase.REACH_STOP
                    self.pub_score.publish(String(data=json.dumps({'type': 'STOP_REACHED'})))
                    self._think('orange exhausted + obstacle → REACH_STOP',
                                rule='state_machine.orange_exhausted', confidence=0.8)
            self._publish(Twist())
            return

        # Drive toward chosen edge: align heading with edge, then drive forward.
        target_heading = self._edge_to_yaw(decision.chosen)
        diff = math.atan2(
            math.sin(target_heading - self.s.current_yaw),
            math.cos(target_heading - self.s.current_yaw),
        )
        if abs(diff) > math.radians(15):
            cmd.angular.z = self.w_turn * (1 if diff > 0 else -1) * 0.8
        else:
            cmd.linear.x = self.v_fol
            cmd.angular.z = 0.6 * diff   # P-correction while driving

        self._publish(cmd)

    def _walls_from_octants(self) -> Dict[str, bool]:
        # Octant 0 = forward (+x), CCW. Map to compass directions of the robot body.
        # Near 0.45 m means a wall is effectively at the tile edge ahead.
        o = self._octants
        def blocked(idx: int) -> bool:
            v = o[idx] if idx < len(o) else float('inf')
            return 0 < v < 0.45
        return {
            'N': blocked(0),  # forward
            'W': blocked(2),  # left
            'S': blocked(4),  # back
            'E': blocked(6),  # right
        }

    @staticmethod
    def _edge_to_yaw(edge: str) -> float:
        # With the robot spawned facing +x (yaw=0), N=forward=+x=yaw 0.
        # This is a ROBOT-RELATIVE mapping; the tile N/S/E/W here is robot-
        # relative since the floor cam is body-fixed.
        return {'N': 0.0, 'E': -math.pi / 2, 'S': math.pi, 'W': math.pi / 2}[edge]

    # ================================================================
    def _publish(self, cmd: Twist) -> None:
        self.pub_cmd.publish(cmd)
        # idle watchdog: if we've published zero-twist for too long, nudge
        if abs(cmd.linear.x) + abs(cmd.angular.z) < 1e-3:
            if time.monotonic() - self._last_cmd_time > self.idle_s:
                nudge = Twist()
                nudge.angular.z = 0.3
                self.pub_cmd.publish(nudge)
                self._last_cmd_time = time.monotonic()
                self._think('idle watchdog nudge — avoiding -3 penalty',
                            rule='state_machine.idle_watchdog', confidence=1.0)
        else:
            self._last_cmd_time = time.monotonic()

    def _think(self, hypothesis: str, rule: str, confidence: float,
               action: str = '', alternatives: str = '', extras: dict | None = None) -> None:
        self.s.seq += 1
        t = Thought()
        t.stamp = self.get_clock().now().to_msg()
        t.seq = self.s.seq
        t.phase = self.s.phase.name
        t.hypothesis = hypothesis
        t.action_chosen = action or 'cmd_vel'
        t.rule_applied = rule
        t.alt_considered = alternatives
        t.extras_json = json.dumps(extras or {})
        t.confidence = float(confidence)
        self.pub_thought.publish(t)


def main(args=None):
    rclpy.init(args=args)
    node = StateMachine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
