#!/usr/bin/env python3
"""goto_controller.py — Direct go-to-point controller using Gazebo ground truth.

No TF dependency. Gets robot pose from `gz model --pose`, computes heading +
distance to goal, publishes proportional cmd_vel.

Also publishes ground-truth pose to /gz/ground_truth_pose (PoseStamped) so
other nodes (mission_executive, gz_ground_truth_odom) can subscribe instead
of spawning their own subprocesses.

Control law:
  1. |heading_error| > turn_threshold  →  spin in place
  2. else                               →  drive forward + proportional steering
  3. distance < arrival_threshold       →  stop, clear goal

Subscribes:  /goal_pose             (PoseStamped)
Publishes:   /cmd_vel               (Twist)
             /gz/ground_truth_pose  (PoseStamped)
"""

import math
import re
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist


_BRACKET_RE = re.compile(
    r'\[\s*([-+]?\d+\.?\d*(?:[eE][-+]?\d+)?)'
    r'\s+([-+]?\d+\.?\d*(?:[eE][-+]?\d+)?)'
    r'\s+([-+]?\d+\.?\d*(?:[eE][-+]?\d+)?)\s*\]'
)


def _yaw_to_quat(yaw):
    """Yaw (rad) → (w, x, y, z) quaternion."""
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (cy, 0.0, 0.0, sy)


class GotoController(Node):

    def __init__(self):
        super().__init__('goto_controller')

        # Parameters
        self.declare_parameter('model_name', 'jackal_ar4')
        self.declare_parameter('arrival_threshold', 0.30)
        self.declare_parameter('turn_threshold', 0.25)
        self.declare_parameter('max_linear_vel', 0.25)
        self.declare_parameter('max_angular_vel', 0.8)
        self.declare_parameter('angular_kp', 1.5)
        self.declare_parameter('pose_rate', 5.0)

        self._model = self.get_parameter('model_name').value
        self._arrival_thr = self.get_parameter('arrival_threshold').value
        self._turn_thr = self.get_parameter('turn_threshold').value
        self._max_lin = self.get_parameter('max_linear_vel').value
        self._max_ang = self.get_parameter('max_angular_vel').value
        self._ang_kp = self.get_parameter('angular_kp').value
        self._pose_rate = self.get_parameter('pose_rate').value

        # State (thread-safe)
        self._lock = threading.Lock()
        self._pose = None   # (x, y, yaw)
        self._goal = None   # (gx, gy)

        # Pub / sub
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._gt_pub = self.create_publisher(PoseStamped, '/gz/ground_truth_pose', 10)
        self.create_subscription(PoseStamped, '/goal_pose', self._goal_cb, 10)

        # Single subprocess poller — all other nodes subscribe to /gz/ground_truth_pose
        self._pose_thread = threading.Thread(target=self._poll_pose, daemon=True)
        self._pose_thread.start()

        # Control loop
        self.create_timer(0.1, self._control_loop)  # 10 Hz

        self.get_logger().info(
            f'goto_controller: polling gz model -m {self._model} @ '
            f'{self._pose_rate} Hz')

    # ── callbacks ────────────────────────────────────────────────────────────

    def _goal_cb(self, msg: PoseStamped):
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        with self._lock:
            self._goal = (gx, gy)
        self.get_logger().info(f'Goal received: ({gx:.2f}, {gy:.2f})')

    # ── ground-truth pose polling (single source for entire system) ──────────

    def _poll_pose(self):
        cmd = ['gz', 'model', '-m', self._model, '--pose']
        period = 1.0 / max(self._pose_rate, 1.0)
        while rclpy.ok():
            try:
                r = subprocess.run(
                    cmd, capture_output=True, text=True, timeout=2.0)
                if r.returncode == 0:
                    parsed = self._parse_full(r.stdout)
                    if parsed:
                        x, y, z, yaw = parsed
                        with self._lock:
                            self._pose = (x, y, yaw)
                        # Publish for other nodes
                        self._publish_gt(x, y, z, yaw)
            except subprocess.TimeoutExpired:
                pass
            except Exception as e:
                self.get_logger().warn(f'pose poll: {e}', once=True)
            time.sleep(period)

    @staticmethod
    def _parse_full(text):
        """Parse gz model --pose → (x, y, z, yaw)."""
        matches = _BRACKET_RE.findall(text)
        if len(matches) >= 2:
            x = float(matches[0][0])
            y = float(matches[0][1])
            z = float(matches[0][2])
            yaw = float(matches[1][2])
            return (x, y, z, yaw)
        return None

    def _publish_gt(self, x, y, z, yaw):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        w, qx, qy, qz = _yaw_to_quat(yaw)
        msg.pose.orientation.w = w
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        self._gt_pub.publish(msg)

    # ── control loop ─────────────────────────────────────────────────────────

    def _control_loop(self):
        with self._lock:
            pose = self._pose
            goal = self._goal

        if pose is None or goal is None:
            return

        x, y, yaw = pose
        gx, gy = goal

        dx = gx - x
        dy = gy - y
        distance = math.hypot(dx, dy)

        # Arrived
        if distance < self._arrival_thr:
            self.get_logger().info(
                f'Arrived at ({gx:.2f}, {gy:.2f})  [dist={distance:.2f}]')
            with self._lock:
                self._goal = None
            self._publish_cmd(0.0, 0.0)
            return

        # Heading error (wraps correctly to [-pi, pi])
        desired = math.atan2(dy, dx)
        err = math.atan2(math.sin(desired - yaw), math.cos(desired - yaw))

        cmd = Twist()
        ang = max(-self._max_ang, min(self._max_ang, self._ang_kp * err))

        if abs(err) > self._turn_thr:
            # Spin in place — don't drive forward while misaligned
            cmd.angular.z = ang
        else:
            cmd.linear.x = self._max_lin
            cmd.angular.z = ang

        self._cmd_pub.publish(cmd)

    def _publish_cmd(self, lin: float, ang: float):
        cmd = Twist()
        cmd.linear.x = lin
        cmd.angular.z = ang
        self._cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = GotoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
