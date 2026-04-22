#!/usr/bin/env python3
"""mission_executive.py — Orchestrates the trash collection pipeline.

Bridges TrashPlanner -> A* Navigation -> WorkspaceExecutive -> Cleanup.

State machine:
  IDLE        — waiting for /planned_goal from TrashPlanner
  NAVIGATING  — robot en route; 2 Hz arrival check
  SETTLING    — arrived; waiting 1s for robot to stop + fresh detections
  PICKING     — arm executing pick-and-place via WorkspaceExecutive
  COOLDOWN    — pick done; waiting for fresh /detected_goals that confirms
                TrashGenerator removed the picked bottle before moving on.

Workspace model:
  The arm workspace is a cylinder offset forward from base_link.
  offset = half_robot_length + workspace_radius + 0.05
  Bottles are filtered by distance to the workspace center, not the robot
  center, using the robot's heading at the time of picking.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Twist

import tf2_geometry_msgs  # noqa: F401 — registers PoseStamped with tf2

from jackal_ar4_interfaces.action import WorkspacePickAndPlace


def _yaw_from_quat(q):
    """Extract yaw from a quaternion (x, y, z, w)."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class _S:
    IDLE = 'IDLE'
    NAVIGATING = 'NAVIGATING'
    ROTATING = 'ROTATING'
    SETTLING = 'SETTLING'
    FINE_APPROACH = 'FINE_APPROACH'
    PICKING = 'PICKING'
    COOLDOWN = 'COOLDOWN'


class MissionExecutive(Node):
    def __init__(self):
        super().__init__('mission_executive')

        self.declare_parameter('arrival_threshold', 0.20)
        self.declare_parameter('workspace_radius', 0.15)
        self.declare_parameter('workspace_offset', 0.36)
        self.declare_parameter('goal_change_threshold', 0.3)
        self.declare_parameter('pick_z_map_frame', 0.115)

        self._arrival = self.get_parameter('arrival_threshold').value
        self._ws_r = self.get_parameter('workspace_radius').value
        self._ws_off = self.get_parameter('workspace_offset').value
        self._goal_thr = self.get_parameter('goal_change_threshold').value
        self._pick_z = self.get_parameter('pick_z_map_frame').value

        self._tf = Buffer()
        TransformListener(self._tf, self)

        self.create_subscription(PoseStamped, '/planned_goal', self._goal_cb, 10)
        self.create_subscription(PoseArray, '/detected_goals', self._det_cb, 10)
        self._goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self._rem_pub = self.create_publisher(PoseArray, '/removed_goals', 10)
        self._vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self._ws = ActionClient(self, WorkspacePickAndPlace, 'workspace_pick_and_place')

        self._state = _S.IDLE
        self._nav_goal = None
        self._last_relay = None
        self._detected = []
        self._removed = []
        self._settle_time = None
        self._target_yaw = None
        self._nav_start_time = None
        self._retry_count = 0
        self._fine_start = None
        self._idle_since = None

        self.create_timer(0.5, self._tick)
        self.get_logger().info(
            f'Mission executive ready  ws_r={self._ws_r:.2f} '
            f'ws_off={self._ws_off:.2f}')

    # ── /planned_goal ──────────────────────────────────────────────────────

    def _goal_cb(self, msg):
        gx, gy = msg.pose.position.x, msg.pose.position.y

        # Drop goals while arm is busy or settling/rotating
        if self._state in (_S.PICKING, _S.COOLDOWN, _S.SETTLING, _S.ROTATING,
                          _S.FINE_APPROACH):
            return

        # Debounce: skip if same goal re-published (only while actively navigating)
        if self._state == _S.NAVIGATING and self._last_relay is not None:
            d = math.hypot(gx - self._last_relay[0], gy - self._last_relay[1])
            if d < self._goal_thr:
                return

        self._nav_goal = (gx, gy)
        self._last_relay = (gx, gy)
        self._goal_pub.publish(msg)
        self._state = _S.NAVIGATING
        self._nav_start_time = self.get_clock().now()
        self._retry_count = 0
        self.get_logger().info(f'NAV -> ({gx:.2f}, {gy:.2f})')

    # ── /detected_goals ────────────────────────────────────────────────────

    def _det_cb(self, msg):
        self._detected = [(p.position.x, p.position.y) for p in msg.poses]

        # COOLDOWN: wait for removed items to disappear from detection list
        if self._state == _S.COOLDOWN and self._removed:
            still_present = any(
                any(math.hypot(dx - rx, dy - ry) < 0.2
                    for rx, ry in self._removed)
                for dx, dy in self._detected
            )
            if not still_present:
                self._removed.clear()
                self._state = _S.IDLE
                self._last_relay = None
                self.get_logger().info(
                    f'COOLDOWN → IDLE ({len(self._detected)} goals remain)')

    # ── Periodic tick ──────────────────────────────────────────────────────

    def _tick(self):
        if self._state == _S.IDLE and self._idle_since is not None:
            elapsed = (self.get_clock().now() - self._idle_since).nanoseconds / 1e9
            if elapsed > 15.0:
                self._last_relay = None
                self._idle_since = None
                self.get_logger().info('IDLE timeout — accepting new goals')
        elif self._state == _S.NAVIGATING:
            self._tick_nav()
        elif self._state == _S.ROTATING:
            self._tick_rotate()
        elif self._state == _S.SETTLING:
            self._tick_settle()
        elif self._state == _S.FINE_APPROACH:
            self._tick_fine()

    def _tick_nav(self):
        if self._nav_start_time is not None:
            elapsed = (self.get_clock().now() - self._nav_start_time).nanoseconds / 1e9
            if elapsed > 25.0:
                self.get_logger().warn('Nav timeout (25s) — skipping goal')
                self._vel_pub.publish(Twist())
                self._state = _S.IDLE
                self._idle_since = self.get_clock().now()
                return
        robot = self._robot_xy()
        if robot is None or self._nav_goal is None:
            return
        d = math.hypot(robot[0] - self._nav_goal[0],
                       robot[1] - self._nav_goal[1])
        if d < self._arrival:
            self.get_logger().info(
                f'ARRIVED at ({robot[0]:.2f},{robot[1]:.2f}) '
                f'goal=({self._nav_goal[0]:.2f},{self._nav_goal[1]:.2f}) d={d:.2f}')
            self._vel_pub.publish(Twist())
            # Find closest detected trash to compute facing direction
            self._begin_rotate(robot)

    def _begin_rotate(self, robot_xy):
        """Compute target yaw toward nearest trash and enter ROTATING."""
        rx, ry = robot_xy
        closest, best_d = None, math.inf
        for tx, ty in self._detected:
            d = math.hypot(tx - rx, ty - ry)
            if d < best_d:
                best_d = d
                closest = (tx, ty)

        if closest is None or best_d < 0.01:
            # No trash or already on top — skip rotation
            self._state = _S.SETTLING
            self._settle_time = self.get_clock().now()
            return

        self._target_yaw = math.atan2(closest[1] - ry, closest[0] - rx)
        self._state = _S.ROTATING
        self.get_logger().info(
            f'ROTATING to face trash at ({closest[0]:.2f},{closest[1]:.2f}) '
            f'target_yaw={math.degrees(self._target_yaw):.0f}°')

    def _tick_rotate(self):
        """Spin in place until facing the target yaw."""
        pose = self._robot_pose()
        if pose is None:
            return
        _, _, yaw = pose
        err = math.atan2(math.sin(self._target_yaw - yaw),
                         math.cos(self._target_yaw - yaw))

        if abs(err) < 0.10:  # ~6° tolerance
            self._vel_pub.publish(Twist())
            self.get_logger().info(
                f'Facing target (err={math.degrees(err):.1f}°) — SETTLING')
            self._state = _S.SETTLING
            self._settle_time = self.get_clock().now()
            return

        cmd = Twist()
        cmd.angular.z = max(-0.8, min(0.8, 1.5 * err))
        self._vel_pub.publish(cmd)

    def _tick_settle(self):
        elapsed = (self.get_clock().now() - self._settle_time).nanoseconds / 1e9
        if elapsed < 1.0:
            self._vel_pub.publish(Twist())
            return
        self.get_logger().info('Settled — computing workspace picks with fresh TF')
        self._state = _S.PICKING
        self._do_pick()

    def _tick_fine(self):
        """Drive forward slowly to close the gap to workspace targets."""
        elapsed = (self.get_clock().now() - self._fine_start).nanoseconds / 1e9
        if elapsed > 3.0:
            self._vel_pub.publish(Twist())
            self.get_logger().info('Fine approach done — settling')
            self._state = _S.SETTLING
            self._settle_time = self.get_clock().now()
            return
        cmd = Twist()
        cmd.linear.x = 0.10
        self._vel_pub.publish(cmd)

    # ── Pick orchestration ─────────────────────────────────────────────────

    def _do_pick(self):
        """Filter bottles by the forward-offset workspace, then send to arm."""
        pose = self._robot_pose()
        if pose is None:
            self.get_logger().warn('Cannot get robot pose — IDLE')
            self._state = _S.IDLE
            self._last_relay = None
            return

        rx, ry, yaw = pose
        # Workspace center is offset forward from robot
        wcx = rx + self._ws_off * math.cos(yaw)
        wcy = ry + self._ws_off * math.sin(yaw)

        reachable = [(x, y) for x, y in self._detected
                     if math.hypot(x - wcx, y - wcy) < self._ws_r]

        if not reachable:
            if self._retry_count < 1 and self._detected:
                # Skip fine_approach if robot is already very close (< 0.3m from trash)
                closest_d = min(math.hypot(x - wcx, y - wcy) for x, y in self._detected)
                if closest_d > 0.3:
                    self._retry_count += 1
                    self._state = _S.FINE_APPROACH
                    self._fine_start = self.get_clock().now()
                    return
            self.get_logger().info(
                f'Nothing in workspace after retries — skipping goal')
            self._state = _S.IDLE
            self._idle_since = self.get_clock().now()
            return

        picks, wc = [], []
        for wx, wy in reachable:
            arm = self._to_arm(wx, wy, self._pick_z)
            if arm is not None:
                p = Pose()
                p.position = arm
                p.orientation.w = 1.0
                picks.append(p)
                wc.append((wx, wy))

        if not picks:
            self.get_logger().warn('TF failed for all — IDLE')
            self._state = _S.IDLE
            self._last_relay = None
            return

        self.get_logger().info(f'Sending {len(picks)} pick(s) to arm')
        if not self._ws.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server unavailable — IDLE')
            self._state = _S.IDLE
            self._last_relay = None
            return

        goal = WorkspacePickAndPlace.Goal()
        goal.pick_poses = picks
        goal.use_default_place = True
        self._ws.send_goal_async(goal).add_done_callback(
            lambda f: self._goal_resp(f, wc))

    def _goal_resp(self, future, wc):
        h = future.result()
        if not h.accepted:
            self.get_logger().warn('Goal rejected — IDLE')
            self._state = _S.IDLE
            self._last_relay = None
            return
        self.get_logger().info('Arm accepted')
        h.get_result_async().add_done_callback(lambda f: self._done(f, wc))

    def _done(self, future, wc):
        r = future.result().result
        self.get_logger().info(
            f'DONE picked={r.picked_count} '
            f'unreach={r.unreachable_count} fail={r.failed_count}')

        skip = set(r.unreachable_indices) | set(r.failed_indices)
        picked = [c for i, c in enumerate(wc) if i not in skip]

        if picked:
            msg = PoseArray()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            for wx, wy in picked:
                p = Pose()
                p.position.x = float(wx)
                p.position.y = float(wy)
                p.orientation.w = 1.0
                msg.poses.append(p)
            self._rem_pub.publish(msg)
            self._removed = list(picked)
            self.get_logger().info(f'Removed {len(picked)} — COOLDOWN')
            self._state = _S.COOLDOWN
        else:
            self._state = _S.IDLE
            self._idle_since = self.get_clock().now()
            self.get_logger().info('Nothing picked — skipping goal')

    # ── TF helpers ─────────────────────────────────────────────────────────

    def _robot_xy(self):
        try:
            t = self._tf.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.3))
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception:
            return None

    def _robot_pose(self):
        """Return (x, y, yaw) in map frame, or None."""
        try:
            t = self._tf.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.3))
            x = t.transform.translation.x
            y = t.transform.translation.y
            yaw = _yaw_from_quat(t.transform.rotation)
            return (x, y, yaw)
        except Exception:
            return None

    def _to_arm(self, wx, wy, wz):
        """Transform a map-frame point to ar4_base_link using tf2."""
        try:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = float(wx)
            ps.pose.position.y = float(wy)
            ps.pose.position.z = float(wz)
            ps.pose.orientation.w = 1.0
            out = self._tf.transform(
                ps, 'ar4_base_link',
                timeout=rclpy.duration.Duration(seconds=0.5))
            return out.pose.position
        except Exception as e:
            self.get_logger().warn(f'TF map->arm: {e}')
            return None


def main():
    rclpy.init()
    node = MissionExecutive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
