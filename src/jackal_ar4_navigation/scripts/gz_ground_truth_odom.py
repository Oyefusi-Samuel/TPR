#!/usr/bin/env python3
"""gz_ground_truth_odom.py — Ground-truth map→odom from Gazebo plugin.

Subscribes to /ground_truth_odom (nav_msgs/Odometry) published by the
OdometryPublisher Gazebo system plugin (bridged via ros_gz_bridge).
No subprocess calls — zero performance impact.

Computes: map→odom = ground_truth(map→base_link) × inv(odom→base_link)
Broadcasts: map→odom TF at the rate Gazebo publishes (10 Hz).
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener


# ── quaternion helpers (w, x, y, z) ─────────────────────────────────────────

def _qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return (
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    )


def _qconj(q):
    return (q[0], -q[1], -q[2], -q[3])


def _qrot(q, v):
    r = _qmul(_qmul(q, (0.0, v[0], v[1], v[2])), _qconj(q))
    return (r[1], r[2], r[3])


class GzGroundTruthLocalization(Node):
    def __init__(self):
        super().__init__('gz_ground_truth_localization')

        self._tf_buf = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)
        self._tf_bc = TransformBroadcaster(self)

        self.create_subscription(
            Odometry, '/ground_truth_odom', self._gt_cb, 10)

        self.get_logger().info(
            'Ground-truth localization: subscribing to /ground_truth_odom '
            '(Gazebo OdometryPublisher plugin)')

    def _gt_cb(self, msg: Odometry):
        """Receive ground-truth pose, compute and broadcast map→odom."""
        q_gt = (
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
        )
        t_gt = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        )

        # Look up odom → base_link (from DiffDrive, 50 Hz)
        try:
            tf_od = self._tf_buf.lookup_transform(
                'odom', 'base_link', rclpy.time.Time())
        except Exception:
            return

        q_od = (
            tf_od.transform.rotation.w,
            tf_od.transform.rotation.x,
            tf_od.transform.rotation.y,
            tf_od.transform.rotation.z,
        )
        t_od = (
            tf_od.transform.translation.x,
            tf_od.transform.translation.y,
            tf_od.transform.translation.z,
        )

        # map→odom = map→base_link × inv(odom→base_link)
        q_od_inv = _qconj(q_od)
        t_od_inv = _qrot(q_od_inv, (-t_od[0], -t_od[1], -t_od[2]))

        q_map_odom = _qmul(q_gt, q_od_inv)
        t_rot = _qrot(q_gt, t_od_inv)
        t_map_odom = (
            t_gt[0] + t_rot[0],
            t_gt[1] + t_rot[1],
            t_gt[2] + t_rot[2],
        )

        ts = TransformStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = 'map'
        ts.child_frame_id = 'odom'
        ts.transform.translation.x = t_map_odom[0]
        ts.transform.translation.y = t_map_odom[1]
        ts.transform.translation.z = t_map_odom[2]
        ts.transform.rotation.w = q_map_odom[0]
        ts.transform.rotation.x = q_map_odom[1]
        ts.transform.rotation.y = q_map_odom[2]
        ts.transform.rotation.z = q_map_odom[3]
        self._tf_bc.sendTransform(ts)


def main():
    rclpy.init()
    node = GzGroundTruthLocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
