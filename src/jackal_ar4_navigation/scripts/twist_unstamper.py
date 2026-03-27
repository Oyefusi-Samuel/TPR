#!/usr/bin/env python3
"""
twist_unstamper.py

Converts geometry_msgs/TwistStamped  →  geometry_msgs/Twist

The a_star_smooth_planner's pure_pursuit node publishes TwistStamped on
/cmd_vel_stamped.  The Gazebo diff_drive plugin (and teleop_twist_keyboard)
expect plain Twist on /cmd_vel.  This node strips the header.

Subscribes:  /cmd_vel_stamped  (geometry_msgs/msg/TwistStamped)
Publishes:   /cmd_vel          (geometry_msgs/msg/Twist)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class TwistUnstamper(Node):
    def __init__(self):
        super().__init__('twist_unstamper')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(
            TwistStamped, '/cmd_vel_stamped', self.callback, 10
        )
        self.get_logger().info('twist_unstamper ready: /cmd_vel_stamped → /cmd_vel')

    def callback(self, msg: TwistStamped):
        out = Twist()
        out.linear  = msg.twist.linear
        out.angular = msg.twist.angular
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TwistUnstamper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
