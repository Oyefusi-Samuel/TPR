#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
import sys

class AR4PickGoal(Node):
    def __init__(self):
        super().__init__('ar4_pick_goal')

        self._action_client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )

        self.get_logger().info('AR4 arm goal client ready')

    def send_pick_goal(self, x, y, z):
        """Send pick goal: (x, y, z) position for end-effector"""
        goal_msg = MoveGroup.Goal()

        # Motion plan request
        goal_msg.request = MotionPlanRequest()
        goal_msg.request.group_name = 'ar4_arm'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0

        # Goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'base_link'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = z

        # Default orientation (pointing down for picking)
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 1.0  # Pointing downward
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 0.0

        # Position constraint
        position_constraint = PositionConstraint()
        position_constraint.header = goal_pose.header
        position_constraint.link_name = 'ar4_link6'  # End-effector link
        position_constraint.target_point_offset.x = 0.0
        position_constraint.target_point_offset.y = 0.0
        position_constraint.target_point_offset.z = 0.0

        constraints = Constraints()
        constraints.position_constraints.append(position_constraint)
        goal_msg.request.goal_constraints.append(constraints)

        self.get_logger().info(f'Sending pick goal: x={x}, y={y}, z={z}')

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MoveGroup action server not available after waiting 5 seconds')
            return

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Arm goal rejected')
            return

        self.get_logger().info('Arm goal accepted, planning...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code.val == 1:  # SUCCESS
            self.get_logger().info('Arm reached pick position!')
        else:
            self.get_logger().info(f'Motion planning failed: {result.error_code.val}')
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = AR4PickGoal()

    # Get x, y, z from command line or use default
    x = 0.3
    y = 0.2
    z = 0.1
    
    if len(sys.argv) >= 4:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])

    node.send_pick_goal(x, y, z)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    main()
