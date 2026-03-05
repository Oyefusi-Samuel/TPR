#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from rclpy.action import ActionClient
import sys

class AR4ArmGoal(Node):
    def __init__(self):
        super().__init__("ar4_arm_goal_client")
        self._action_client = ActionClient(self, MoveGroup, "move_action")
        self.get_logger().info("AR4 MoveIt2 Goal Client Ready")

    def send_pose_goal(self, x, y, z):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ar4_arm"
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.pipeline_id = "ompl"

        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = "ar4_base_link"
        pos_constraint.link_name = "ar4_link_6"
        
        s = SolidPrimitive()
        s.type = SolidPrimitive.BOX
        s.dimensions = [0.01, 0.01, 0.01]
        pos_constraint.constraint_region.primitives.append(s)
        
        p = Pose()
        p.position.x = x
        p.position.y = y
        p.position.z = z
        p.orientation.w = 1.0
        pos_constraint.constraint_region.primitive_poses.append(p)
        pos_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pos_constraint)
        goal_msg.request.goal_constraints.append(constraints)

        self.get_logger().info(f"Sending arm goal (rel to ar4_base_link): x={x}, y={y}, z={z}")
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("MoveGroup action server not available")
            return
        
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            rclpy.shutdown()
            return
        self.get_logger().info("Goal accepted, waiting for result...")
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info("Motion complete!")
        rclpy.shutdown()
        sys.exit(0)

def main():
    rclpy.init()
    node = AR4ArmGoal()
    x, y, z = 0.2, 0.0, 0.2
    if len(sys.argv) >= 4:
        x, y, z = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
    node.send_pose_goal(x, y, z)
    rclpy.spin(node)

if __name__ == "__main__":
    main()
