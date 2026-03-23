#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose

class GoalEmitter(Node):
    def __init__(self):
        super().__init__('goal_emitter')
        # We use PoseArray to send multiple goals in one "burst"
        self.publisher_ = self.create_publisher(PoseArray, 'detected_goals', 10)
        self.timer = self.create_timer(2.0, self.publish_goals)

    def publish_goals(self):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map' # Matches your ROS 2 map frame

        # Create dummy goals (replace with your actual goal detection)
        positions = [(2.5, 2), (2.8, 2.4), (4.0, 4.0),[1.2,2],[1.0,2], [-2,1],[2,1]] 
        
        for x, y in positions:
            p = Pose()
            p.position.x = float(x)
            p.position.y = float(y)
            p.orientation.w = 1.0 # Neutral orientation
            msg.poses.append(p)

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published {len(msg.poses)} goals')

    
        

def main():
    rclpy.init()
    node = GoalEmitter()
    rclpy.spin(node)
    rclpy.shutdown(node)

if __name__ == "__main__":
    main()