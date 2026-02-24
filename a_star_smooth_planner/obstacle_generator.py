#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose, Twist
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import Buffer, TransformListener, LookupException

import random
from random import *

class Obstacle:
    def __init__(self,coordinate:tuple ,twist_vector:Twist):
        self.coord = coordinate
        self.twist = twist_vector


class obstacleController(Node):
    def __init__(self,obstacle_duration,number_of_obstacles):
        super().__init__("ObstacleController")
        self.subscription = self.create_subscription(OccupancyGrid, 'map', self.topic_callback,10)
        self.get_logger().info("--- Obstacle Node Correctly Implemented!!! ---")
        self.obstacle_dur = obstacle_duration
        self.num_of_obstacles = number_of_obstacles
        self.map = None
        self.obstacle_dict : dict[tuple[int,int]:int] = []
        # Create a timer to print a message every 2 seconds
        self.timer = self.create_timer(2.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('Checking node functionality... Status: OK')
        
    def topic_callback(self,msg:OccupancyGrid):
        self.map = msg
        self.get_logger().info(f'recieved map') 

    #make a new obstacle on the occupancy grid, then give the new occupancy grid back to the topic 
    def obstacle_generator(self):
        if self.map:
            for _ in range(self.num_of_obstacles):
                radius = 5 
                twist = Twist(0,0,0) #this should have bounds from map message data. 
                coord = (Random.randint(0,self.map.info.height),Random.randint(0,self.map.info.width))
                new_obstacle = Obstacle(coord,twist)
                self.obstacle_dict[new_obstacle] = radius
        pass


    # remove obstacles from the occupancy grid.
    def remove_obstacle(self,obstacle:Obstacle):
        self.map[obstacle.coordinate] = 0
        self.obstacle_list.remove(obstacle)

    # dynamically move the obstacle in the grid at a fixed speed to replicate human motion. 
    def move_obstacle(self,obstacle:Obstacle):
        temp_obstacle = obstacle
        self.remove_obstacle(obstacle)
        # compute the move based on twist then do recreate and publish new map.
        #should happen on a set timerz
        pass

# from sensor_msgs.msg import LaserScan
# import math

# class FakeObstaclePublisher(Node):
#     def __init__(self):
#         super().__init__('fake_obstacle_node')
#         self.publisher_ = self.create_publisher(LaserScan, '/fake_scan', 10)
#         self.timer = self.create_timer(0.1, self.publish_fake_scan) # 10Hz

#     def publish_fake_scan(self):
#         scan = LaserScan()
#         scan.header.stamp = self.get_clock().now().to_msg()
#         scan.header.frame_id = 'map' # Attach to map so it stays put
        
#         # Define the scan parameters
#         scan.angle_min = 0.0
#         scan.angle_max = 2 * math.pi
#         scan.angle_increment = math.pi / 180 # 1 degree resolution
#         scan.time_increment = 0.0
#         scan.range_min = 0.1
#         scan.range_max = 20.0
        

#         # Fill with "infinity" so we don't clear real obstacles
#         scan.ranges = [float('inf')] * 360
        
#         # Place a 3-point wide "obstacle" at 2 meters away
#         # This will appear at the 0-degree mark relative to the map frame
#         obstacle_x = 4.0 #2 meters from map origin
#         obstacle_y = -3 #0.5 meters to the left
#         obstacle_radius = 0.2 #30cm radius

#         # Distance from origin to chair center
#         d = math.sqrt(obstacle_x**2 + obstacle_y**2)
#         # Angle from origin to chair center
#         alpha = math.atan2(obstacle_y, obstacle_x)
#         for i in range(360):
#             theta = i * scan.angle_increment
            
#             # Using the Law of Cosines to find the intersection of the ray and circle
#             # Solve for r: r^2 - 2rd*cos(theta-alpha) + (d^2 - R^2) = 0
#             # Using the quadratic formula:
#             b = -2 * d * math.cos(theta - alpha)
#             c = d**2 - obstacle_radius**2
#             discriminant = b**2 - 4*c

#             if discriminant >= 0:
#                 # We want the closest hit (the minus in the quadratic formula)
#                 r = (-b - math.sqrt(discriminant)) / 2
#                 if r > 0:
#                     scan.ranges[i] = r

#         self.publisher_.publish(scan)

# def main(args=None):
#     rclpy.init(args=args)
#     node = FakeObstaclePublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()










if __name__ == "__main__":
    # main()
    rclpy.init()
    node = obstacleController(10,10)
    rclpy.spin(node)
    rclpy.shutdown(node)