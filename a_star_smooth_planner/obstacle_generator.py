#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose, Twist
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import Buffer, TransformListener, LookupException

import random
from random import *
import math

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
        # self.get_logger().info('Checking node functionality... Status: OK')
        pass
        
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











if __name__ == "__main__":
    # main()
    pass
    # rclpy.init()
    # node = obstacleController(10,10)
    # rclpy.spin(node)
    # rclpy.shutdown(node)