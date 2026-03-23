#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose, Twist
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import Buffer, TransformListener, LookupException
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Odometry

import random
from random import *
import math
import numpy as np

class TrashPlanner(Node):
    def __init__(self, node_name, *, context = None, cli_args = None, namespace = None, use_global_arguments = True, enable_rosout = True, start_parameter_services = True, parameter_overrides = None, allow_undeclared_parameters = False, automatically_declare_parameters_from_overrides = False, enable_logger_service = False):
        super().__init__(node_name, context=context, cli_args=cli_args, namespace=namespace, use_global_arguments=use_global_arguments, enable_rosout=enable_rosout, start_parameter_services=start_parameter_services, parameter_overrides=parameter_overrides, allow_undeclared_parameters=allow_undeclared_parameters, automatically_declare_parameters_from_overrides=automatically_declare_parameters_from_overrides, enable_logger_service=enable_logger_service)
        self.arm_workspace_radius = 0.4 #40 cm #TODO fill in with actual radius here
        # Subscribe to the costmap topic from your rqt list
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/costmap', 
            self.costmap_callback,
            10)
        self.goal_sub = self.create_subscription(
            PoseArray, 'detected_goals', self.process_goals, 10)
        
        self.goal_pub = self.create_publisher(PoseStamped, '/planned_goal', 10)
        
        # Setup TF2 Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Initialize pose to a safe default
        
        self.grid_data = None
        self.map_resolution = 0.0
        self.map_origin = None
        self.robotPose = None

    def update_robot_pose(self):
        try:
            # Get the latest transform (Time(0))
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'map', 
                'base_link', 
                now,
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            self.robotPose = [
                trans.transform.translation.x,
                trans.transform.translation.y
            ]
            return True
        except Exception as e:
            self.get_logger().warning(f'Could not look up robot pose: {e}')
            return False

    def costmap_callback(self, msg):
        # Store metadata for coordinate conversion
        self.map_resolution = msg.info.resolution
        self.map_origin = msg.info.origin.position
        
        # Convert 1D data to 2D numpy array for easier indexing
        # ROS 2 data is row-major
        self.grid_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))

    def process_goals(self,msg):
        self.update_robot_pose()
        if not msg.poses or self.robotPose is None or self.grid_data is None:
            self.get_logger().info(f'missing an element for calculation!, \n poses are {msg.poses} \n robotPose is {self.robotPose} \n gridData is {self.grid_data} ')
            return
        coord_list = []
        for p in msg.poses:
            coords = (p.position.x,p.position.y)
            coord_list.append(coords)
        
        
        closet_trash = self.find_closest(coord_list,self.robotPose)
        if closet_trash:
            goal = self.determine_goal(closet_trash,coord_list)
            # Create the PoseStamped message
            goal_msg = PoseStamped()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = 'map'  # Matches your TF frame
            
            goal_msg.pose.position.x = float(goal[0])
            goal_msg.pose.position.y = float(goal[1])
            goal_msg.pose.position.z = 0.0
            
            # No rotation needed for the centroid, so we set a "neutral" quaternion
            goal_msg.pose.orientation.w = 1.0 

            # Publish it!
            self.goal_pub.publish(goal_msg)
            self.get_logger().info(f'Published Goal: {goal}')

    def world_to_grid(self, world_x, world_y):
        """Converts real-world meters to grid index"""
        if self.map_origin is None: return None
        
        grid_x = int((world_x - self.map_origin.x) / self.map_resolution)
        grid_y = int((world_y - self.map_origin.y) / self.map_resolution)
        return (grid_x, grid_y)

    def find_closest(self,trashList, robotPose):
        # self.get_logger().info(f'NO GRID DATA FOR COSTMAP CALCULATIONS! Grid data is {self.grid_data}')
        closest_dist = math.inf
        closest_trash = None
        for coords in trashList:
            gridx , gridy = self.world_to_grid(coords[0],coords[1])

            ############################
            # Need some quick algorithm which can 
            originGrid = self.world_to_grid(robotPose[0],robotPose[1])
            endGrid = (gridx,gridy)
            obstacles_in_way = self.is_path_clear(self.grid_data,originGrid,endGrid)      
            ############################

            Euclideandist = math.sqrt((coords[0] - robotPose[0]) **2 + (coords[1] - robotPose[1] )** 2 )

            match obstacles_in_way: #prioritizes anypath with no obstacles in way. and adds a 3x penalty to all paths through walls. 
                case False:
                    if Euclideandist < closest_dist and self.grid_data[gridy][gridx] <= 20:
                        closest_dist = Euclideandist
                        closest_trash = coords

                case True:
                    if Euclideandist*3.0 < closest_dist and self.grid_data[gridy][gridx] <= 20:
                        closest_dist = Euclideandist*3.0
                        closest_trash = coords

        return closest_trash
    
    def determine_goal(self,closest_trash,trashList):
        nearby_trash = []
        for coords in trashList:
            x, y = coords
            dist = math.sqrt((x - closest_trash[0])**2 + (y - closest_trash[1])**2)
            
            # Check if trash is within reach
            if dist < self.arm_workspace_radius * 2: 
                # Convert world coordinates (meters) to grid indices (integers)
                start_grid = self.world_to_grid(closest_trash[0], closest_trash[1])
                end_grid = self.world_to_grid(x, y)
                
                # Ensure conversion worked and then check the path
                if start_grid and end_grid:
                    if self.is_path_clear(self.grid_data, start_grid, end_grid):
                        nearby_trash.append(coords)
        
        
        if len(nearby_trash) == 1: 
            x,y = nearby_trash[0]
            rx = self.robotPose[0]
            ry = self.robotPose[1]
            
            dx = x - rx
            dy = y - ry
            currLen = math.sqrt(dx**2 + dy**2)
            if currLen <= self.arm_workspace_radius/2.0:
                return (self.robotPose[0],self.robotPose[1]) # Or handle as an error
            
            newLen = currLen - self.arm_workspace_radius/2.0
            ratio = newLen / currLen

            newx = self.robotPose[0] + (dx*ratio)
            newy = self.robotPose[1] + (dy*ratio)
            gx,gy = self.world_to_grid(newx,newy)

            if self.grid_data[gy][gx] > 20: #checks if this would go into the wall!!              
                newLen = currLen + self.arm_workspace_radius/2.0
                ratio = newLen / currLen

                newx = self.robotPose[0] + (dx*ratio)
                newy = self.robotPose[1] + (dy*ratio)
            return (newx,newy)
        #for multiple pieces of trash
        sumx = 0
        sumy = 0
        for x,y in nearby_trash:
            sumx += x
            sumy += y
        centroid = ((sumx)/len(nearby_trash),(sumy)/len(nearby_trash)) #uses geometric average (centroid) of the object to be equadistant to in the middle of the points
        gridx, gridy = self.world_to_grid(centroid[0],centroid[1])
        if self.grid_data[gridy][gridx] >= 100:
            return closest_trash
        else:  
            return centroid
            
    def get_line(self, start, end):
        """
        Returns a list of grid coordinates (x, y) from start to end 
        using Bresenham's Line Algorithm.
        """
        x0, y0 = start
        x1, y1 = end
        
        cells = []
        
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        
        while True:
            cells.append((x0, y0))
            
            if x0 == x1 and y0 == y1:
                break
                
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy
                
        return cells
    
    def is_path_clear(self, grid, start, end, lethal_value=254):
        """
        Checks if the straight line between two points hits a lethal obstacle.
        'grid' is your 2D numpy array or list-of-lists.
        """
        line_path = self.get_line(start, end)
        
        for x, y in line_path:
            # Boundary check for safety
            if 0 <= x < len(grid) and 0 <= y < len(grid[0]):
                if grid[y][x] >= lethal_value:
                    return False  # Obstacle detected
            else:
                return False  # Path goes off-grid
                
        return True # Path is clear
    
    

def main():
    rclpy.init()
    node = TrashPlanner('planner')
    rclpy.spin(node)
    rclpy.shutdown(node)

if __name__ == "__main__":
    main()