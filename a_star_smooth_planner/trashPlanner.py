import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose, Twist
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import Buffer, TransformListener, LookupException

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
        self.grid_data = None
        self.map_resolution = 0.0
        self.map_origin = None

    def costmap_callback(self, msg):
        # Store metadata for coordinate conversion
        self.map_resolution = msg.info.resolution
        self.map_origin = msg.info.origin.position
        
        # Convert 1D data to 2D numpy array for easier indexing
        # ROS 2 data is row-major
        self.grid_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))

    def find_closest(self,trashDict, robotPose):
        closest_dist = math.inf
        closest_trash = None
        for key, _ in trashDict:
            Euclideandist = math.sqrt((key[0] - robotPose[0]) **2 + (key[1] - robotPose[1] )** 2 )
            if Euclideandist < closest_dist:
                closest_dist = Euclideandist
                closest_trash = key

        return closest_trash
    
    def determine_goal(self,closest_trash,trashDict):
        nearby_trash = []
        for key, _ in trashDict:
            x,y = key
            Euclideandist = math.sqrt((x-closest_trash[0])**2 + (y-closest_trash[1])**2)
            if Euclideandist < self.arm_workspace_radius*2: #use the diameter to allow the robot to be just able to grab both without moving
                if self.is_path_clear('occupancy grid conatining values for obstacles', (closest_trash[0], closest_trash[1]), (x,y)): #need to fill in the actual occupancy grid
                    nearby_trash.append(key)
        
        sumx = 0
        sumy = 0
        ################################################
        #FUTURE UPDATES: Ensure wall geometry is taken into account. 
        # 2 pieces of trash on either side of the wall shouldn't be matched. 
        #################################################
        for x,y in nearby_trash:
            sumx += x
            sumy += y
        centroid = ((sumx)/len(nearby_trash),(sumy)/len(nearby_trash)) #uses geometric average (centroid) of the object to be equadistant to in the middle of the points
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
                if grid[x][y] >= lethal_value:
                    return False  # Obstacle detected
            else:
                return False  # Path goes off-grid
                
        return True # Path is clear