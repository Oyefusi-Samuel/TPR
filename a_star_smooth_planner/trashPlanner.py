import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose, Twist
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import Buffer, TransformListener, LookupException

import random
from random import *
import math

class TrashPlanner(Node):
    def __init__(self, node_name, *, context = None, cli_args = None, namespace = None, use_global_arguments = True, enable_rosout = True, start_parameter_services = True, parameter_overrides = None, allow_undeclared_parameters = False, automatically_declare_parameters_from_overrides = False, enable_logger_service = False):
        super().__init__(node_name, context=context, cli_args=cli_args, namespace=namespace, use_global_arguments=use_global_arguments, enable_rosout=enable_rosout, start_parameter_services=start_parameter_services, parameter_overrides=parameter_overrides, allow_undeclared_parameters=allow_undeclared_parameters, automatically_declare_parameters_from_overrides=automatically_declare_parameters_from_overrides, enable_logger_service=enable_logger_service)
        self.arm_workspace_radius = 0.4 #40 cm #TODO fill in with actual radius here

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
                nearby_trash.append(key)
        
        sumx = 0
        sumy = 0
        for x,y in nearby_trash:
            sumx += x
            sumy += y
        centroid = ((sumx)/len(nearby_trash),(sumy)/len(nearby_trash)) #uses geometric average (centroid) of the object to be equadistant to in the middle of the points
        return centroid
            
            