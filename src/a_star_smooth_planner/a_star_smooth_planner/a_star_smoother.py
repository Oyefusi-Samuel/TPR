#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose
from rclpy.qos import QoSProfile, DurabilityPolicy

from queue import Queue

class GridPose:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y



class AStarSmoother(Node):
    def __init__(self):
        super().__init__("a_star_smoother")

        self.declare_parameter("iterations", 2)
        self.declare_parameter("cost_limit", 20)
        self.iterations = self.get_parameter("iterations").value
        self.cost_limit = self.get_parameter("cost_limit").value

        map_qos = QoSProfile(depth=10)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.map_sub = self.create_subscription(
            OccupancyGrid, "/costmap", self.map_callback, map_qos
        )
        self.path_sub = self.create_subscription(Path, "/a_star/path", self.path_callback, 10)
        self.path_pub = self.create_publisher(Path, "/a_star/path/smooth", 10)

        self.map_ = None
        self.path_ = None

        self.get_logger().info("a_star_smoother node has just started")

    def pose_to_grid(self, pose: Pose) -> GridPose:
        grid_x = int((pose.position.x - self.map_.info.origin.position.x) / self.map_.info.resolution)
        grid_y = int((pose.position.y - self.map_.info.origin.position.y) / self.map_.info.resolution)
        return GridPose(grid_x, grid_y)

    def grid_to_pose(self, grid_pose: GridPose) -> Pose:
        pose = Pose()
        pose.position.x = grid_pose.x * self.map_.info.resolution + self.map_.info.origin.position.x
        pose.position.y = grid_pose.y * self.map_.info.resolution + self.map_.info.origin.position.y
        return pose

    def grid_to_cell(self, grid_pose: GridPose):
        return grid_pose.y * self.map_.info.width + grid_pose.x
    
    def line_crosses_obstacle(self, line): 
        map_data_list = self.map_.data
        grid_to_cell = self.grid_to_cell
        cost_limit = self.cost_limit       
        for grid_pose in line:
            cell_index = grid_to_cell(grid_pose)
            if map_data_list[cell_index] > cost_limit:
                return True
        return False
    
    def map_callback(self, map_msg: OccupancyGrid):
        self.map_ = map_msg

    def path_callback(self, path_msg: Path):
        if (self.map_ == None):
            self.get_logger().error("No Map data received")
        
        self.get_logger().info("Path received")
        self.path_ = path_msg
        new_path = self.smoothen_path(self.path_)
        
        self.path_pub.publish(new_path)
        self.get_logger().info('smoothned a_star path published')

    def smoothen_path(self, path: Path) -> Path:
        pose_to_grid = self.pose_to_grid
        grid_to_pose = self.grid_to_pose
        # bresenham = self.bresenham_line
        smoothen_grid_path = self.smoothen_grid_path

        grid_pose_list = [pose_to_grid(pose.pose) for pose in path.poses]

        for _ in range(self.iterations):
            grid_pose_list = smoothen_grid_path(grid_pose_list)

        new_path = Path()
        new_path.header.frame_id = self.map_.header.frame_id
        for grid_pose in grid_pose_list:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = self.map_.header.frame_id
            pose_stamped.pose = grid_to_pose(grid_pose)
            new_path.poses.append(pose_stamped)
        
        return new_path
    
    def smoothen_grid_path(self, grid_pose_list):
        pose_queue_check = Queue(maxsize=2)
        new_grid_pose_list = []

        start_grid_pose = grid_pose_list[0]
        goal_grid_pose = grid_pose_list[-1]

        bresenham = self.bresenham_line
        line_crosses_obstacle = self.line_crosses_obstacle

        no_of_path = len(grid_pose_list)
        touch_obstacle = False
        i = 1

        if no_of_path == 1:
            self.get_logger().warning("only one path was found")
            new_grid_pose_list.append(start_grid_pose)
            return 
        
        pose_queue_check.put(start_grid_pose)
        new_grid_pose_list.append(start_grid_pose)
        
        while i < no_of_path and rclpy.ok():
            next_grid_pose = grid_pose_list[i]
            pose_queue_check.put(next_grid_pose)

            line = bresenham(start_grid_pose, next_grid_pose)
            touch_obstacle = line_crosses_obstacle(line)

            if touch_obstacle:
                next_grid_pose = pose_queue_check.get()
                new_grid_pose_list.append(next_grid_pose)
                start_grid_pose = next_grid_pose
                pose_queue_check.get()
                pose_queue_check.put(next_grid_pose)
                touch_obstacle = False
            
            else:
                pose_queue_check.get()
                i+=1

        new_grid_pose_list.append(goal_grid_pose)

        full_new_grid_pose_list = []
        full_new_grid_pose_list.append(new_grid_pose_list[0])
        length = len(new_grid_pose_list)
        for i in range(1, length):
            line = bresenham(new_grid_pose_list[i-1], new_grid_pose_list[i])
            full_new_grid_pose_list.extend(line[1:])
        return full_new_grid_pose_list
    
    def bresenham_line(self, start: GridPose, end: GridPose):
        line = []

        x0, y0, x1, y1 = start.x, start.y, end.x, end.y

        dx = x1 - x0
        dy = y1 - y0

        xsign = 1 if dx > 0 else -1
        ysign = 1 if dy > 0 else -1

        dx = abs(dx)
        dy = abs(dy)

        if dx > dy:
            xx = xsign
            xy = 0
            yx = 0
            yy = ysign
        else:
            tmp = dx
            dx = dy
            dy = tmp
            xx = 0
            xy = ysign
            yx = xsign
            yy = 0

        D = 2 * dy - dx
        y = 0

        for i in range(dx + 1):
            line.append(GridPose(x0 + i * xx + y * yx, y0 + i * xy + y * yy))
            if D >= 0:
                y += 1
                D -= 2 * dx
            D += 2 * dy

        return line



def main(args=None):
    rclpy.init(args=args)
    node = AStarSmoother()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()