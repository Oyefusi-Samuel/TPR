#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import subprocess
from geometry_msgs.msg import PoseArray, Pose
import math

class GoalEmitter(Node):
    def __init__(self):
        super().__init__('goal_emitter')

        # World name must match the <world name="..."> in the SDF file
        # (used for Gazebo service calls to remove models).
        self.declare_parameter('world_name', 'room_with_walls_star')
        self.world_name = self.get_parameter('world_name').value

        # We use PoseArray to send multiple goals in one "burst"
        self.publisher_ = self.create_publisher(PoseArray, '/detected_goals', 10)
        self.timer = self.create_timer(2.0, self.publish_goals)
        self.sub = self.create_subscription(PoseArray,'/removed_goals',self.remove_goals_callback,10)

        # Spread out bottles, some clustered, away from table at x=1.60
        self.trash_data = [
            # Cluster of 2 at left side
            {'Name': 'bottle0', 'pos': (-0.8, 0.3)},
            {'Name': 'bottle1', 'pos': (-0.75, 0.5)},
            # Isolated bottles
            {'Name': 'bottle2', 'pos': (0.5, 1.2)},
            {'Name': 'bottle3', 'pos': (0.2, -0.9)},
            {'Name': 'bottle4', 'pos': (1.0, 0.1)},
            {'Name': 'bottle5', 'pos': (-0.3, -1.1)},
        ]

        self.spawn_all_trash()

    def publish_goals(self):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map' # Matches your ROS 2 map frame

        for item in self.trash_data:
            x,y = item['pos']
            p = Pose()
            p.position.x = float(x)
            p.position.y = float(y)
            p.orientation.w = 1.0 # Neutral orientation
            msg.poses.append(p)

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published {len(msg.poses)} goals')

    def remove_goals_callback(self, msg):
        for p in msg.poses:
            self.remove_bottle_at_coord(
                target_x=p.position.x, target_y=p.position.y)
        

    #gazebo code below   
    def spawn_all_trash(self):
        for item in self.trash_data:
            x,y = item['pos']
            self.spawn_bottle(item['Name'],x,y)

    def spawn_bottle(self, name, x, y):
        """Uses subprocess to call the Gazebo Harmonic creation tool."""
        sdf_string = self.get_bottle_sdf(name)
        
        # The 'ros_gz_sim create' command is the standard for Gazebo Harmonic
        command = [
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-name', str(name),
            '-x', str(x),
            '-y', str(y),
            '-z', '0.5',  # Spawn slightly above the floor so it drops in
            '-string', sdf_string
        ]
        
        try:
            # We use Popen so the node doesn't freeze while waiting for Gazebo
            subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.get_logger().info(f"Spawn command sent for {name} at ({x}, {y})")
        except Exception as e:
            self.get_logger().error(f"Failed to launch spawn process for {name}: {e}")
    
    def get_bottle_sdf(self, name):
        """Returns the SDF string for the bottle model."""
        return f"""
        <?xml version="1.0" ?>
        <sdf version="1.6">
          <model name="{name}">
            <static>false</static>
            <link name="link">
              <inertial>
                <mass>0.40</mass>
                <inertia>
                  <ixx>0.00045</ixx><iyy>0.00045</iyy><izz>0.00002</izz>
                  <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
                </inertia>
              </inertial>
              <collision name="col">
                <geometry><cylinder><radius>0.005</radius><length>0.115</length></cylinder></geometry>
                <surface><contact><collide_bitmask>0x02</collide_bitmask></contact></surface>
              </collision>
              <visual name="vis">
                <geometry><cylinder><radius>0.020</radius><length>0.115</length></cylinder></geometry>
                <material>
                  <ambient>0.78 0.93 1.0 0.55</ambient>
                  <diffuse>0.78 0.93 1.0 0.55</diffuse>
                </material>
              </visual>
            </link>
          </model>
        </sdf>
        """
    def remove_bottle_at_coord(self, target_x, target_y):
        found_item = None
        tolerance = 0.3 

        for item in self.trash_data:
            dist = math.dist(item["pos"], (target_x, target_y))
            if dist < tolerance:
                found_item = item
                break

        if found_item:
            name_to_remove = found_item["Name"]
            
            # We construct the EXACT string that worked for you in the terminal
            # Note the use of double quotes for the outer string and single quotes for the name
            command = (
                f"gz service -s /world/{self.world_name}/remove "
                f"--reqtype gz.msgs.Entity "
                f"--reptype gz.msgs.Boolean "
                f"--timeout 2000 "
                f"--req \"name: '{name_to_remove}', type: MODEL\""
            )
            
            try:
                # shell=True is critical here to handle the nested quotes in the --req flag
                subprocess.Popen(command, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                
                self.trash_data.remove(found_item)
                self.get_logger().info(f"Successfully triggered Gazebo removal for {name_to_remove}")
            except Exception as e:
                self.get_logger().error(f"Shell command failed: {e}")

def main():
    rclpy.init()
    node = GoalEmitter()
    rclpy.spin(node)
    rclpy.shutdown(node)

if __name__ == "__main__":
    main()