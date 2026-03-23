#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import subprocess
from geometry_msgs.msg import PoseArray, Pose

class GoalEmitter(Node):
    def __init__(self):
        super().__init__('goal_emitter')
        # We use PoseArray to send multiple goals in one "burst"
        self.publisher_ = self.create_publisher(PoseArray, '/detected_goals', 10)
        self.timer = self.create_timer(2.0, self.publish_goals)
        self.sub = self.create_subscription(PoseArray,'/removed_goals',self.remove_goals_callback,10)
        
        self.positions = [(2.5, 2), (2.8, 2.4), (4.0, 4.0),(1.2,2),(1.0,2), (2.4,1), (2.4,-0.4)] 
                
        self.spawn_all_trash()

    def publish_goals(self):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map' # Matches your ROS 2 map frame

        for x, y in self.positions:
            p = Pose()
            p.position.x = float(x)
            p.position.y = float(y)
            p.orientation.w = 1.0 # Neutral orientation
            msg.poses.append(p)

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published {len(msg.poses)} goals')

    def remove_goals_callback(self,msg):
        for p in msg.poses:
            try:
                self.positions.remove((p.position.x,p.position.y))
            except ValueError:
                self.get_logger().warn(f'Goal {(p.position.x,p.position.y)} not in list of goals!')
        
    def spawn_all_trash(self):
        for i, (x, y) in enumerate(self.positions):
            self.spawn_bottle(f"bottle_{i}", x, y)

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

    def spawn_all_trash(self):
        """Loops through initial positions and populates the sim."""
        self.get_logger().info("Spawning initial trash in Gazebo...")
        for i, (x, y) in enumerate(self.positions):
            self.spawn_bottle(f"bottle_{i}", x, y)
    
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
                <geometry><cylinder><radius>0.020</radius><length>0.115</length></cylinder></geometry>
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

def main():
    rclpy.init()
    node = GoalEmitter()
    rclpy.spin(node)
    rclpy.shutdown(node)

if __name__ == "__main__":
    main()