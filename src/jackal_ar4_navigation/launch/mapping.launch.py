"""
mapping.launch.py  -  SLAM Toolbox mapping session for Jackal AR4

WORKFLOW:
  Terminal 1 (sim, already running):
    ros2 launch jackal_ar4_description gazebo.launch.py world:=tpr

  Terminal 2 (this launch):
    ros2 launch jackal_ar4_navigation mapping.launch.py

  Terminal 3 (teleop):
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel

  Drive the robot around the TPR world to build the map.
  When done, save the map:
    ros2 run nav2_map_server map_saver_cli -f ~/tpr/jackal_ar4_ws/src/jackal_ar4_navigation/maps/tpr_map

  Then rebuild:
    cd ~/tpr/jackal_ar4_ws && colcon build --symlink-install && source install/setup.bash
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_navigation   = get_package_share_directory('jackal_ar4_navigation')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    slam_params_file = os.path.join(
        pkg_navigation, 'config', 'slam_toolbox_mapping_params.yaml'
    )
    rviz_config_file = os.path.join(
        pkg_navigation, 'config', 'mapping.rviz'
    )

    # slam_toolbox's own online_async_launch.py handles all lifecycle setup
    slam_toolbox_launch_path = os.path.join(
        pkg_slam_toolbox, 'launch', 'online_async_launch.py'
    )

    # ── Args ──────────────────────────────────────────────────────────────
    slam_params_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=slam_params_file,
        description='Full path to SLAM Toolbox parameter file',
    )
    slam_params = LaunchConfiguration('slam_params_file')

    # ── SLAM Toolbox via its own launch ───────────────────────────────────
    # Using IncludeLaunchDescription on slam_toolbox's online_async_launch.py
    # is the correct pattern (same as mobo_bot). It internally handles the
    # node arguments including use_lifecycle_manager correctly.
    #
    # Our slam_toolbox_mapping_params.yaml also sets:
    #   use_lifecycle_manager: false  ← ensures node self-configures
    #   scan_topic: /lidar/scan       ← our bridged LiDAR topic
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch_path),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': slam_params,
        }.items(),
    )

    # ── RViz ──────────────────────────────────────────────────────────────
    # Fixed frame = map. Map display uses Transient Local QoS to match
    # slam_toolbox's latched /map topic — this is why the map was not visible
    # before (Volatile QoS means RViz misses the latched message).
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_mapping',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        slam_params_arg,
        slam_launch,
        rviz_node,
    ])
