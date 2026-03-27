"""
mapping.launch.py  -  Jackal AR4: SLAM Toolbox mapping session

Usage:
  ros2 launch jackal_ar4_navigation mapping.launch.py
  ros2 launch jackal_ar4_navigation mapping.launch.py world:=tpr

This launches:
  1. The full Gazebo simulation (gazebo.launch.py)
  2. SLAM Toolbox in online async mode — builds a map from /lidar/scan

While it runs, drive the robot with teleop to map the environment:
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel

When you are satisfied with the map, SAVE IT:
  ros2 run nav2_map_server map_saver_cli -f ~/jackal_ar4_ws/src/jackal_ar4_navigation/maps/tpr_map

Then use it for A* navigation:
  ros2 launch jackal_ar4_navigation astar_navigation.launch.py map:=<full_path>/tpr_map.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    pkg_description  = get_package_share_directory('jackal_ar4_description')
    pkg_navigation   = get_package_share_directory('jackal_ar4_navigation')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    slam_params_file = os.path.join(
        pkg_navigation, 'config', 'slam_toolbox_mapping_params.yaml'
    )

    # ── Args ──────────────────────────────────────────────────────────────
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='tpr',
        description='Gazebo world name (no .sdf). '
                    'Available: empty, empty_room, room_with_walls, '
                    'room_with_walls_star, turtlebot_arena, tpr',
    )
    slam_params_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=slam_params_file,
        description='Full path to SLAM Toolbox parameter file',
    )

    world       = LaunchConfiguration('world')
    slam_params = LaunchConfiguration('slam_params_file')

    # ── 1. Full simulation (Gazebo + controllers + Nav2 + RViz) ──────────
    # launch_rviz=true so we see the map being built live in RViz.
    # Nav2 is included by gazebo.launch.py; it is useful to have it running
    # so the costmaps are visible, but navigation goals are not needed here.
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world,
            'launch_rviz': 'true',
        }.items(),
    )

    # ── 2. SLAM Toolbox — online async mapping ────────────────────────────
    # Delayed 20 s to ensure Gazebo, RSP, controllers and bridges are all
    # fully up before SLAM starts subscribing to /lidar/scan and /tf.
    slam_launch = TimerAction(
        period=20.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'slam_params_file': slam_params,
            }.items(),
        )],
    )

    return LaunchDescription([
        world_arg,
        slam_params_arg,
        gazebo_launch,
        slam_launch,
    ])
