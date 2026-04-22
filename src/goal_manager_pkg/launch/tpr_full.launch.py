"""tpr_full.launch.py — Complete TPR bringup in one command

Usage:
    ros2 launch goal_manager_pkg tpr_full.launch.py

This launches the entire Jackal AR4 trash pickup system:
  1. Gazebo simulation with TPR world
  2. A* navigation stack (planner + smoother)
  3. Mission pipeline (trash detection + planning + pickup)

All components start with proper sequencing. The simulation is given 8 seconds
to fully initialize before navigation and mission nodes start.
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    pkg_description = get_package_share_directory('jackal_ar4_description')
    pkg_navigation = get_package_share_directory('jackal_ar4_navigation')
    pkg_goals = get_package_share_directory('goal_manager_pkg')

    # Launch gazebo simulation with TPR world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': 'tpr'}.items(),
    )

    # Launch A* navigation (delayed to allow Gazebo to initialize)
    nav_launch = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_navigation, 'launch', 'astar_navigation.launch.py')
                )
            )
        ],
    )

    # Note: gazebo.launch.py already includes goal_manager.launch.py at 18s,
    # so we only need Gazebo + navigation here.

    return LaunchDescription([
        gazebo_launch,
        nav_launch,
    ])
