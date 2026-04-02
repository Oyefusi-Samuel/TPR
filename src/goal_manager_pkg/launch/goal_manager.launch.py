import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('goal_manager_pkg')
    
    # Path to your saved RViz config (Make sure you saved it to the rviz/ folder!)
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'planner_view.rviz')

    return LaunchDescription([
        # The Goal Planner Node
        Node(
            package='goal_manager_pkg',
            executable='trash_planner_node',
            name='trash_planner',
            output='screen',
            parameters=[{'use_sim_time': True}] # Helpful for simulation
        ),

        # The Trash Generator Node
        Node(
            package='goal_manager_pkg',
            executable='trash_generator_node',
            name='trash_generator',
            output='screen'
        ),

        # RViz with your custom settings
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', rviz_config_path],
        #     condition=None # You can add logic to toggle this off if needed
        # )
    ])
