import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='room_with_walls_star',
        description='Gazebo <world name="..."> from the SDF (for model removal)',
    )
    world_name = LaunchConfiguration('world_name')

    return LaunchDescription([
        world_name_arg,

        # Workspace executive — arm pick-and-place action server
        # (from jackal_ar4_goals; needs move_group + gripper controller running)
        Node(
            package='jackal_ar4_goals',
            executable='workspace_executive',
            name='workspace_executive',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        # Trash planner — finds closest trash, publishes /planned_goal
        Node(
            package='goal_manager_pkg',
            executable='trash_planner_node',
            name='trash_planner',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'auto_remove': False,  # mission_executive handles removal
            }],
        ),

        # Trash generator — spawns bottles, publishes /detected_goals
        Node(
            package='goal_manager_pkg',
            executable='trash_generator_node',
            name='trash_generator',
            output='screen',
            parameters=[{'world_name': world_name}],
        ),

        # Mission executive — orchestrates nav → pick → remove pipeline
        Node(
            package='goal_manager_pkg',
            executable='mission_executive',
            name='mission_executive',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])
