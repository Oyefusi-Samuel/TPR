from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_navigation = FindPackageShare('jackal_ar4_navigation')
    
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')

    arg_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([pkg_navigation, 'config', 'nav2_params.yaml'])
    )

    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false'
    )

    arg_map_yaml_file = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([pkg_navigation, 'maps', 'map.yaml'])
    )

    # Define the nodes we actually want to run
    # Excluding collision_monitor, route_server, docking_server
    lifecycle_nodes = [
        'map_server',
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'velocity_smoother',
        'smoother_server'
    ]

    nodes = [
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[params_file, {'use_sim_time': use_sim_time, 'yaml_filename': map_yaml_file}]
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=[('cmd_vel', 'cmd_vel')] # Direct output since we removed collision_monitor
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            parameters=[params_file, {'use_sim_time': use_sim_time}]
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[params_file, {'use_sim_time': use_sim_time}]
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            parameters=[params_file, {'use_sim_time': use_sim_time}]
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            parameters=[params_file, {'use_sim_time': use_sim_time}]
        ),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            parameters=[params_file, {'use_sim_time': use_sim_time}]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            parameters=[{'use_sim_time': use_sim_time, 'autostart': True, 'node_names': lifecycle_nodes}]
        )
    ]

    return LaunchDescription([
        arg_params_file,
        arg_use_sim_time,
        arg_map_yaml_file,
        *nodes
    ])
