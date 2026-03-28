"""
astar_navigation.launch.py  -  Jackal AR4: A* Smooth Planner Navigation

Run this AFTER the simulation is already running:
  Terminal 1:  ros2 launch jackal_ar4_description gazebo.launch.py world:=tpr
  Terminal 2:  ros2 launch jackal_ar4_navigation astar_navigation.launch.py
  Terminal 3:  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel

Custom map:
  ros2 launch jackal_ar4_navigation astar_navigation.launch.py map:=/full/path/to/map.yaml

Then use '2D Goal Pose' in RViz to send navigation goals.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_navigation = get_package_share_directory('jackal_ar4_navigation')
    pkg_astar      = get_package_share_directory('a_star_smooth_planner')

    # ── Paths ──────────────────────────────────────────────────────────────
    default_map  = os.path.join(pkg_navigation, 'maps', 'tpr_map.yaml')
    astar_params = os.path.join(pkg_astar, 'config', 'nav2_bringup_params.yaml')
    # Use our own navigation.rviz (in jackal_ar4_navigation/config/)
    # which fixes the GLSL map shader bug by using Color Scheme: raw for /map
    # instead of the default "map" scheme used in a_star's amcl.rviz
    astar_rviz   = os.path.join(pkg_navigation, 'config', 'navigation.rviz')

    # ── Args ───────────────────────────────────────────────────────────────
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to tpr_map.yaml',
    )
    map_path = LaunchConfiguration('map')

    # ─────────────────────────────────────────────────────────────────────
    # NAV2 STACK
    #
    # CRITICAL FIXES vs previous version:
    #
    # Fix 1 — lifecycle_manager MUST NOT have use_sim_time:True
    #   The lifecycle_manager uses wall clock for bond health checks internally.
    #   With use_sim_time:True set on the node, its timer runs on sim clock.
    #   If sim clock is slow to arrive the bond check fires at t=0 and
    #   immediately reports failure → "Failed to change state for node: map_server"
    #   even though the map loaded fine.
    #   Solution: match nav_test.launch.py exactly — no use_sim_time on lifecycle_manager.
    #
    # Fix 2 — individual nodes only need params_file + yaml_filename override
    #   use_sim_time:true is already in nav2_bringup_params.yaml for every node.
    #   Adding it again as a separate Python dict caused param namespace conflicts.
    #   Solution: pass [astar_params, {'yaml_filename': map_path}] only — 
    #   same pattern as the original nav_test.launch.py in the A* package.
    # ─────────────────────────────────────────────────────────────────────

    lifecycle_nodes = ['map_server', 'amcl', 'costmap']

    # map_server: loads tpr_map.yaml, publishes /map (Transient Local QoS)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[astar_params, {'yaml_filename': map_path}],
    )

    # amcl: localises robot using /lidar/scan against /map
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[astar_params],
    )

    # costmap: builds obstacle + inflation layers from /lidar/scan
    # a_star_planner reads from /costmap (OccupancyGrid) published by this node
    costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='costmap',
        output='screen',
        parameters=[astar_params],
    )

    # lifecycle_manager: activates map_server → amcl → costmap in sequence
    # NO use_sim_time here — lifecycle_manager bond checks run on wall clock.
    # Delayed 3s so map_server/amcl/costmap nodes are fully initialized
    # before the lifecycle_manager sends the configure transition.
    # Without the delay: lifecycle_manager fires configure at the same instant
    # map_server is created → configure call arrives before map_server's service
    # is ready → "Failed to change state" race condition.
    lifecycle_manager_node = TimerAction(
        period=3.0,
        actions=[Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_astar',
            output='screen',
            parameters=[
                {'autostart': True, 'bond_timeout': 0.0},
                {'node_names': lifecycle_nodes},
            ],
        )],
    )

    # ── A* Planner ─────────────────────────────────────────────────────────
    # Subscribes: /costmap (OccupancyGrid), /goal_pose (PoseStamped)
    # Publishes:  /a_star/path (Path)
    a_star_planner = Node(
        package='a_star_smooth_planner',
        executable='a_star_planner.py',
        name='a_star_planner',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ── A* Smoother ────────────────────────────────────────────────────────
    # Subscribes: /a_star/path
    # Publishes:  /a_star/path/smooth
    a_star_smoother = Node(
        package='a_star_smooth_planner',
        executable='a_star_smoother',
        name='a_star_smoother',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'iterations': 5,
            'cost_limit': 20,
        }],
    )

    # ── Pure Pursuit ───────────────────────────────────────────────────────
    # Subscribes: /a_star/path/smooth
    # Publishes:  /cmd_vel_stamped (TwistStamped — remapped from /cmd_vel)
    # twist_unstamper converts /cmd_vel_stamped → /cmd_vel (plain Twist for bridge)
    pure_pursuit = Node(
        package='a_star_smooth_planner',
        executable='pure_pursuit',
        name='pure_pursuit',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'look_ahead_distance': 0.3,
            'max_linear_velocity': 0.25,
            'max_angular_velocity': 1.0,
            'path_topic': '/a_star/path/smooth',
        }],
        remappings=[('/cmd_vel', '/cmd_vel_stamped')],
    )

    # ── Twist Unstamper ────────────────────────────────────────────────────
    # Converts /cmd_vel_stamped (TwistStamped) → /cmd_vel (Twist)
    twist_unstamper = Node(
        package='jackal_ar4_navigation',
        executable='twist_unstamper.py',
        name='twist_unstamper',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ── RViz ───────────────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', astar_rviz],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        map_arg,
        # Nav2
        map_server_node,
        amcl_node,
        costmap_node,
        lifecycle_manager_node,
        # A* stack
        a_star_planner,
        a_star_smoother,
        pure_pursuit,
        twist_unstamper,
        # RViz
        rviz_node,
    ])