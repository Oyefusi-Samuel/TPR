"""
astar_navigation.launch.py  -  Jackal AR4: A* Smooth Planner Navigation

Runs the A* planner + smoother + pure_pursuit controller
on the Jackal robot simulation using a pre-built map.

Usage (after mapping and saving the map):
  ros2 launch jackal_ar4_navigation astar_navigation.launch.py

  Or with a specific map:
  ros2 launch jackal_ar4_navigation astar_navigation.launch.py \\
      map:=/path/to/your_map.yaml

Prerequisite:
  1. Build the map with:
       ros2 launch jackal_ar4_navigation mapping.launch.py world:=tpr
     Drive the robot around to map the environment, then save:
       ros2 run nav2_map_server map_saver_cli -f ~/jackal_ar4_ws/src/jackal_ar4_navigation/maps/tpr_map
  2. Copy a_star_smooth_planner into your workspace src/ and colcon build.

In RViz, use the '2D Goal Pose' button to send navigation goals.

Architecture:
  - a_star_planner    subscribes  /costmap, /goal_pose   →  publishes /a_star/path
  - a_star_smoother   subscribes  /a_star/path            →  publishes /a_star/path/smooth
  - pure_pursuit      subscribes  /a_star/path/smooth     →  publishes /cmd_vel (TwistStamped)
  - twist_unstamper   subscribes  /cmd_vel (TwistStamped) →  publishes /cmd_vel_twist (Twist)
  - bridge in gazebo.launch.py forwards /cmd_vel (Twist) → Gazebo diff_drive

  Nav2 components:
  - map_server  : loads the saved map
  - amcl        : localises the robot using /lidar/scan + the map
  - costmap     : builds obstacle + inflation layers from /lidar/scan
  - lifecycle_manager activates all three
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_description  = get_package_share_directory('jackal_ar4_description')
    pkg_navigation   = get_package_share_directory('jackal_ar4_navigation')
    pkg_astar        = get_package_share_directory('a_star_smooth_planner')

    default_map    = os.path.join(pkg_navigation, 'maps', 'tpr_map.yaml')
    astar_params   = os.path.join(pkg_astar,     'config', 'nav2_bringup_params.yaml')
    astar_rviz     = os.path.join(pkg_astar,     'config', 'amcl.rviz')

    # ── Args ──────────────────────────────────────────────────────────────
    world_arg = DeclareLaunchArgument(
        'world', default_value='tpr',
        description='Gazebo world to load',
    )
    map_arg = DeclareLaunchArgument(
        'map', default_value=default_map,
        description='Full path to the saved map YAML file',
    )

    world = LaunchConfiguration('world')
    map   = LaunchConfiguration('map')

    # ── 1. Full Jackal simulation (no RViz — we use the A* RViz config) ──
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world,
            'launch_rviz': 'false',   # A* brings its own RViz config
        }.items(),
    )

    # ── 2. A* planner + smoother + pure_pursuit ───────────────────────────
    # Delayed 20 s to let the simulation fully stabilise first.
    #
    # pure_pursuit publishes TwistStamped on /cmd_vel_stamped.
    # The Gazebo bridge expects plain Twist on /cmd_vel.
    # twist_unstamper (below) strips the header.
    a_star_planner = Node(
        package='a_star_smooth_planner',
        executable='a_star_planner.py',
        name='a_star_planner',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

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

    # pure_pursuit publishes TwistStamped on /cmd_vel_stamped
    # (remapped away from /cmd_vel to avoid type conflict with the bridge)
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
        # Remap pure_pursuit output to /cmd_vel_stamped so the
        # twist_unstamper below can convert it to plain Twist
        remappings=[('/cmd_vel', '/cmd_vel_stamped')],
    )

    # twist_unstamper: converts TwistStamped → Twist
    # The A* pure_pursuit node publishes TwistStamped but the Gazebo
    # diff_drive bridge (and teleop_twist_keyboard) use plain Twist.
    # This tiny Python node strips the header so the bridge accepts it.
    twist_unstamper = Node(
        package='jackal_ar4_navigation',
        executable='twist_unstamper.py',
        name='twist_unstamper',
        output='screen',
        parameters=[{'use_sim_time': True}],
        # reads /cmd_vel_stamped, writes /cmd_vel
    )

    astar_nodes = TimerAction(
        period=20.0,
        actions=[a_star_planner, a_star_smoother, pure_pursuit, twist_unstamper],
    )

    # ── 3. Nav2 localisation + costmap (AMCL-based) ───────────────────────
    # Uses the A* package's nav2_bringup_params.yaml which already sets
    # scan_topic: /lidar/scan and the costmap obstacle_layer correctly.
    # Only map_server, amcl, and costmap are needed — no NavFn planner.
    lifecycle_nodes = ['map_server', 'amcl', 'costmap']

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[astar_params, {'use_sim_time': True, 'yaml_filename': map}],
        remappings=remappings,
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[astar_params, {'use_sim_time': True}],
        remappings=remappings,
    )

    costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='costmap',
        output='screen',
        parameters=[astar_params, {'use_sim_time': True}],
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_astar',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'bond_timeout': 0.0,
            'node_names': lifecycle_nodes,
        }],
    )

    nav2_nodes = TimerAction(
        period=20.0,
        actions=[map_server_node, amcl_node, costmap_node, lifecycle_manager_node],
    )

    # ── 4. RViz with A* config ─────────────────────────────────────────────
    rviz_node = TimerAction(
        period=22.0,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', astar_rviz],
            parameters=[{'use_sim_time': True}],
            output='screen',
        )],
    )

    return LaunchDescription([
        world_arg,
        map_arg,
        gazebo_launch,
        astar_nodes,
        nav2_nodes,
        rviz_node,
    ])
