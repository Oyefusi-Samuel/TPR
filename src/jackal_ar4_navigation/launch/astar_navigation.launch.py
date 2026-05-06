"""
astar_navigation.launch.py  -  Jackal AR4: A* Smooth Planner Navigation

Run this AFTER the simulation is already running:
  Terminal 1:  ros2 launch jackal_ar4_description gazebo.launch.py world:=tpr
  Terminal 2:  ros2 launch jackal_ar4_navigation astar_navigation.launch.py
  Terminal 3:  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel

Custom map:
  ros2 launch jackal_ar4_navigation astar_navigation.launch.py map:=/full/path/to/map.yaml

Then use '2D Goal Pose' in RViz to send navigation goals.

Diagnostic commands (run in separate terminals):
  ros2 lifecycle get /amcl            # should be "active"
  ros2 lifecycle get /map_server      # should be "active"
  ros2 topic hz /lidar/scan           # should show ~10 Hz
  ros2 topic hz /map                  # should show data (transient local)
  ros2 run tf2_ros tf2_echo map odom  # should show AMCL's correction transform
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_navigation = get_package_share_directory('jackal_ar4_navigation')
    pkg_astar      = get_package_share_directory('a_star_smooth_planner')

    # ── Paths ──────────────────────────────────────────────────────────────
    default_map  = os.path.join(pkg_navigation, 'maps', 'tpr_map.yaml')
    astar_params = os.path.join(pkg_astar, 'config', 'nav2_bringup_params.yaml')
    astar_rviz   = os.path.join(pkg_navigation, 'config', 'navigation.rviz')

    # ── Args ───────────────────────────────────────────────────────────────
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to tpr_map.yaml',
    )
    use_amcl_arg = DeclareLaunchArgument(
        'use_amcl', default_value='false',
        description='Use AMCL for map->odom localization. '
                    'Default false: ground-truth from Gazebo (2 Hz subprocess).',
    )
    map_path  = LaunchConfiguration('map')
    use_amcl  = LaunchConfiguration('use_amcl')

    # ────────────────────────────────────────────────────────────────────��
    # NAV2 STACK  —  Matches a_star_smooth_planner/launch/nav_test.launch.py
    #
    # lifecycle_manager MUST NOT have use_sim_time:True (bond checks use
    # wall clock).  Individual nodes get use_sim_time from the yaml.
    # ─────────────────────────────────────────────────────────────────────

    lifecycle_nodes_amcl    = ['map_server', 'amcl', 'costmap']
    lifecycle_nodes_no_amcl = ['map_server', 'costmap']

    # Standard Nav2 TF remappings (matches nav_test.launch.py)
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[astar_params, {'yaml_filename': map_path}],
        remappings=remappings,
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[astar_params],
        remappings=remappings,
        condition=IfCondition(use_amcl),
    )

    costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='costmap',
        output='screen',
        parameters=[astar_params],
        remappings=remappings,
    )

    # Lifecycle manager — NO use_sim_time, NO bond timeout.
    # Delayed 3s so nodes are initialized before configure transition.
    lifecycle_manager_node_with_amcl = TimerAction(
        period=3.0,
        actions=[Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_astar',
            output='screen',
            parameters=[
                {'autostart': True, 'bond_timeout': 0.0},
                {'node_names': lifecycle_nodes_amcl},
            ],
            condition=IfCondition(use_amcl),
        )],
    )
    lifecycle_manager_node_no_amcl = TimerAction(
        period=3.0,
        actions=[Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_astar',
            output='screen',
            parameters=[
                {'autostart': True, 'bond_timeout': 0.0},
                {'node_names': lifecycle_nodes_no_amcl},
            ],
            condition=UnlessCondition(use_amcl),
        )],
    )

    # ── Ground-truth localization (fallback when use_amcl:=false) ────────
    gt_localization = Node(
        package='jackal_ar4_navigation',
        executable='gz_ground_truth_odom.py',
        name='gz_ground_truth_localization',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=UnlessCondition(use_amcl),
    )

    # ── A* Planner ───────────────────────────────────────────────────────
    a_star_planner = Node(
        package='a_star_smooth_planner',
        executable='a_star_planner.py',
        name='a_star_planner',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ── A* Smoother ──────────────────────────────────────────────────────
    a_star_smoother = Node(
        package='a_star_smooth_planner',
        executable='a_star_smoother',
        name='a_star_smoother',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'iterations': 5,
            'cost_limit': 90,
        }],
    )

    # ── Pure Pursuit ─────────────────────────────────────────────────────
    # NOTE: pure_pursuit.cpp was patched to look up "map"→"base_link"
    # instead of "odom"→"base_link", so it uses the AMCL-corrected pose.
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

    # ── Twist Unstamper ──────────────────────────────────────────────────
    twist_unstamper = Node(
        package='jackal_ar4_navigation',
        executable='twist_unstamper.py',
        name='twist_unstamper',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ── RViz ─────────────────────────────────────────────────────────────
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
        use_amcl_arg,
        # Nav2 (map + localization + costmap)
        map_server_node,
        amcl_node,              # only when use_amcl:=true
        costmap_node,
        lifecycle_manager_node_with_amcl,
        lifecycle_manager_node_no_amcl,
        gt_localization,        # only when use_amcl:=false
        # A* planning + control
        a_star_planner,
        a_star_smoother,
        pure_pursuit,
        twist_unstamper,
        # RViz
        rviz_node,
    ])
