"""
gazebo.launch.py  -  Jackal + AR4 in Gazebo Harmonic (ROS 2 Jazzy)

Launch examples:
  ros2 launch jackal_ar4_description gazebo.launch.py
  ros2 launch jackal_ar4_description gazebo.launch.py world:=tpr
  ros2 launch jackal_ar4_description gazebo.launch.py world:=room_with_walls
"""

import os

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess,
    IncludeLaunchDescription, TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    pkg_description  = get_package_share_directory('jackal_ar4_description')
    pkg_moveit       = get_package_share_directory('jackal_ar4_moveit_config')
    pkg_navigation   = get_package_share_directory('jackal_ar4_navigation')
    pkg_ar4_desc     = get_package_share_directory('ar4_description')
    pkg_clearpath    = get_package_share_directory('clearpath_platform_description')
    controllers_yaml = os.path.join(pkg_moveit, 'config', 'ros2_controllers.yaml')

    # Worlds package is optional — fall back to Gazebo built-in empty world
    try:
        pkg_worlds = get_package_share_directory('jackal_ar4_worlds')
        worlds_dir = os.path.join(pkg_worlds, 'worlds')
        # BUG FIX: use only the basename here.
        # world_path is built as PathJoinSubstitution([worlds_dir, [world_name, '.sdf']]),
        # so if default_world were the full path the launch would produce
        # "…/worlds/empty.sdf.sdf" and Gazebo would fail to start.
        default_world = 'empty'
    except PackageNotFoundError:
        pkg_worlds = None
        worlds_dir = None
        default_world = 'empty.sdf'  # Gazebo's built-in fallback

    gz_resource_paths = [
        os.path.dirname(pkg_ar4_desc),
        os.path.dirname(pkg_clearpath),
        os.path.dirname(pkg_description),
    ]
    if worlds_dir:
        gz_resource_paths.append(worlds_dir)

    gz_env = {'GZ_SIM_RESOURCE_PATH': ':'.join(gz_resource_paths)}

    # ── Args ──────────────────────────────────────────────────────────────
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World name (no .sdf) or full path. '
                    'Available: empty, empty_room, room_with_walls, '
                    'room_with_walls_star, turtlebot_arena, tpr',
    )
    launch_rviz_arg = DeclareLaunchArgument('launch_rviz', default_value='true')

    world_name  = LaunchConfiguration('world')
    launch_rviz = LaunchConfiguration('launch_rviz')

    # If worlds package exists, resolve short name -> full path
    # Otherwise pass the value directly (either full path or built-in name)
    if worlds_dir:
        world_path = PathJoinSubstitution([worlds_dir, [world_name, '.sdf']])
    else:
        world_path = world_name

    robot_description_content = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ',
            os.path.join(pkg_description, 'urdf', 'jackal_ar4.urdf.xacro'),
            ' is_sim:=true',
            ' use_platform_controllers:=false',
        ]),
        value_type=str,
    )
    robot_description = {'robot_description': robot_description_content}

    # ── 1. Gazebo ─────────────────────────────────────────────────────────
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        additional_env=gz_env,
        output='screen',
    )

    # ── 2. Robot State Publisher ──────────────────────────────────────────
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': True}],
        output='screen',
    )

    # ── 3. Spawn into Gazebo ──────────────────────────────────────────────
    # Delay spawn until Gazebo is ready
    spawn_robot = TimerAction(period=3.0, actions=[Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'jackal_ar4', '-topic', 'robot_description', '-z', '0.15'],
        output='screen',
    )])

    # ── 4. Bridges ────────────────────────────────────────────────────────
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/wheel_joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    tf_static_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='tf_static_bridge',
        arguments=['/tf_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
        parameters=[{
            'use_sim_time': True,
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen',
    )

    # ── 5. Wheel joint state relay ────────────────────────────────────────
    wheel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='wheel_state_relay',
        arguments=['/wheel_joint_states', '/joint_states'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ── 6. map -> odom (no use_sim_time — avoids timestamp=0 bounce) ──────
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--yaw', '0', '--pitch', '0', '--roll', '0',
                   '--frame-id', 'map', '--child-frame-id', 'odom'],
    )

    # ── 7. Controller Manager ─────────────────────────────────────────────
    # Delayed until sim clock is flowing — prevents use_sim_time freeze
    controller_manager = TimerAction(period=5.0, actions=[Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        parameters=[robot_description, controllers_yaml, {'use_sim_time': True}],
        output='screen',
    )])

    # ── 8. Spawners — delayed well after controller_manager + sim clock ───
    def spawner(name, delay):
        return TimerAction(period=delay, actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name],
            parameters=[{'use_sim_time': True}],
            output='screen',
        )])

    spawn_jsb     = spawner('joint_state_broadcaster', delay=8.0)
    spawn_arm     = spawner('arm_controller',           delay=9.0)
    spawn_gripper = spawner('ar_gripper_controller',    delay=9.0)

    # ── 9. Nav2 ───────────────────────────────────────────────────────────
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_navigation, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    # ── 10. MoveIt move_group ─────────────────────────────────────────────
    moveit_config = (
        MoveItConfigsBuilder('jackal_ar4', package_name='jackal_ar4_moveit_config')
        .robot_description(
            file_path=os.path.join(pkg_description, 'urdf', 'jackal_ar4.urdf.xacro'),
            mappings={'is_sim': 'true', 'use_platform_controllers': 'false'},
        )
        .robot_description_semantic(file_path='config/jackal_ar4.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .planning_pipelines(pipelines=['ompl'], default_planning_pipeline='ompl')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    move_group = TimerAction(period=11.0, actions=[Node(
        package='moveit_ros_move_group',
        executable='move_group',
        parameters=[moveit_config.to_dict(), {'use_sim_time': True}],
        output='screen',
    )])

    # ── 11. RViz ──────────────────────────────────────────────────────────
    rviz_config = os.path.join(pkg_moveit, 'config', 'moveit.rviz')
    if not os.path.exists(rviz_config):
        rviz_config = os.path.join(pkg_description, 'config', 'rviz_config.rviz')

    rviz = TimerAction(period=12.0, actions=[Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[moveit_config.to_dict(), {'use_sim_time': True}],
        condition=IfCondition(launch_rviz),
        output='screen',
    )])

    return LaunchDescription([
        world_arg,
        launch_rviz_arg,
        gz_sim,
        rsp_node,
        bridge,
        tf_static_bridge,
        wheel_relay,
        map_to_odom,
        spawn_robot,       # t=3s
        controller_manager,# t=5s  (after sim clock is flowing)
        spawn_jsb,         # t=8s
        spawn_arm,         # t=9s
        spawn_gripper,     # t=9s
        nav2,
        move_group,        # t=11s
        rviz,              # t=12s
    ])