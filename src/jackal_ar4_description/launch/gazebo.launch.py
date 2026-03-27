"""
gazebo.launch.py  -  Jackal + AR4 in Gazebo Harmonic (ROS 2 Jazzy)

Launch examples:
  ros2 launch jackal_ar4_description gazebo.launch.py
  ros2 launch jackal_ar4_description gazebo.launch.py world:=tpr
  ros2 launch jackal_ar4_description gazebo.launch.py world:=room_with_walls
"""

import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix, PackageNotFoundError
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

    # GZ_SIM_RESOURCE_PATH must point at the models/ directory.
    # World SDFs use <include><uri>room_with_walls_star</uri> which Gazebo
    # resolves by searching for a subfolder named 'room_with_walls_star'
    # containing model.config inside GZ_SIM_RESOURCE_PATH.
    models_dir = os.path.join(pkg_worlds, 'models') if pkg_worlds else None

    gz_resource_paths = [
        os.path.dirname(pkg_ar4_desc),
        os.path.dirname(pkg_clearpath),
        os.path.dirname(pkg_description),
    ]
    if models_dir:
        gz_resource_paths.append(models_dir)

    # GZ_SIM_SYSTEM_PLUGIN_PATH must include the ROS lib dir so Gazebo can find
    # libgz_ros2_control-system.so (installed by ros-jazzy-gz-ros2-control).
    # The ROS setup.bash sets this in interactive shells but the value is not
    # reliably inherited by the Gazebo subprocess launched here.
    gz_plugin_paths = [get_package_prefix('gz_ros2_control') + '/lib']
    existing = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    if existing:
        gz_plugin_paths.append(existing)

    gz_env = {
        'GZ_SIM_RESOURCE_PATH': ':'.join(gz_resource_paths),
        'GZ_SIM_SYSTEM_PLUGIN_PATH': ':'.join(gz_plugin_paths),
    }

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
            ' gazebo_controllers:=', controllers_yaml,
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
    # Delay increased to 8s — complex worlds (tpr, room_with_walls) take
    # longer to load.  Spawning too early causes a silent failure where
    # Gazebo's world service isn't ready yet and the robot never appears.
    spawn_robot = TimerAction(period=8.0, actions=[Node(
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
            # LiDAR scan — Gazebo→ROS, topic name matches gz_frame_id in URDF
            '/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
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
    # The gz_ros2_control Gazebo plugin (embedded in the URDF) hosts the
    # controller_manager internally — no standalone ros2_control_node needed.
    # The plugin activates when Gazebo spawns the robot (t=8s).

    # ── 8. Spawners — delayed well after robot spawn + gz_ros2_control init ─
    # --controller-manager-timeout 30 lets spawners retry while the plugin CM
    # finishes loading, avoiding a race with complex world load times.
    def spawner(name, delay):
        return TimerAction(period=delay, actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name, '--controller-manager-timeout', '30'],
            parameters=[{'use_sim_time': True}],
            output='screen',
        )])

    spawn_jsb     = spawner('joint_state_broadcaster', delay=13.0)
    spawn_arm     = spawner('arm_controller',           delay=14.0)
    spawn_gripper = spawner('ar_gripper_controller',    delay=14.0)

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

    move_group = TimerAction(period=16.0, actions=[Node(
        package='moveit_ros_move_group',
        executable='move_group',
        parameters=[moveit_config.to_dict(), {
            'use_sim_time': True,
            # Tolerate start-state joint values that land exactly on a boundary
            # (e.g. ar4_joint_6 at -2.70526 after hitting its limit).
            # The adapter clamps to the boundary rather than rejecting the plan.
            # In Gazebo the JointTrajectoryController can overshoot a joint
            # limit significantly (ar4_joint_2 overshoots by ~1.57 rad when
            # reaching a pose at the edge of the workspace), causing
            # CheckStartStateBounds to reject the next plan.
            # CheckStartStateBounds CLAMPS the start state to the nearest
            # valid bound when the deviation is within this value, so 2.0 rad
            # covers any realistic overshoot while still clamping (not
            # ignoring) the out-of-bounds value before planning.
            'start_state_max_bounds_error': 2.0,
            # Gazebo sim has latency between joint-state publication and
            # trajectory dispatch, and joints can still be settling after a
            # trajectory completes (especially joint_6 after a wrist rotation).
            # 0.1 rad (~5.7°) gives enough slack for post-trajectory settle
            # without masking genuinely bad start states.
            'trajectory_execution.allowed_start_tolerance': 0.1,
        }],
        output='screen',
    )])

    # ── 11. RViz ──────────────────────────────────────────────────────────
    rviz_config = os.path.join(pkg_moveit, 'config', 'moveit.rviz')
    if not os.path.exists(rviz_config):
        rviz_config = os.path.join(pkg_description, 'config', 'rviz_config.rviz')

    rviz = TimerAction(period=17.0, actions=[Node(
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
        spawn_robot,       # t=8s  — robot spawned, gz_ros2_control plugin starts
        spawn_jsb,         # t=13s — joint_state_broadcaster
        spawn_arm,         # t=14s — arm_controller (JointTrajectoryController)
        spawn_gripper,     # t=14s — ar_gripper_controller
        nav2,
        move_group,        # t=11s
        rviz,              # t=12s
    ])