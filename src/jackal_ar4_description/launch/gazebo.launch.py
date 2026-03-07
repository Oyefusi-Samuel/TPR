"""
gazebo.launch.py  -  Jackal + AR4 in Gazebo Harmonic (ROS 2 Jazzy)

Mirrors jackal_ar4.launch exactly for the arm/gripper stack.
Adds Gazebo physics + wheel bridge on top.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    pkg_description  = get_package_share_directory('jackal_ar4_description')
    pkg_moveit       = get_package_share_directory('jackal_ar4_moveit_config')
    pkg_ar4_desc     = get_package_share_directory('ar4_description')
    pkg_clearpath    = get_package_share_directory('clearpath_platform_description')
    controllers_yaml = os.path.join(pkg_moveit, 'config', 'ros2_controllers.yaml')

    # Mesh paths so Gazebo resolves package:// URIs
    gz_env = {
        'GZ_SIM_RESOURCE_PATH': ':'.join([
            os.path.dirname(pkg_ar4_desc),
            os.path.dirname(pkg_clearpath),
            os.path.dirname(pkg_description),
        ]),
    }

    # ── Args ──────────────────────────────────────────────────────────────
    DeclareLaunchArgument('world', default_value='empty.sdf')
    launch_rviz_arg = DeclareLaunchArgument('launch_rviz', default_value='true')
    world_arg = DeclareLaunchArgument('world', default_value='empty.sdf')

    world       = LaunchConfiguration('world')
    launch_rviz = LaunchConfiguration('launch_rviz')

    # ── Robot description (is_sim:=true) ──────────────────────────────────
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
        cmd=['gz', 'sim', '-r', world],
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
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'jackal_ar4', '-topic', 'robot_description', '-z', '0.15'],
        output='screen',
    )

    # ── 4. ROS <-> Gazebo bridge ──────────────────────────────────────────
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

    # ── 5. Merge wheel joint states into /joint_states ────────────────────
    # ros2_control publishes arm joints to /joint_states
    # Gazebo publishes wheel joints to /wheel_joint_states
    # Relay merges them so RSP gets a complete robot state
    wheel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='wheel_state_relay',
        arguments=['/wheel_joint_states', '/joint_states'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ── 6. Static transform map -> odom ──────────────────────────────────
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--yaw', '0', '--pitch', '0', '--roll', '0',
                   '--frame-id', 'map', '--child-frame-id', 'odom'],
        parameters=[{'use_sim_time': True}],
    )

    # ── 7. Controller Manager (same as working launch) ────────────────────
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        parameters=[robot_description, controllers_yaml, {'use_sim_time': True}],
        output='screen',
    )

    # ── 8. Spawners (same as working launch) ──────────────────────────────
    def spawner(name, delay=3.0):
        return TimerAction(period=delay, actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name],
            parameters=[{'use_sim_time': True}],
            output='screen',
        )])

    spawn_jsb     = spawner('joint_state_broadcaster', delay=3.0)
    spawn_arm     = spawner('arm_controller',           delay=4.0)
    spawn_gripper = spawner('ar_gripper_controller',    delay=4.0)

    # ── 9. MoveIt move_group (same as working launch) ─────────────────────
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

    move_group = TimerAction(period=6.0, actions=[Node(
        package='moveit_ros_move_group',
        executable='move_group',
        parameters=[moveit_config.to_dict(), {'use_sim_time': True}],
        output='screen',
    )])

    # ── 10. RViz (same as working launch) ─────────────────────────────────
    rviz_config = os.path.join(pkg_moveit, 'config', 'moveit.rviz')
    if not os.path.exists(rviz_config):
        rviz_config = os.path.join(pkg_description, 'config', 'rviz_config.rviz')

    rviz = TimerAction(period=7.0, actions=[Node(
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
        spawn_robot,
        bridge,
        tf_static_bridge,
        wheel_relay,
        map_to_odom,
        controller_manager,
        spawn_jsb,
        spawn_arm,
        spawn_gripper,
        move_group,
        rviz,
    ])
