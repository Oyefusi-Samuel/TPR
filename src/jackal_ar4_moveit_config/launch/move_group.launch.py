import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    pkg_description = get_package_share_directory('jackal_ar4_description')

    moveit_config = (
        MoveItConfigsBuilder('jackal_ar4', package_name='jackal_ar4_moveit_config')
        .robot_description(file_path=os.path.join(pkg_description, 'urdf', 'jackal_ar4.urdf.xacro'), mappings={'use_platform_controllers': 'false'})
        .robot_description_semantic(file_path='config/jackal_ar4.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .planning_pipelines(pipelines=['ompl'], default_planning_pipeline='ompl')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .to_moveit_configs()
    )

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict()],
    )

    return LaunchDescription([move_group_node])
