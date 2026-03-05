import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    pkg_description = get_package_share_directory('jackal_ar4_description')
    pkg_moveit = get_package_share_directory('jackal_ar4_moveit_config')
    
    moveit_config = (
        MoveItConfigsBuilder('jackal_ar4', package_name='jackal_ar4_moveit_config')
        .robot_description(file_path=os.path.join(pkg_description, 'urdf', 'jackal_ar4.urdf.xacro'), mappings={'use_platform_controllers': 'false'})
        .robot_description_semantic(file_path='config/jackal_ar4.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    rviz_config = os.path.join(pkg_description, 'config', 'rviz_config.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([rviz_node])
