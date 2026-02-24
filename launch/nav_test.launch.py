import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
  DeclareLaunchArgument,
  IncludeLaunchDescription)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
  # Set the path to this package.
  pkg_path = get_package_share_directory('a_star_smooth_planner')
 
  # Set the path to the map file
  map_file_name = 'room_with_walls.yaml'
  keepout_file_name = 'keepout_mask.yaml'
  map_yaml_path = os.path.join(pkg_path, 'maps', map_file_name)
  keepout_yaml_path = os.path.join(pkg_path,'maps',keepout_file_name)

  # Set the path to the nav params file
  nav_params_file_name = 'nav2_bringup_params.yaml'
  nav_params_file = os.path.join(pkg_path, 'config', nav_params_file_name)
 
  #--------------------------------------------------------------------------

  # Launch configuration variables specific to simulation
  use_sim_time = LaunchConfiguration('use_sim_time')
  map = LaunchConfiguration('map')
  keepout_map = LaunchConfiguration('keepout_map')
  params_file = LaunchConfiguration('params_file')
     
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='True',
    description='Use simulation (Gazebo) clock if true')
  
  declare_map_cmd = DeclareLaunchArgument(
      name='map',
      default_value=map_yaml_path,
      description='file path to the map needed for navigation')
  
  declare_keepout_cmd = DeclareLaunchArgument(
    name='keepout_map',
    default_value=keepout_yaml_path,
    description= "file path to the keepout filter mask"
  )
  
  declare_params_file_cmd = DeclareLaunchArgument(
      name='params_file',
      default_value=nav_params_file,
      description='file path to the navigation paramater file needed for navigation')

  #-----------------------------------------------------------------------------
  rviz_config_file = os.path.join(pkg_path,'config','amcl.rviz')


  # create needed nodes or launch files
  rviz_node = Node(
      package='rviz2',
      executable='rviz2',
      arguments=['-d', rviz_config_file],
      output='screen'
  )

  a_star_planner_node = Node(
    package='a_star_smooth_planner',
    executable='a_star_planner.py',
    name='a_star_planner',
    output='screen',
  )

  a_star_smoothner_node = Node(
    package='a_star_smooth_planner',
    executable='a_star_smoother',
    name='a_star_smoother',
    output='screen',
    parameters=[{'iterations': 5,
                 'cost_limit': 20
                 }],
  )

  pure_pursuit_node = Node(
    package='a_star_smooth_planner',
    executable='pure_pursuit',
    name='pure_pursuit',
    output='screen',
    parameters=[{'look_ahead_distance': 0.3,
                 'max_linear_velocity': 0.2,
                 'max_angular_velocity': 1.0,
                 'path_topic': '/a_star/path/smooth'
                 }],
    remappings=[('/cmd_vel', '/cmd_vel_nav')],
  )

  lifecycle_nodes = [
    'map_server',
    'amcl',
    'costmap',
    'keepout_filter_mask_server',
    'costmap_filter_info_server'
  ]

  remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

  nav2_map_server_node = Node(
    package='nav2_map_server',
    executable='map_server',
    name='map_server',
    output='screen',
    parameters=[params_file, {'yaml_filename': map}],
    remappings=remappings,
  )
  # 1. The Map Server for the Keepout Mask
  keepout_map_server = Node(
    package='nav2_map_server',
    executable='map_server',
    name='keepout_filter_mask_server',
    output='screen',
    parameters=[params_file, {'yaml_filename': keepout_map}],
    remappings=[('/map', '/keepout_filter_mask')] # Remap to the mask topic
  )

  # 2. The Filter Info Server (Tells Nav2 this is a "Keepout" type filter)
  filter_info_server = Node(
    package='nav2_map_server',
    executable='costmap_filter_info_server',
    name='costmap_filter_info_server',
    output='screen',
    parameters=[params_file],
    remappings=[('/map', '/keepout_filter_mask')]
  )

  nav2_costmap_2d_node = Node(
    package='nav2_costmap_2d',
    executable='nav2_costmap_2d',
    name='costmap',
    output='screen',
    parameters=[params_file],
  )

  nav2_amcl_node = Node(
    package='nav2_amcl',
    executable='amcl',
    name='amcl',
    output='screen',
    parameters=[params_file],
    remappings=remappings,
  )

  nav2_lifecycle_manager_node = Node(
    package='nav2_lifecycle_manager',
    executable='lifecycle_manager',
    output='screen',
    parameters=[{"autostart": True, "bond_timeout": 0.0}, {'node_names': lifecycle_nodes}],
  )

  obstacle_generator_node = Node(
    package='a_star_smooth_planner',
    executable='obstacle_generator.py',
    name='obstacle_generator',
    output='screen'
  )

  #--------------------------------------------------------------------------------

  # Create the launch description
  ld = LaunchDescription()
 
  # add the necessary declared launch arguments to the launch description
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_map_cmd)
  ld.add_action(declare_keepout_cmd)
  ld.add_action(declare_params_file_cmd)
 
  # Add the nodes to the launch description
  ld.add_action(rviz_node)
  ld.add_action(a_star_planner_node)
  ld.add_action(a_star_smoothner_node)
  ld.add_action(pure_pursuit_node)
  ld.add_action(nav2_map_server_node)
  ld.add_action(keepout_map_server)
  ld.add_action(filter_info_server)
  ld.add_action(nav2_costmap_2d_node)
  ld.add_action(nav2_amcl_node)
  ld.add_action(nav2_lifecycle_manager_node)
  ld.add_action(obstacle_generator_node)

  return ld
