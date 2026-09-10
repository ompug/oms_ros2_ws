import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

  # Configure environment
  stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')
  stdout_colorized_envvar = SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1')

  # Nodes Configurations
  config_file = os.path.join(get_package_share_directory('lego_loam_sr'), 'config', 'loam_config.yaml')
  rviz_config = os.path.join(get_package_share_directory('lego_loam_sr'), 'rviz', 'test.rviz')

  points_topic = LaunchConfiguration('points_topic')
  params_file = LaunchConfiguration('params_file')
  use_sim_time = LaunchConfiguration('use_sim_time')
  rviz = LaunchConfiguration('rviz')
  publish_reference_tf = LaunchConfiguration('publish_reference_tf')

  # Tf transformations
  transform_map = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='camera_init_to_map',
    arguments=[
      '--x', '0', '--y', '0', '--z', '0',
      '--yaw', '1.570795', '--pitch', '0', '--roll', '1.570795',
      '--frame-id', 'map', '--child-frame-id', 'camera_init'],
    parameters=[{'use_sim_time': use_sim_time}],
    condition=IfCondition(publish_reference_tf),
  )

  transform_camera = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_link_to_camera',
    arguments=[
      '--x', '0', '--y', '0', '--z', '0',
      '--yaw', '-1.570795', '--pitch', '-1.570795', '--roll', '0',
      '--frame-id', 'camera', '--child-frame-id', 'base_link'],
    parameters=[{'use_sim_time': use_sim_time}],
    condition=IfCondition(publish_reference_tf),
  )

  transform_velodyne = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='velodyne_to_base_link',
    arguments=[
      '--x', '0', '--y', '0', '--z', '0',
      '--yaw', '0', '--pitch', '0', '--roll', '0',
      '--frame-id', 'base_link', '--child-frame-id', 'velodyne'],
    parameters=[{'use_sim_time': use_sim_time}],
    condition=IfCondition(publish_reference_tf),
  )

  # LeGO-LOAM
  lego_loam_node = Node(
    package='lego_loam_sr',
    executable='lego_loam_sr',
    output='screen',
    parameters=[params_file, {'use_sim_time': use_sim_time}],
    remappings=[('/lidar_points', points_topic)],
  )

  # Rviz
  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', rviz_config],
    parameters=[{'use_sim_time': use_sim_time}],
    output='screen',
    condition=IfCondition(rviz),
  )

  ld = LaunchDescription()
  # Set environment variables
  ld.add_action(stdout_linebuf_envvar)
  ld.add_action(stdout_colorized_envvar)
  ld.add_action(DeclareLaunchArgument(
    'points_topic', default_value='/velodyne_points',
    description='External PointCloud2 input topic'))
  ld.add_action(DeclareLaunchArgument(
    'params_file', default_value=config_file,
    description='LeGO-LOAM parameter YAML file'))
  ld.add_action(DeclareLaunchArgument(
    'use_sim_time', default_value='true',
    description='Use /clock from rosbag2 or a test publisher'))
  ld.add_action(DeclareLaunchArgument(
    'rviz', default_value='true',
    description='Start RViz with the installed project configuration'))
  ld.add_action(DeclareLaunchArgument(
    'publish_reference_tf', default_value='true',
    description='Publish reference-only map/camera/base/velodyne static transforms'))
  # Add nodes
  ld.add_action(lego_loam_node)
  ld.add_action(transform_map)
  ld.add_action(transform_camera)
  ld.add_action(transform_velodyne)
  ld.add_action(rviz_node)

  return ld
