"""Passive localization + optional timestamp-aware bridge + optional existing graph navigation."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory
import yaml


def setup(context):
    from ugv_relocalization.geometry import graph_pose
    from ugv_relocalization.core.frames import xyz_rpy
    share = Path(get_package_share_directory('ugv_relocalization'))
    def value(k):
        return LaunchConfiguration(k).perform(context)
    def boolean(k):
        return value(k).lower() == 'true'
    graph = yaml.safe_load((share/'config/graph.yaml').read_text())
    pin = xyz_rpy(graph_pose(graph, value('start_node'), value('facing_node')))
    common = {'use_sim_time': boolean('use_sim_time')}
    actions = []
    mode=value('mode')
    if mode not in ('rough','multistart','math_pin'):
        raise ValueError('mode must be rough, multistart, math_pin')
    # Original bridge ignores timestamps; disable graph injections unless the patched bridge is selected.
    if mode != 'math_pin' and value('calibration') and not boolean('launch_bridge'):
        raise ValueError('calibrated /initialpose requires launch_bridge:=true; stop the old bridge first')
    if boolean('launch_bridge'):
        actions.append(Node(package='ugv_relocalization', executable='pose_bridge', output='screen',
            parameters=[common, dict(start_x=float(pin[0]),start_y=float(pin[1]),start_yaw=float(pin[5]),
                extrinsic=[float(x) for x in value('extrinsic').split(',')])]))
    params = dict(common, mode=mode, map_path=value('map_path'), calibration=value('calibration'),
        start_node=value('start_node'), facing_node=value('facing_node'),
        extrinsic=[float(x) for x in value('extrinsic').split(',')],
        ref_start=[float(x) for x in value('ref_start').split(',')],
        extrinsic_measured=boolean('extrinsic_measured'), publish_initialpose=boolean('publish_initialpose'))
    actions.append(Node(package='ugv_relocalization', executable='relocalize', output='screen',
        parameters=[str(share/'config/robot.yaml'), params]))
    if boolean('launch_navigation'):
        import tempfile
        from ugv_relocalization.graph_config import export_graph
        nodes_csv, edges_csv = export_graph(graph, tempfile.mkdtemp(prefix='ugv_graph_'))
        nav = Path(get_package_share_directory('ugv_bringup'))/'launch/navigation_stack.launch.py'
        actions.append(GroupAction([SetParameter(name='use_sim_time', value=boolean('use_sim_time')),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(nav)), launch_arguments={
            'navigation_config': str(share/'config/navigation.yaml'),
            'nodes_csv':nodes_csv, 'edges_csv':edges_csv,
            'odom_topic':'/lego/odometry', 'start_node':value('start_node'), 'goal_node':value('goal_node'),
            'auto_plan_on_start':'false', 'launch_yolo':'false',
        }.items())]))
    return actions


def generate_launch_description():
    share=Path(get_package_share_directory('ugv_relocalization'))
    defaults={'mode':'rough','use_sim_time':'false', 'map_path':str(share/'maps/reference_map.pcd'),
        'calibration':'', 'launch_bridge':'false', 'launch_navigation':'false',
        'start_node':'A','facing_node':'B','goal_node':'I',
        'extrinsic':'0,0,0,0,0,0','extrinsic_measured':'false','ref_start':'0,0,0,0,0,0',
        'publish_initialpose':'true'}
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()]+[
        OpaqueFunction(function=setup)])
