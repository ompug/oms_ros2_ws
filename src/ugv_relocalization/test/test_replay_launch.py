"""Launch test: rosbag2 -> scan/LeGO odom -> ICP -> /initialpose -> bridge.

Default fixture uses the committed map as a same-map synthetic scan, never field evidence.
RELOC_TEST_BAG selects a header-synchronized real bag (prepare_replay.py).
"""
import json
import os
from pathlib import Path
import tempfile
import time
import unittest
import numpy as np
import pytest
import yaml
import launch
import launch_testing
import launch_testing.markers
import launch_testing.actions
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from ugv_relocalization.core.io import read_pcd
from ugv_relocalization.geometry import ROS_FROM_CI
from ugv_relocalization.ros_common import xyz_to_cloud, ns_to_stamp


def fixture_bag(root, share):
    import rosbag2_py
    from rclpy.serialization import serialize_message
    bag=root/'bag'
    points=read_pcd(share/'maps/reference_map.pcd')[0]
    points=points[np.isfinite(points).all(axis=1)]
    points=points[np.linalg.norm(points,axis=1)<40.]
    points=points[np.random.default_rng(3).choice(len(points), min(8000,len(points)), replace=False)]
    cloud=points @ ROS_FROM_CI[:3,:3].T
    writer=rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
    for name,typ in [('/rslidar_points','sensor_msgs/msg/PointCloud2'),('/integrated_to_init','nav_msgs/msg/Odometry')]:
        writer.create_topic(rosbag2_py.TopicMetadata(name=name,type=typ,serialization_format='cdr'))
    for i in range(251):
        stamp=int((100+i*.1)*1e9)
        od=Odometry(); od.header.stamp=ns_to_stamp(stamp); od.header.frame_id='camera_init'
        od.pose.pose.orientation.w=1.
        writer.write('/integrated_to_init',serialize_message(od),stamp)
        if i%10==0:
            writer.write('/rslidar_points',serialize_message(xyz_to_cloud(cloud,'rslidar',stamp)),stamp)
    del writer
    return bag


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    global CAPTURE_PATH, BAG_LAST_NS
    share=Path(get_package_share_directory('ugv_relocalization'))
    root=Path(tempfile.mkdtemp(prefix='ugv_reloc_launch_'))
    CAPTURE_PATH=root/'captured.yaml'
    bag=Path(os.environ['RELOC_TEST_BAG']) if os.environ.get('RELOC_TEST_BAG') else fixture_bag(root,share)
    meta=yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    BAG_LAST_NS=meta['starting_time']['nanoseconds_since_epoch']+meta['duration']['nanoseconds']
    cal=root/'test_only_calibration.yaml'
    cal.write_text(yaml.safe_dump(dict(parent='map',child='camera_init_reference',T_graph_refmap=ROS_FROM_CI.tolist())))
    # This identity graph calibration and zero extrinsic are test fixtures, never robot calibration.
    reloc=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/relocalization.launch.py')),
        launch_arguments={'use_sim_time':'true','calibration':str(cal),'launch_bridge':'true',
            'extrinsic_measured':'true','launch_navigation':os.environ.get('RELOC_TEST_NAV','false'),
            'mode':os.environ.get('RELOC_TEST_MODE','rough')}.items())
    capture=Node(package='ugv_relocalization',executable='calibrate',arguments=[
        '--output',str(root/'captured.yaml'),'--extrinsic','0','0','0','0','0','0'],
        parameters=[{'use_sim_time':True}],output='screen')
    player=ExecuteProcess(cmd=['ros2','bag','play',str(bag),'--clock','100','--rate','1.0'],output='screen')
    return launch.LaunchDescription([GroupAction([
        SetRemap(src='/cmd_vel',dst='/test/cmd_vel'),
        SetRemap(src='/cmd_vel_raw',dst='/test/cmd_vel_raw'),reloc]),
        *([] if os.environ.get('RELOC_TEST_BAG') else [capture]),
        TimerAction(period=3.,actions=[player]),launch_testing.actions.ReadyToTest()]),dict(player=player, capture_path=root/'captured.yaml')



class TestReplay(unittest.TestCase):
    def test_pipeline(self):
        rclpy.init()
        n=rclpy.create_node('integration_observer')
        events=[]; bridge_events=[]; poses=[]; odom=[]
        n.create_subscription(String,'/reloc/status',lambda m:events.append(json.loads(m.data)),10)
        n.create_subscription(String,'/reloc/bridge_status',lambda m:bridge_events.append(json.loads(m.data)),10)
        n.create_subscription(PoseWithCovarianceStamped,'/initialpose',poses.append,10)
        n.create_subscription(Odometry,'/lego/odometry',odom.append,qos_profile_sensor_data)
        navigation=os.environ.get('RELOC_TEST_NAV')=='true'
        paths=[]; dense=[]; commands=[]; first=[]; blocked=None; stage=0
        if navigation:
            from ugv_msgs.msg import PlannedPath, BlockedEdge
            from geometry_msgs.msg import Twist
            from nav_msgs.msg import Path as NavPath
            n.create_subscription(PlannedPath,'/planned_path',paths.append,10)
            n.create_subscription(NavPath,'/dense_path',dense.append,qos_profile_sensor_data)
            n.create_subscription(Twist,'/test/cmd_vel_raw',commands.append,10)
            goals=n.create_publisher(String,'/goal_node',10)
            blocks=n.create_publisher(BlockedEdge,'/blocked_edge',10)
        try:
            deadline=time.monotonic()+65.
            while time.monotonic()<deadline:
                rclpy.spin_once(n,timeout_sec=.1)
                startup_ok=any(e.get('event')=='startup' and e.get('accepted') for e in events)
                bridge_ok=any(e.get('accepted') for e in bridge_events)
                correction_ok=any(e.get('event')=='correction' and e.get('accepted') for e in events)
                # Exercise navigation while bag /clock still advances, after startup is accepted.
                if navigation and startup_ok and bridge_ok:
                    if stage==0 and goals.get_subscription_count()>0 and blocks.get_subscription_count()>0:
                        goals.publish(String(data='I')); stage=1
                    elif stage==1 and paths and dense and commands:
                        first=list(paths[-1].edge_ids)
                        assert first
                        blocked=first[0]
                        blocks.publish(BlockedEdge(edge_id=blocked,source='isolated_replay_test')); stage=2
                    elif stage==2 and len(paths)>1 and blocked not in paths[-1].edge_ids:
                        stage=3
                end=bool(odom) and odom[-1].header.stamp.sec*1_000_000_000+odom[-1].header.stamp.nanosec >= BAG_LAST_NS-300_000_000
                if startup_ok and bridge_ok and correction_ok and end and (not navigation or stage==3):
                    break
            assert any(e.get('event')=='startup' and e.get('accepted') for e in events), events
            assert any(e.get('event')=='correction' and e.get('accepted') for e in events),events
            assert any(e.get('accepted') for e in bridge_events),bridge_events
            if not os.environ.get('RELOC_TEST_BAG'):
                capture_deadline=time.monotonic()+3.
                while not CAPTURE_PATH.exists() and time.monotonic()<capture_deadline:
                    rclpy.spin_once(n,timeout_sec=.1)
                assert CAPTURE_PATH.exists(), 'calibration capture did not complete'
                calibration=yaml.safe_load(CAPTURE_PATH.read_text())
                assert calibration['node']=='A' and calibration['facing']=='B'
                assert calibration['map_sha256']
            assert poses and all(p.header.frame_id=='map' for p in poses)
            assert odom and all(o.header.frame_id=='map' for o in odom)
            assert odom[-1].header.stamp.sec*1_000_000_000+odom[-1].header.stamp.nanosec >= BAG_LAST_NS-300_000_000, 'replay did not reach end of bag'
            for name in ('ugv_relocalization','lego_pose_bridge'):
                pubs=n.get_publisher_names_and_types_by_node(name,'/')
                assert not any(t in ('/cmd_vel','/cmd_vel_raw','/tf','/tf_static') for t,_ in pubs),pubs
            if navigation:
                assert stage==3, dict(stage=stage,paths=len(paths),dense=len(dense),commands=len(commands))
                assert list(paths[-1].edge_ids)!=first
                print('NAV_EVIDENCE',json.dumps(dict(first_edges=first,replanned_edges=list(paths[-1].edge_ids),
                    controller_messages=len(commands),dense_path_messages=len(dense))))
            print('REPLAY_EVIDENCE',json.dumps(dict(events=events,bridge=bridge_events,
                graph_fixes=len(poses),graph_odometry=len(odom))))
        finally:
            n.destroy_node();rclpy.shutdown()
