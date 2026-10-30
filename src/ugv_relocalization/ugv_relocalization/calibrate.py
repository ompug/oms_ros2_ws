"""Capture one accepted stationary ICP pose while math-pinned at a surveyed graph pose."""
import argparse
import hashlib
from pathlib import Path
import time
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory
from .geometry import graph_pose, calibration_from_pose, planar
from .core.frames import pose_T
from .ros_common import pose_to_T, stamp_to_ns
from .runtime import correction_size


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--map-path', default='')
    parser.add_argument('--node', default='A')
    parser.add_argument('--facing', default='B')
    parser.add_argument('--extrinsic', nargs=6, type=float, required=True,
                        help='Measured base_link <- rslidar: x y z roll pitch yaw')
    parser.add_argument('--timeout', type=float, default=180.)
    args, ros = parser.parse_known_args()
    share = Path(get_package_share_directory('ugv_relocalization'))
    graph = yaml.safe_load((share/'config/graph.yaml').read_text())
    target = graph_pose(graph, args.node, args.facing)
    rclpy.init(args=ros)
    n = Node('calibrate_graph_refmap')
    captured, odom = [], []
    n.create_subscription(PoseWithCovarianceStamped, '/reloc/pose', captured.append, 10)
    def on_odom(msg):
        odom.append(msg)
        del odom[:-500]
    n.create_subscription(Odometry, '/lego/odometry', on_odom, qos_profile_sensor_data)
    deadline = time.monotonic()+args.timeout
    try:
        while time.monotonic()<deadline and rclpy.ok():
            rclpy.spin_once(n, timeout_sec=.2)
            if not captured or len(odom)<2:
                continue
            fix = captured[-1]
            if fix.header.frame_id != 'refmap_ros':
                raise ValueError('unexpected reference pose frame')
            stamp = stamp_to_ns(fix.header.stamp)
            near = min(odom, key=lambda m: abs(stamp_to_ns(m.header.stamp)-stamp))
            if abs(stamp_to_ns(near.header.stamp)-stamp)>200_000_000:
                continue
            current = pose_to_T(near.pose.pose)
            d, a = correction_size(target, current)
            if d>.05 or a>1.:
                raise ValueError('bridge is not math-pinned at the requested exact graph pose')
            recent = [m for m in odom if stamp-2_000_000_000 <= stamp_to_ns(m.header.stamp)<=stamp]
            if not recent or stamp-stamp_to_ns(recent[0].header.stamp)<1_800_000_000:
                continue
            if any(correction_size(current, pose_to_T(m.pose.pose))[0]>.03 or
                   correction_size(current, pose_to_T(m.pose.pose))[1]>1. for m in recent):
                raise ValueError('robot moved during calibration')
            T = calibration_from_pose(target, pose_to_T(fix.pose.pose), pose_T(*args.extrinsic))
            out = dict(parent='map', child='camera_init_reference', T_graph_refmap=T.tolist(),
                       map_sha256=hashlib.sha256(Path(args.map_path or share/'maps/reference_map.pcd').read_bytes()).hexdigest(),
                       graph_nodes_sha256=graph['nodes_sha256'], graph_edges_sha256=graph['edges_sha256'],
                       node=args.node, facing=args.facing, extrinsic=args.extrinsic,
                       stamp_ns=stamp, method='stationary math-pin plus accepted startup ICP')
            path=Path(args.output)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(yaml.safe_dump(out, sort_keys=False))
            print(f'Saved {path}: T_graph_refmap (raw LeGO camera_init -> graph map)')
            return
        raise TimeoutError('no accepted stationary startup pose; restart ICP after starting capture')
    finally:
        n.destroy_node()
        rclpy.shutdown()
