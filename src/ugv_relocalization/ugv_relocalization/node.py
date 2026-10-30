"""Passive relocalization: accepted reference poses and guarded graph /initialpose. No TF."""
import json
import hashlib
import os
import queue
import threading
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from .engine import Engine, load
from .geometry import (Coverage, ROS_FROM_CI, graph_pose, load_calibration, planar)
from .core.frames import pose_T, invert_T, xyz_rpy
from .core.gate import features_dict
from .runtime import (OdomPredictor, RuntimeLocalizer, SubmapRegistrars, RuntimeThresholds,
                      correction_size)
from .ros_common import cloud_to_xyz, stamp_to_ns, pose_to_T, make_pose_msg, pose_covariance


class Relocalize(Node):
    def __init__(self):
        super().__init__('ugv_relocalization')
        share = Path(get_package_share_directory('ugv_relocalization'))
        defaults = dict(mode='rough', map_path=str(share/'maps/reference_map.pcd'),
            config_dir=str(share/'config'), calibration='', scan_topic='/rslidar_points',
            odometry_topic='/integrated_to_init', period_s=5., max_correction_m=.25,
            max_correction_deg=2., max_startup_m=3., max_startup_deg=15.,
            ref_start=[0., 0., 0., 0., 0., 0.], start_node='A', facing_node='B',
            extrinsic=[0., 0., 0., 0., 0., 0.], extrinsic_measured=False,
            publish_initialpose=True, coverage_radius_m=5., coverage_min_points=30,
            max_fix_age_s=120., required_domain_id=42)
        self.p = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        p = self.p
        if os.environ.get('ROS_DOMAIN_ID') != str(p['required_domain_id']):
            raise RuntimeError('source scripts/robot_env.sh: ROS_DOMAIN_ID must match required_domain_id')
        if p['mode'] not in ('rough', 'multistart', 'math_pin'):
            raise ValueError('mode must be rough, multistart, or math_pin')
        self.status = self.create_publisher(String, '/reloc/status', 10)
        self.pose = self.create_publisher(PoseWithCovarianceStamped, '/reloc/pose', 10)
        self.correction = self.create_publisher(PoseWithCovarianceStamped, '/reloc/correction', 10)
        self.initial = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.cov = pose_covariance(.1, 2., 1.)
        self.predictor = OdomPredictor(240., 1., .15)
        self.T_base_lidar = pose_T(*p['extrinsic'])
        self.G = load_calibration(p['calibration']) @ invert_T(ROS_FROM_CI) if p['calibration'] else None
        if self.G is not None and not p['extrinsic_measured']:
            raise ValueError('graph publication requires measured base_link <- rslidar extrinsic')
        if p['calibration']:
            calibration = load(p['calibration'])
            if not np.allclose(calibration.get('extrinsic', p['extrinsic']), p['extrinsic']):
                raise ValueError('configured extrinsic differs from calibration extrinsic')
            if calibration.get('map_sha256') and calibration['map_sha256'] != hashlib.sha256(Path(p['map_path']).read_bytes()).hexdigest():
                raise ValueError('reference map differs from calibrated map')
            graph = load(Path(p['config_dir'])/'graph.yaml')
            for key in ('nodes', 'edges'):
                if calibration.get('graph_'+key+'_sha256', graph[key+'_sha256']) != graph[key+'_sha256']:
                    raise ValueError('navigation graph differs from calibrated graph')
        self.seed = pose_T(*p['ref_start'])
        self.start_graph = graph_pose(load(Path(p['config_dir'])/'graph.yaml'), p['start_node'], p['facing_node'])
        if self.G is not None:
            self.seed = invert_T(self.G) @ self.start_graph @ self.T_base_lidar
        self.last_job = None
        self.seen_inputs = set()
        self.jobs = queue.Queue(maxsize=1)
        self.busy = False
        self.epoch = 0
        self.create_subscription(Odometry, p['odometry_topic'], self.on_odom, qos_profile_sensor_data)
        if p['mode'] != 'math_pin':
            self.engine = Engine(p['config_dir'], p['map_path'])
            self.coverage = Coverage(self.engine.points, p['coverage_radius_m'], p['coverage_min_points'])
            regs = SubmapRegistrars(self.engine.ctx, self.engine.fine_cfg, self.engine.fine, 35., 10., 6, 2000)
            th = RuntimeThresholds(.5, 5., 5., False, p['max_correction_m'], p['max_correction_deg'])
            self.localizer = RuntimeLocalizer(self.engine.frames, self.engine.tree,
                self.engine.gate['features'], th, regs, self.predictor)
            self.create_subscription(PointCloud2, p['scan_topic'], self.on_scan, QoSProfile(depth=1))
            threading.Thread(target=self.worker, daemon=True).start()
        self.emit('ready', calibrated=self.G is not None, mode=p['mode'])

    def emit(self, event, **kw):
        if rclpy.ok(context=self.context):
            self.status.publish(String(data=json.dumps(dict(event=event, **kw))))

    def on_odom(self, msg):
        stamp = stamp_to_ns(msg.header.stamp)
        if 'odometry' not in self.seen_inputs:
            self.seen_inputs.add('odometry')
            self.emit('input_seen', topic=self.p['odometry_topic'], frame=msg.header.frame_id, stamp_ns=stamp)
        if msg.header.frame_id.lstrip('/') != 'camera_init':
            self.emit('odometry_rejected', reason='expected camera_init')
            return
        # LeGO camera pose -> ROS axes, exactly as the legacy pose bridge.
        T = ROS_FROM_CI @ pose_to_T(msg.pose.pose) @ invert_T(ROS_FROM_CI)
        if self.predictor.newest_ns is not None and stamp < self.predictor.newest_ns:
            self.epoch += 1
            self.predictor = OdomPredictor(240., 1., .15)
            if hasattr(self, 'localizer'):
                self.localizer.predictor = self.predictor
            self.last_job = None
            self.emit('clock_reset')
        self.predictor.add(stamp, T)

    def on_scan(self, msg):
        if 'scan' not in self.seen_inputs:
            self.seen_inputs.add('scan')
            self.emit('input_seen', topic=self.p['scan_topic'], frame=msg.header.frame_id, stamp_ns=stamp_to_ns(msg.header.stamp))
        if self.busy or msg.header.frame_id.lstrip('/') != 'rslidar':
            return
        stamp = stamp_to_ns(msg.header.stamp)
        if self.last_job is not None and stamp-self.last_job < self.p['period_s']*1e9:
            return
        try:
            xyz = cloud_to_xyz(msg)
        except ValueError as e:
            self.emit('scan_rejected', reason=str(e))
            return
        self.busy = True
        self.last_job = stamp
        self.jobs.put((self.epoch, stamp, xyz))

    def worker(self):
        while rclpy.ok():
            try:
                epoch, stamp, xyz = self.jobs.get(timeout=.2)
            except queue.Empty:
                continue
            try:
                self.process(epoch, stamp, xyz)
            except Exception as e:
                self.emit('registration_error', error=repr(e))
            finally:
                self.busy = False

    def process(self, epoch, stamp, xyz):
        started = time.monotonic()
        self.emit('registration_started', stamp_ns=stamp)
        predictor = self.predictor
        deadline = time.monotonic()+1.
        while (predictor.newest_ns is None or predictor.newest_ns < stamp) and time.monotonic()<deadline:
            time.sleep(.01)
        if predictor.odom_at(stamp) is None or (not predictor.ready and predictor.odom_at(stamp-2_000_000_000) is None):
            self.emit('odometry_missing', stamp_ns=stamp)
            return
        scan = self.engine.frames.scan_to_camera(xyz)
        startup = not predictor.ready
        if startup:
            result, T = self.engine.startup(scan, self.p['mode'], self.seed)
            accepted, reasons = result.accepted, list(result.reasons)
            details = dict(ambiguity_ratio=float(result.ambiguity),
                features=features_dict(result.final.features) if result.final else {},
                icp_match_ratio=result.final.icp.match_ratio if result.final else None)
            if accepted and self.p['mode'] == 'rough':
                d, a = correction_size(self.seed, T)
                if d > self.p['max_startup_m'] or a > self.p['max_startup_deg']:
                    accepted, reasons = False, ['startup_far_from_rough_placement']
            # Every restart remains bounded after a fix; no automatic global reinitialization while driving.
            if epoch != self.epoch:
                return
            if accepted:
                accepted = predictor.set_fix(stamp, T)
        else:
            prediction = predictor.predict(stamp)
            if prediction is None:
                self.emit('odometry_missing', stamp_ns=stamp)
                return
            if not self.coverage.contains(prediction[:2, 3]):
                self.emit('odometry_only', stamp_ns=stamp, reason='outside_map_coverage')
                return
            rec = self.localizer.attempt(scan, stamp)
            if rec is None:
                return
            accepted, reasons, T = rec['accepted'], rec['reasons'], rec['T_map_lidar']
            details = dict(features=rec['features'], correction_xy_m=rec['correction_xy_m'],
                correction_yaw_deg=rec['correction_yaw_deg'])
        if epoch != self.epoch:
            return
        self.emit('startup' if startup else 'correction', stamp_ns=stamp,
                  accepted=bool(accepted), reasons=reasons, duration_s=time.monotonic()-started,
                  candidate_ref_xyzrpy=xyz_rpy(T).tolist() if T is not None else None, **details)
        if not accepted:
            return
        (self.pose if startup else self.correction).publish(make_pose_msg(T, 'refmap_ros', stamp, self.cov))
        if self.G is None or not self.p['publish_initialpose']:
            return
        # Bridge uses history at this stamp, so processing delay cannot move the pin to the wrong instant.
        if (predictor.newest_ns-stamp)/1e9 > self.p['max_fix_age_s']:
            self.emit('graph_fix_rejected', reason='stale')
            return
        base = planar(self.G @ T @ invert_T(self.T_base_lidar))
        self.initial.publish(make_pose_msg(base, 'map', stamp, self.cov))


def main():
    rclpy.init()
    n = Relocalize()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
