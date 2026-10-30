import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from ugv_relocalization.geometry import (calibration_from_pose, ROS_FROM_CI, Coverage,
                                         validate_transform, graph_pose)
from ugv_relocalization.core.frames import pose_T, invert_T
from ugv_relocalization.runtime import OdomPredictor, RuntimeThresholds, gate_correction
from ugv_relocalization.core.gate import GateFeatures
from ugv_relocalization.bridge import LegoBridge
from ugv_relocalization.ros_common import make_pose_msg, pose_covariance


def test_calibration_recovers_axes_and_lever_arm():
    G=pose_T(11.,-8.,0.,yaw=.6)
    R=pose_T(3.,2.,1.,yaw=-.4)
    E=pose_T(.4,.1,.8,yaw=.2)
    target=G @ R @ invert_T(E)
    T=calibration_from_pose(target,R,E)
    # Calibration is planar; vertical graph placement is intentionally discarded.
    assert np.allclose(T[:2], (G @ ROS_FROM_CI)[:2])
    assert np.allclose((T @ invert_T(ROS_FROM_CI) @ R @ invert_T(E))[:2,3],target[:2,3])


def test_graph_bearing_uses_coordinates_not_csv_yaw():
    T=graph_pose({'nodes':{'A':[0.,0.],'B':[4.,0.]}},'A','B')
    assert np.allclose(T,np.eye(4))


def test_invalid_calibration_fails_closed():
    with pytest.raises(ValueError):
        validate_transform(np.diag([1,1,-1,1]))


def test_prediction_single_sample_and_no_long_gap():
    p=OdomPredictor(100.,1.,.1)
    p.add(10_000_000_000,np.eye(4))
    assert np.allclose(p.odom_at(10_000_000_000),np.eye(4))
    assert p.set_fix(10_000_000_000,pose_T(2,3,0))
    p.add(13_000_000_000,pose_T(3,0,0))
    assert p.odom_at(12_000_000_000) is None
    assert p.odom_at(14_000_000_000) is None


def test_coverage_does_not_fill_empty_map_holes():
    c=Coverage(np.random.default_rng(1).normal(0,.1,(100,3)),radius=1.,min_points=30)
    assert c.contains([0,0])
    assert not c.contains([10,0])


def test_bridge_delayed_fix_and_jump_guard():
    rclpy.init()
    b=LegoBridge()
    try:
        # Zero motion for 3s; first fix may change the math-pin while stationary.
        for i in range(31):
            msg=Odometry(); msg.header.stamp.sec=10+i//10; msg.header.stamp.nanosec=(i%10)*100_000_000
            msg.pose.pose.orientation.w=1.
            b.cb(msg)
        b.cb_pin(make_pose_msg(pose_T(1,0,0),'map',11_000_000_000,pose_covariance(.1,1,1)))
        assert np.isclose(b.pin[0,2],1.)
        # Robot has moved 1m by callback time; delayed fix is still at x=1.1 at t=13.
        for i in range(1,11):
            msg=Odometry(); msg.header.stamp.sec=13; msg.header.stamp.nanosec=i*90_000_000
            msg.pose.pose.position.z=i/10.; msg.pose.pose.orientation.w=1.; b.cb(msg)
        b.cb_pin(make_pose_msg(pose_T(1.1,0,0),'map',13_000_000_000,pose_covariance(.1,1,1)))
        assert np.isclose(b.pin[0,2],1.1)
        b.cb_pin(make_pose_msg(pose_T(50,0,0),'map',13_000_000_000,pose_covariance(.1,1,1)))
        assert np.isclose(b.pin[0,2],1.1)
        b.cb_pin(make_pose_msg(pose_T(1.2,0,0),'wrong',13_000_000_000,pose_covariance(.1,1,1)))
        assert np.isclose(b.pin[0,2],1.1)
    finally:
        b.destroy_node(); rclpy.shutdown()


def test_runtime_jump_rejection_even_with_perfect_structure():
    f=GateFeatures(1.,1.,1.,0.,0.,0.,True,1000)
    th=RuntimeThresholds(.5,5.,5.,False,.25,2.)
    assert gate_correction(f,True,False,.1,1.,th)==(True,[])
    assert gate_correction(f,True,False,2.,1.,th)==(False,['correction_xy_too_large'])
    assert gate_correction(f,True,False,.1,10.,th)==(False,['correction_yaw_too_large'])


def test_bridge_conjugates_lidar_lever_arm():
    rclpy.init()
    b=LegoBridge()
    try:
        b.extrinsic=pose_T(.4,0,.8)
        # Rotation about a stationary base displaces the offset LiDAR.
        base=pose_T(0,0,0,yaw=.3)
        lidar=invert_T(b.extrinsic) @ base @ b.extrinsic
        camera=invert_T(ROS_FROM_CI) @ lidar @ ROS_FROM_CI
        from ugv_relocalization.ros_common import T_to_pose
        msg=Odometry();msg.pose.pose=T_to_pose(camera)
        x,y,yaw=b.raw_pose(msg)
        assert np.allclose([x,y,yaw],[0.,0.,.3])
    finally:
        b.destroy_node();rclpy.shutdown()
