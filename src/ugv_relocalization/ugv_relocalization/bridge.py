#!/usr/bin/env python3
"""Modified from ~/previous_agx_work/lego_pose_bridge.py (read-only source).

Timestamped pin history, graph-frame validation and correction jump rejection.
Owns /lego/odometry only; no TF or drive publisher. Math-pin remains the fallback.
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from .runtime import OdomPredictor
from .ros_common import stamp_to_ns
from .geometry import planar
from .core.frames import pose_T
from .runtime import correction_size

# camera_init(optical) -> ROS: ros_x=cam_z, ros_y=cam_x, ros_z=cam_y
R_PERM = np.array([[0.0, 0.0, 1.0],
                   [1.0, 0.0, 0.0],
                   [0.0, 1.0, 0.0]])

def quat_to_R(x, y, z, w):
    n = math.sqrt(x*x + y*y + z*z + w*w) or 1.0
    x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)],
    ])

def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw/2.0), math.cos(yaw/2.0))

def se2(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, x], [s, c, y], [0, 0, 1.0]])

class LegoBridge(Node):
    def __init__(self):
        super().__init__("lego_pose_bridge")
        self.declare_parameter("mirror", False)
        self.mirror = self.get_parameter("mirror").value
        self.extrinsic = pose_T(*self.declare_parameter("extrinsic", [0., 0., 0., 0., 0., 0.]).value)
        # MATH PIN (PhD-style deterministic origin): if the robot boots at a KNOWN
        # graph pose (place at node D facing __F_G_5), set the pin analytically from
        # the graph coords + bearing -- no rviz drag, transform exact by construction.
        # /initialpose (rviz 2D Pose Estimate) still overrides at runtime if needed.
        self.declare_parameter("start_x", 0.0)
        self.declare_parameter("start_y", 0.0)
        self.declare_parameter("start_yaw", 0.0)
        sx = float(self.get_parameter("start_x").value)
        sy = float(self.get_parameter("start_y").value)
        syaw = float(self.get_parameter("start_yaw").value)
        self.history = OdomPredictor(240., 1., 0.)
        self.first_fix = True
        self.max_jump_m = self.declare_parameter("max_jump_m", 0.25).value
        self.max_jump_deg = self.declare_parameter("max_jump_deg", 2.0).value
        self.events = self.create_publisher(String, "/reloc/bridge_status", 10)
        self.pin = se2(sx, sy, syaw)           # map <- lego_raw  (boot-pose pin)
        self.get_logger().info(f"start pin = ({sx:.2f},{sy:.2f},{math.degrees(syaw):.1f}deg)")
        self.pub = self.create_publisher(Odometry, "/lego/odometry", qos_profile_sensor_data)
        self.create_subscription(Odometry, "/integrated_to_init",
                                 self.cb, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose",
                                 self.cb_pin, 10)
        self.get_logger().info("lego_pose_bridge up: /integrated_to_init -> /lego/odometry (map)")

    def raw_pose(self, msg):
        """LeGO camera_init pose -> (x, y, yaw) in the un-pinned ROS frame."""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        p_ros = R_PERM @ np.array([p.x, p.y, p.z])
        R_ros = R_PERM @ quat_to_R(q.x, q.y, q.z, q.w) @ R_PERM.T
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = R_ros, p_ros
        # E @ relative_lidar_motion @ inv(E) is base motion relative to the boot base.
        T = self.extrinsic @ T @ np.linalg.inv(self.extrinsic)
        x, y = float(T[0, 3]), float(T[1, 3])
        yaw = math.atan2(T[1, 0], T[0, 0])
        if self.mirror:
            y = -y
            yaw = -yaw
        return x, y, yaw

    def cb_pin(self, msg):
        """Solve pin at measurement time, never against the latest moving pose."""
        import json
        reason = None
        d = a = None
        stamp = stamp_to_ns(msg.header.stamp)
        raw4 = self.history.odom_at(stamp)
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        vals = [p.x, p.y, q.x, q.y, q.z, q.w]
        if msg.header.frame_id != "map" or not np.isfinite(vals).all():
            reason = "invalid_frame_or_pose"
        elif raw4 is None or self._last_raw is None:
            reason = "no_odometry_at_fix_stamp"
        else:
            cyaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
            raw = se2(raw4[0, 3], raw4[1, 3], math.atan2(raw4[1, 0], raw4[0, 0]))
            candidate = se2(p.x, p.y, cyaw) @ np.linalg.inv(raw)
            current = self.pin @ se2(*self._last_raw)
            proposed = candidate @ se2(*self._last_raw)
            d = np.linalg.norm(current[:2, 2]-proposed[:2, 2])
            delta_yaw = math.atan2(proposed[1, 0], proposed[0, 0]) - math.atan2(current[1, 0], current[0, 0])
            a = abs(math.degrees(math.atan2(math.sin(delta_yaw), math.cos(delta_yaw))))
            if self.first_fix and (d > self.max_jump_m or a > self.max_jump_deg):
                now = self.history.newest_ns
                before = self.history.odom_at(now-int(2e9))
                latest = self.history.odom_at(now)
                if before is None or correction_size(before, latest)[0] > .05 or correction_size(before, latest)[1] > 2.:
                    reason = "first_fix_requires_two_seconds_stationary"
            elif d > self.max_jump_m or a > self.max_jump_deg:
                reason = "correction_jump_rejected"
            if reason is None:
                self.pin = candidate
                self.first_fix = False
        self.events.publish(String(data=json.dumps({"accepted": reason is None,
            "reason": reason, "stamp_ns": stamp, "delta_xy_m": float(d) if d is not None else None,
            "delta_yaw_deg": a})))

    _last_raw = None
    def cb(self, msg):
        rx, ry, ryaw = self.raw_pose(msg)
        stamp = stamp_to_ns(msg.header.stamp)
        if self.history.newest_ns is not None and stamp < self.history.newest_ns:
            self.history = OdomPredictor(240., 1., 0.)
            self.first_fix = True
        self.history.add(stamp, pose_T(rx, ry, 0., yaw=ryaw))
        self._last_raw = (rx, ry, ryaw)
        M = self.pin @ se2(rx, ry, ryaw)
        x, y = float(M[0, 2]), float(M[1, 2])
        yaw = math.atan2(M[1, 0], M[0, 0])
        qx, qy, qz, qw = yaw_to_quat(yaw)

        od = Odometry()
        od.header.stamp = msg.header.stamp
        od.header.frame_id = "map"
        od.child_frame_id = "base_link"
        od.pose.pose.position.x = x
        od.pose.pose.position.y = y
        od.pose.pose.orientation.x = qx
        od.pose.pose.orientation.y = qy
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw
        self.pub.publish(od)

def main():
    rclpy.init()
    n = LegoBridge()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
