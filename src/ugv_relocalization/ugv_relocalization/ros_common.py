"""ROS 2 helpers shared by the E7 nodes: message <-> numpy conversion and parameter loading.

Pure Python plus message classes; nothing here spins a node, so tests can call it directly.
Poses are 4x4 float64 (see ``relocalization.frames``): ``T_A_B`` maps points in B into A.
"""
from __future__ import annotations

import json
import os
import sys
from typing import Any, Iterable, Mapping

import numpy as np
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from sensor_msgs.msg import PointField, PointCloud2

from .core.frames import Frames, T_from_xyz_quat, quat_xyzw

_FIELD_DTYPES = {PointField.INT8: "i1", PointField.UINT8: "u1", PointField.INT16: "i2",
                 PointField.UINT16: "u2", PointField.INT32: "i4", PointField.UINT32: "u4",
                 PointField.FLOAT32: "f4", PointField.FLOAT64: "f8"}
INTENSITY_FIELDS = ("x", "y", "z", "intensity")
ODOMETRY_TYPES = ("ros", "lego", "none")


def stamp_to_ns(stamp: Time) -> int:
    """builtin_interfaces/Time -> integer nanoseconds."""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def ns_to_stamp(ns: int) -> Time:
    """Integer nanoseconds -> builtin_interfaces/Time."""
    return Time(sec=int(ns // 1_000_000_000), nanosec=int(ns % 1_000_000_000))


def cloud_to_xyz(msg: PointCloud2) -> np.ndarray:
    """(N, 3) float64 x, y, z of a PointCloud2 with any field layout (NaN rows are kept)."""
    names = [f.name for f in msg.fields]
    missing = [n for n in ("x", "y", "z") if n not in names]
    if missing:
        raise ValueError(f"PointCloud2 has no field(s) {missing}")
    endian = ">" if msg.is_bigendian else "<"
    dtype = np.dtype({"names": names, "offsets": [f.offset for f in msg.fields],
                      "formats": [endian + _FIELD_DTYPES[f.datatype] for f in msg.fields],
                      "itemsize": msg.point_step})
    n = int(msg.width) * int(msg.height)
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if msg.row_step != msg.width * msg.point_step:  # padded rows
        raw = raw.reshape(msg.height, msg.row_step)[:, : msg.width * msg.point_step].reshape(-1)
    points = np.frombuffer(raw[: n * msg.point_step].tobytes(), dtype=dtype, count=n)
    return np.column_stack([points[a].astype(np.float64) for a in ("x", "y", "z")])


def xyz_to_cloud(xyz: np.ndarray, frame_id: str, stamp_ns: int,
                 intensity: np.ndarray | float = 0.0) -> PointCloud2:
    """x y z intensity float32 PointCloud2 (unorganised) from an (N, 3) array."""
    xyz = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
    data = np.empty(len(xyz), dtype=[(n, "<f4") for n in INTENSITY_FIELDS])
    for i, name in enumerate(INTENSITY_FIELDS[:3]):
        data[name] = xyz[:, i]
    data["intensity"] = intensity
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp = ns_to_stamp(stamp_ns)
    msg.height, msg.width = 1, len(xyz)
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(INTENSITY_FIELDS)]
    msg.is_bigendian, msg.point_step = False, 4 * len(INTENSITY_FIELDS)
    msg.row_step, msg.is_dense = msg.point_step * len(xyz), True
    msg.data = data.tobytes()
    return msg


def T_to_pose(T: np.ndarray) -> Pose:
    """4x4 pose -> geometry_msgs/Pose."""
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in T[:3, 3])
    q = quat_xyzw(T)
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (float(v) for v in q)
    return pose


def pose_to_T(pose: Pose) -> np.ndarray:
    """geometry_msgs/Pose -> 4x4 pose."""
    p, q = pose.position, pose.orientation
    return T_from_xyz_quat(np.array([p.x, p.y, p.z]), np.array([q.x, q.y, q.z, q.w]))


def pose_covariance(pos_std_m: float, roll_pitch_std_deg: float, yaw_std_deg: float) -> list[float]:
    """Row-major 6x6 covariance (x, y, z, roll, pitch, yaw) with the given 1-sigma values."""
    cov = np.zeros((6, 6))
    var = [pos_std_m ** 2] * 3 + [np.radians(roll_pitch_std_deg) ** 2] * 2 + [np.radians(yaw_std_deg) ** 2]
    cov[np.arange(6), np.arange(6)] = var
    return [float(v) for v in cov.reshape(-1)]


def make_pose_msg(T: np.ndarray, frame_id: str, stamp_ns: int,
                  covariance: Iterable[float]) -> PoseWithCovarianceStamped:
    """PoseWithCovarianceStamped for ``T`` in ``frame_id``."""
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = ns_to_stamp(stamp_ns)
    msg.pose.pose = T_to_pose(T)
    msg.pose.covariance = list(covariance)
    return msg
