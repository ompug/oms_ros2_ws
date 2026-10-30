"""Rigid-transform helpers and the LeGO-LOAM frame conventions.

Notation: ``T_A_B`` is a 4x4 homogeneous matrix that maps points in frame B
into frame A, i.e. the pose of B expressed in A.

Two pose representations are used:

* ``T_ci_cam`` -- camera_init <- camera. This is what ICP estimates (scans
  are converted to LeGO camera axes first) and what LeGO publishes on
  ``/aft_mapped_to_init``. The vertical axis is camera_init **Y**.
* ``T_map_lidar`` -- map <- rslidar, with map = camera_init in ROS axes
  (z up). Used for every human-readable x, y, z, roll, pitch, yaw. It is a
  LiDAR pose: T_base_link_rslidar is not measured yet.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

import numpy as np
from scipy.spatial.transform import Rotation


def make_T(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Build a 4x4 homogeneous transform from a 3x3 rotation and a 3-vector."""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def invert_T(T: np.ndarray) -> np.ndarray:
    """Invert a rigid transform without a general matrix inverse."""
    R = T[:3, :3]
    return make_T(R.T, -R.T @ T[:3, 3])


def transform_points(T: np.ndarray, points: np.ndarray) -> np.ndarray:
    """Apply ``T`` to an (N, 3) array of points."""
    return points @ T[:3, :3].T + T[:3, 3]


def pose_T(x: float, y: float, z: float, roll: float = 0.0, pitch: float = 0.0,
           yaw: float = 0.0) -> np.ndarray:
    """Pose from position and ROS roll/pitch/yaw (radians, R = Rz Ry Rx)."""
    R = Rotation.from_euler("xyz", [roll, pitch, yaw]).as_matrix()
    return make_T(R, np.array([x, y, z], dtype=float))


def xyz_rpy(T: np.ndarray) -> np.ndarray:
    """Return [x, y, z, roll, pitch, yaw] (radians) of a pose, ROS convention."""
    rpy = Rotation.from_matrix(np.array(T[:3, :3])).as_euler("xyz")
    return np.concatenate([T[:3, 3], rpy])


def quat_xyzw(T: np.ndarray) -> np.ndarray:
    """Return the rotation of ``T`` as a unit quaternion [qx, qy, qz, qw]."""
    return Rotation.from_matrix(np.array(T[:3, :3])).as_quat()


def T_from_xyz_quat(xyz: np.ndarray, quat_xyzw: np.ndarray) -> np.ndarray:
    """Pose from a translation and a quaternion [qx, qy, qz, qw]."""
    return make_T(Rotation.from_quat(quat_xyzw).as_matrix(), np.asarray(xyz, float))


def wrap_deg(angle: float | np.ndarray) -> float | np.ndarray:
    """Wrap degrees into [-180, 180)."""
    return (np.asarray(angle) + 180.0) % 360.0 - 180.0


def rotation_angle_deg(R: np.ndarray) -> float:
    """Geodesic angle of a rotation matrix, in degrees."""
    return float(np.degrees(Rotation.from_matrix(np.array(R)).magnitude()))


@dataclass(frozen=True)
class Frames:
    """Fixed rotations between rslidar, LeGO camera, camera_init and map."""

    camera_from_rslidar: np.ndarray
    map_from_camera_init: np.ndarray
    pcd_frame_id: str
    pose_label: str

    @classmethod
    def from_config(cls, cfg: Mapping[str, Any]) -> "Frames":
        """Build from the ``frames.yaml`` mapping."""
        yaw, pitch, roll = cfg["map_from_camera_init_euler_zyx_rad"]
        return cls(
            camera_from_rslidar=np.asarray(cfg["camera_from_rslidar"], dtype=float),
            map_from_camera_init=Rotation.from_euler("ZYX", [yaw, pitch, roll]).as_matrix(),
            pcd_frame_id=str(cfg["pcd_frame_id"]),
            pose_label=str(cfg["pose_label"]),
        )

    def scan_to_camera(self, xyz_rslidar: np.ndarray) -> np.ndarray:
        """RoboSense (x, y, z) -> LeGO camera axes (y, z, x). Pure axis change."""
        return xyz_rslidar @ self.camera_from_rslidar.T

    def to_map(self, T_ci_cam: np.ndarray) -> np.ndarray:
        """camera_init <- camera  ->  map <- rslidar (LiDAR pose in map)."""
        T_map_ci = make_T(self.map_from_camera_init, np.zeros(3))
        T_cam_lidar = make_T(self.camera_from_rslidar, np.zeros(3))
        return T_map_ci @ T_ci_cam @ T_cam_lidar

    def from_map(self, T_map_lidar: np.ndarray) -> np.ndarray:
        """map <- rslidar  ->  camera_init <- camera (inverse of ``to_map``)."""
        T_ci_map = make_T(self.map_from_camera_init.T, np.zeros(3))
        T_lidar_cam = make_T(self.camera_from_rslidar.T, np.zeros(3))
        return T_ci_map @ T_map_lidar @ T_lidar_cam

    def map_pose(self, T_ci_cam: np.ndarray) -> np.ndarray:
        """[x, y, z, roll, pitch, yaw] of the LiDAR in map (radians)."""
        return xyz_rpy(self.to_map(T_ci_cam))
