"""Point-cloud preprocessing that is allowed *before* registration.

Allowed: axis conversion (see ``frames``), NaN removal, range limits,
downsampling and cropping the *map*. Never the LeGO reference pose.
"""
from __future__ import annotations

import numpy as np
import open3d as o3d


def remove_nonfinite(points: np.ndarray) -> np.ndarray:
    """Drop rows containing NaN or Inf."""
    return points[np.isfinite(points).all(axis=1)]


def range_filter(points: np.ndarray, min_range: float, max_range: float) -> np.ndarray:
    """Keep points whose distance from the sensor origin is in [min, max]."""
    rng = np.linalg.norm(points, axis=1)
    return points[(rng >= min_range) & (rng <= max_range)]


def to_o3d(points: np.ndarray) -> o3d.geometry.PointCloud:
    """NumPy (N, 3) -> Open3D point cloud."""
    return o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.asarray(points, dtype=float)))


def voxel_downsample(points: np.ndarray, voxel: float) -> np.ndarray:
    """Voxel-grid centroid downsampling."""
    if voxel <= 0:
        return points
    return np.asarray(to_o3d(points).voxel_down_sample(voxel).points)


def crop_cylinder(points: np.ndarray, center: np.ndarray, radius: float,
                  up_axis: int) -> np.ndarray:
    """Keep points within ``radius`` of ``center`` measured perpendicular to ``up_axis``.

    For camera_init clouds the vertical axis is 1 (Y).
    """
    horizontal = [a for a in range(3) if a != up_axis]
    d = points[:, horizontal] - np.asarray(center)[horizontal]
    return points[np.einsum("ij,ij->i", d, d) <= radius * radius]
