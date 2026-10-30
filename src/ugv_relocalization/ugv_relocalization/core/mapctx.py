"""Map-side data computed once per process and shared by all methods.

Everything here depends only on the map (never on a scan or a reference pose).
Precomputation time is recorded per item so it can be reported separately
from per-scan runtime.
"""
from __future__ import annotations

import time
from typing import Any, Callable

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

from .frames import Frames
from .preprocess import to_o3d, voxel_downsample


class MapContext:
    """Frozen map in camera_init with cached derived products."""

    def __init__(self, map_ci: np.ndarray, frames: Frames, up_axis: int = 1) -> None:
        self.points = np.asarray(map_ci, dtype=float)
        self.frames = frames
        self.up_axis = up_axis  # camera_init vertical axis is Y
        self._cache: dict[tuple, Any] = {}
        self.precompute_s: dict[str, float] = {}

    def _cached(self, key: tuple, build: Callable[[], Any]) -> Any:
        if key not in self._cache:
            start = time.perf_counter()
            self._cache[key] = build()
            self.precompute_s["/".join(map(str, key))] = time.perf_counter() - start
        return self._cache[key]

    def downsampled(self, voxel: float) -> np.ndarray:
        """Voxel-centroid map points (voxel <= 0 returns the raw map)."""
        return self._cached(("down", voxel), lambda: voxel_downsample(self.points, voxel))

    def kdtree(self, voxel: float) -> cKDTree:
        """KD-tree over ``downsampled(voxel)``."""
        return self._cached(("kdtree", voxel), lambda: cKDTree(self.downsampled(voxel)))

    def o3d_cloud(self, voxel: float, normal_radius: float, normal_max_nn: int) -> o3d.geometry.PointCloud:
        """Open3D cloud of ``downsampled(voxel)`` with estimated normals."""
        def build() -> o3d.geometry.PointCloud:
            pc = to_o3d(self.downsampled(voxel))
            pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=normal_radius,
                                                                     max_nn=normal_max_nn))
            return pc
        return self._cached(("o3d", voxel, normal_radius, normal_max_nn), build)

    def normals(self, voxel: float, normal_radius: float, normal_max_nn: int) -> np.ndarray:
        """(N, 3) normals aligned with ``downsampled(voxel)``."""
        return np.asarray(self.o3d_cloud(voxel, normal_radius, normal_max_nn).normals)

    def fpfh(self, voxel: float, normal_radius: float, normal_max_nn: int,
             feature_radius: float, feature_max_nn: int) -> np.ndarray:
        """(N, 33) FPFH descriptors aligned with ``downsampled(voxel)``."""
        def build() -> np.ndarray:
            pc = self.o3d_cloud(voxel, normal_radius, normal_max_nn)
            feat = o3d.pipelines.registration.compute_fpfh_feature(
                pc, o3d.geometry.KDTreeSearchParamHybrid(radius=feature_radius, max_nn=feature_max_nn))
            return np.asarray(feat.data).T.copy()
        return self._cached(("fpfh", voxel, normal_radius, normal_max_nn, feature_radius,
                             feature_max_nn), build)

    def horizontal_bounds(self) -> tuple[np.ndarray, np.ndarray]:
        """(min, max) of the map over the two horizontal camera_init axes."""
        h = [a for a in range(3) if a != self.up_axis]
        return self.points[:, h].min(axis=0), self.points[:, h].max(axis=0)
