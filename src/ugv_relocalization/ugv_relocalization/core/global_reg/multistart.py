"""Feature-free control: ground-height estimate + grid x yaw correlative search + ICP refine.

No descriptors are used. The sensor height above ground comes from the scan
alone, candidate positions come from near-horizontal map ground, and the
coarse score is the number of map KD-tree hits within ``hit_radius``.
"""
from __future__ import annotations

import time
from typing import Any, Mapping

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

from ..frames import make_T
from ..mapctx import MapContext
from ..preprocess import to_o3d
from .base import Candidate, GlobalResult
from .scan_prep import GlobalFailure, ScanPrep, run_method

_REG = o3d.pipelines.registration


def estimate_sensor_height(points: np.ndarray, rng: np.random.Generator, ground_thresh: float,
                           iterations: int, max_tilt_deg: float, min_inlier_frac: float,
                           up_axis: int = 1) -> tuple[float, np.ndarray]:
    """RANSAC ground plane from scan points below the sensor; returns (height, inlier mask).

    The mask is over ``points`` (all rows); height is the origin-to-plane distance.
    """
    below = np.flatnonzero(points[:, up_axis] < 0.0)
    if len(below) < 3:
        raise GlobalFailure("too few scan points below the sensor to fit a ground plane")
    pts = points[below]
    cos_tilt = np.cos(np.radians(max_tilt_deg))
    best_n, best_d, best_count = None, 0.0, 0
    for tri in rng.integers(0, len(pts), size=(iterations, 3)):
        a, b, c = pts[tri]
        n = np.cross(b - a, c - a)
        norm = np.linalg.norm(n)
        if norm < 1e-9 or abs(n[up_axis]) / norm < cos_tilt:
            continue
        n = n / norm
        d = -n @ a
        count = int((np.abs(pts @ n + d) < ground_thresh).sum())
        if count > best_count:
            best_n, best_d, best_count = n, d, count
    if best_n is None or best_count < min_inlier_frac * len(pts):
        raise GlobalFailure("no near-horizontal ground plane found in the scan")
    inl = np.abs(pts @ best_n + best_d) < ground_thresh
    centroid = pts[inl].mean(axis=0)
    n = np.linalg.svd(pts[inl] - centroid)[2][-1]
    n = n if n[up_axis] > 0 else -n
    mask = np.zeros(len(points), dtype=bool)
    mask[below[np.abs(pts @ n - n @ centroid) < ground_thresh]] = True
    return float(abs(n @ centroid)), mask


def yaw_matrices(yaws: np.ndarray) -> np.ndarray:
    """(Y, 3, 3) rotations about camera_init Y."""
    c, s = np.cos(yaws), np.sin(yaws)
    R = np.zeros((len(yaws), 3, 3))
    R[:, 0, 0], R[:, 0, 2], R[:, 1, 1], R[:, 2, 0], R[:, 2, 2] = c, s, 1.0, -s, c
    return R


class MultiStartSearch:
    """Grid-position x yaw correlative search, refined by a short ICP; K ranked candidates."""

    def __init__(self, name: str, scan: Mapping[str, Any], voxel: float, normal_radius: float,
                 normal_max_nn: int, ground_normal_thresh_deg: float, grid_spacing: float,
                 cell_height_percentile: float, yaw_steps: int, coarse_points: int,
                 hit_radius: float, top_k: int, nms_radius: float, nms_yaw_deg: float,
                 ground_ransac: Mapping[str, Any], nonground_margin: float, refine_voxel: float,
                 refine_max_dist: float, refine_iterations: int, position_chunk: int,
                 seed: int) -> None:
        self.name = name
        self.prep = ScanPrep.from_config(scan)
        self.voxel, self.normal_radius, self.normal_max_nn = float(voxel), float(normal_radius), int(normal_max_nn)
        self.cos_ground = float(np.cos(np.radians(ground_normal_thresh_deg)))
        self.spacing, self.percentile = float(grid_spacing), float(cell_height_percentile)
        self.yaw_steps, self.coarse_points = int(yaw_steps), int(coarse_points)
        self.radius, self.top_k = float(hit_radius), int(top_k)
        self.nms_radius, self.nms_yaw = float(nms_radius), np.radians(nms_yaw_deg)
        self.ransac = dict(ground_ransac)
        self.margin = float(nonground_margin)
        self.refine_voxel, self.refine_dist = float(refine_voxel), float(refine_max_dist)
        self.refine_iter, self.chunk = int(refine_iterations), int(position_chunk)
        self.seed = int(seed)

    def ground_cells(self, map_ctx: MapContext) -> np.ndarray:
        """(P, 3) candidate ground anchors: cell centre (horizontal) + low-percentile ground height."""
        def build() -> np.ndarray:
            up = map_ctx.up_axis
            pts = map_ctx.downsampled(self.voxel)
            nrm = map_ctx.normals(self.voxel, self.normal_radius, self.normal_max_nn)
            pts = pts[np.abs(nrm[:, up]) > self.cos_ground]
            hor = [a for a in range(3) if a != up]
            cell = np.floor(pts[:, hor] / self.spacing).astype(np.int64)
            _, inv = np.unique(cell, axis=0, return_inverse=True)
            order = np.lexsort((pts[:, up], inv.ravel()))
            inv_sorted = inv.ravel()[order]
            first = np.flatnonzero(np.r_[True, inv_sorted[1:] != inv_sorted[:-1]])
            counts = np.diff(np.r_[first, len(order)])
            pick = order[first + np.floor(self.percentile * (counts - 1)).astype(int)]
            out = np.zeros((len(first), 3))
            out[:, hor] = (cell[order[first]] + 0.5) * self.spacing
            out[:, up] = pts[pick, up]
            return out
        return map_ctx._cached(("multistart_cells", self.voxel, self.normal_radius, self.normal_max_nn,
                                self.spacing, self.percentile, self.cos_ground), build)

    def _coarse(self, anchors: np.ndarray, sub: np.ndarray, tree: cKDTree, yaws: np.ndarray) -> np.ndarray:
        """(Y, P) KD-tree hit counts of the yawed scan subsample placed at every anchor."""
        hits = np.zeros((len(yaws), len(anchors)), dtype=np.int32)
        for iy, R in enumerate(yaw_matrices(yaws)):
            rot = sub @ R.T
            for s in range(0, len(anchors), self.chunk):
                q = anchors[s:s + self.chunk, None, :] + rot[None]
                d, _ = tree.query(q.reshape(-1, 3), k=1, distance_upper_bound=self.radius, workers=-1)
                hits[iy, s:s + self.chunk] = np.isfinite(d).reshape(len(q), -1).sum(axis=1)
        return hits

    def _select(self, hits: np.ndarray, anchors: np.ndarray, yaws: np.ndarray,
                up: int) -> list[tuple[int, int]]:
        """Greedy non-maximum suppression over (yaw, position); returns (yaw_idx, pos_idx) pairs."""
        hor = [a for a in range(3) if a != up]
        flat = np.argsort(hits, axis=None)[::-1][: max(50 * self.top_k, 500)]
        chosen: list[tuple[int, int]] = []
        for f in flat:
            iy, ip = divmod(int(f), hits.shape[1])
            ok = True
            for jy, jp in chosen:
                dyaw = abs((yaws[iy] - yaws[jy] + np.pi) % (2 * np.pi) - np.pi)
                if dyaw <= self.nms_yaw and np.linalg.norm(anchors[ip, hor] - anchors[jp, hor]) <= self.nms_radius:
                    ok = False
                    break
            if ok:
                chosen.append((iy, ip))
                if len(chosen) == self.top_k:
                    break
        return chosen

    def propose(self, scan_cam: np.ndarray, map_ctx: MapContext) -> GlobalResult:
        """Return up to ``top_k`` refined candidates, best (highest inlier fraction) first."""
        return self._propose(scan_cam, map_ctx, None, None)

    def propose_in_region(self, scan_cam: np.ndarray, map_ctx: MapContext, center_xy: np.ndarray,
                          radius_m: float, yaw_center: float, yaw_half_width: float) -> GlobalResult:
        """Like ``propose`` but searching only ground cells within ``radius_m`` of ``center_xy``
        (map frame, metres) and lidar yaws within ``yaw_half_width`` degrees of ``yaw_center``
        degrees (map-frame yaw). The yaw grid keeps the global spacing and is centred on ``yaw_center``.
        """
        return self._propose(scan_cam, map_ctx, (np.asarray(center_xy, dtype=float), float(radius_m)),
                             (float(yaw_center), float(yaw_half_width)))

    def _region_cells(self, anchors: np.ndarray, map_ctx: MapContext, center_xy: np.ndarray,
                      radius_m: float) -> np.ndarray:
        """Anchors (camera_init) within ``radius_m`` of ``center_xy`` horizontally (map frame)."""
        xy_map = (anchors @ map_ctx.frames.map_from_camera_init.T)[:, :2]
        return anchors[np.linalg.norm(xy_map - center_xy, axis=1) <= radius_m]

    def _window_yaws(self, center_deg: float, half_width_deg: float) -> np.ndarray:
        """Yaw grid (radians) at the global spacing covering centre +- half-width (full grid if wider)."""
        step_deg = 360.0 / self.yaw_steps
        m = int(np.ceil(half_width_deg / step_deg))
        if 2 * m + 1 >= self.yaw_steps:
            return np.arange(self.yaw_steps) * 2 * np.pi / self.yaw_steps
        return np.radians(center_deg + np.arange(-m, m + 1) * step_deg) % (2 * np.pi)

    def _propose(self, scan_cam: np.ndarray, map_ctx: MapContext,
                 region: tuple[np.ndarray, float] | None,
                 yaw_window: tuple[float, float] | None) -> GlobalResult:
        def body():
            rng = np.random.default_rng(self.seed)
            up = map_ctx.up_axis
            scan = self.prep(scan_cam)
            times: dict[str, float] = {}
            t0 = time.perf_counter()
            height, ground = estimate_sensor_height(scan, rng, up_axis=up, **self.ransac)
            nonground = scan[~ground & (scan[:, up] > -height + self.margin)]
            if len(nonground) < 3:
                raise GlobalFailure("no non-ground scan points")
            sub = nonground[rng.choice(len(nonground), min(self.coarse_points, len(nonground)),
                                       replace=False)]
            anchors = self.ground_cells(map_ctx).copy()
            if len(anchors) == 0:
                raise GlobalFailure("map has no near-horizontal ground cells")
            if region is not None:
                anchors = self._region_cells(anchors, map_ctx, *region)
                if len(anchors) == 0:
                    raise GlobalFailure("no ground cells inside the search region")
            anchors[:, up] += height
            yaws = (self._window_yaws(*yaw_window) if yaw_window is not None
                    else np.arange(self.yaw_steps) * 2 * np.pi / self.yaw_steps)
            tree = map_ctx.kdtree(self.voxel)
            hits = self._coarse(anchors, sub, tree, yaws)
            chosen = self._select(hits, anchors, yaws, up)
            times["coarse"] = time.perf_counter() - t0

            t0 = time.perf_counter()
            R_all = yaw_matrices(yaws)
            src = to_o3d(nonground).voxel_down_sample(self.refine_voxel) if self.refine_voxel > 0 else to_o3d(nonground)
            dst = map_ctx._cached(("ms_refine_cloud", self.refine_voxel),
                                  lambda: to_o3d(map_ctx.downsampled(self.refine_voxel)))
            criteria = _REG.ICPConvergenceCriteria(max_iteration=self.refine_iter)
            cands = []
            for iy, ip in chosen:
                T0 = make_T(R_all[iy], anchors[ip])
                res = _REG.registration_icp(src, dst, self.refine_dist, T0,
                                            _REG.TransformationEstimationPointToPoint(False), criteria)
                T = np.asarray(res.transformation, dtype=float)
                moved = nonground @ T[:3, :3].T + T[:3, 3]
                d, _ = tree.query(moved, k=1, distance_upper_bound=self.radius, workers=-1)
                frac = float(np.isfinite(d).mean())
                cands.append(Candidate(T, frac, {"coarse_hits": int(hits[iy, ip]),
                                                 "grid_yaw_deg": float(np.degrees(yaws[iy])),
                                                 "grid_xyz": anchors[ip].tolist(),
                                                 "sensor_height": height}))
            times["refine"] = time.perf_counter() - t0
            return sorted(cands, key=lambda c: -c.score), times
        return run_method(self.name, body)
