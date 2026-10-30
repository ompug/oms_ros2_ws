"""ROS-free logic of the relocalization node: prediction, scheduling, gating, mode switching.

Everything works on 4x4 poses in the map frame (``reloc_map``, ROS axes, z up) and on
integer header stamps in nanoseconds. Header stamps, never the node clock, drive every
decision, so playback rate and clock mismatches (bag stamps in 2017, bag clock in 2026) do
not matter. The registration inputs are the scan, the map and the prediction only.
"""
from __future__ import annotations

import threading
import time
from bisect import bisect_left
from collections import OrderedDict, deque
from dataclasses import dataclass
from enum import Enum
from typing import Any, Mapping

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from .engine import build_fine
from .core.frames import Frames, invert_T, wrap_deg, xyz_rpy
from .core.gate import GateFeatures, ScanScorer, decide, features_dict
from .core.mapctx import MapContext
from .core.pipeline import StartupPipeline
from .core.preprocess import crop_cylinder

NS = 1_000_000_000


@dataclass(frozen=True)
class RuntimeThresholds:
    """Gate for runtime corrections (structure fit, attitude, correction size)."""

    min_struct_inlier: float
    max_abs_roll_deg: float
    max_abs_pitch_deg: float
    require_converged: bool
    max_correction_xy_m: float
    max_correction_yaw_deg: float

    def gate_dict(self) -> dict[str, Any]:
        """The subset ``relocalization.gate.decide`` reads (ambiguity is not used at runtime)."""
        return {"min_struct_inlier": self.min_struct_inlier, "max_abs_roll_deg": self.max_abs_roll_deg,
                "max_abs_pitch_deg": self.max_abs_pitch_deg, "require_converged": self.require_converged,
                "max_ambiguity_ratio": 1.0}


def correction_size(T_pred: np.ndarray, T_new: np.ndarray) -> tuple[float, float]:
    """(XY distance m, |yaw difference| deg) between two map poses."""
    a, b = xyz_rpy(T_pred), xyz_rpy(T_new)
    return float(np.linalg.norm(a[:2] - b[:2])), float(abs(wrap_deg(np.degrees(a[5] - b[5]))))


def gate_correction(features: GateFeatures, converged: bool, failed: bool, dxy_m: float,
                    dyaw_deg: float, th: RuntimeThresholds) -> tuple[bool, list[str]]:
    """Accept a runtime correction? Returns (accepted, failed checks); a failed ICP never passes."""
    if failed:
        return False, ["fine_failed"]
    _, reasons = decide(features, 0.0, converged, th.gate_dict())
    if dxy_m > th.max_correction_xy_m:
        reasons.append("correction_xy_too_large")
    if dyaw_deg > th.max_correction_yaw_deg:
        reasons.append("correction_yaw_too_large")
    return not reasons, reasons


def interp_pose(T0: np.ndarray, T1: np.ndarray, alpha: float) -> np.ndarray:
    """Linear position, slerp orientation."""
    T = np.eye(4)
    rot = Slerp([0.0, 1.0], Rotation.from_matrix(np.stack([T0[:3, :3], T1[:3, :3]])))([alpha])[0]
    T[:3, :3] = rot.as_matrix()
    T[:3, 3] = (1 - alpha) * T0[:3, 3] + alpha * T1[:3, 3]
    return T


class OdomPredictor:
    """T_map_lidar(t) = T_map_odom @ T_odom_lidar(t), with T_map_odom set by each accepted fix."""

    def __init__(self, buffer_s: float, max_gap_s: float, max_extrapolation_s: float) -> None:
        self.buffer_ns, self.max_gap_ns = int(buffer_s * NS), int(max_gap_s * NS)
        self.max_extrap_ns = int(max_extrapolation_s * NS)
        self._stamps: deque[int] = deque()
        self._poses: deque[np.ndarray] = deque()
        self.T_map_odom: np.ndarray | None = None
        self._lock = threading.Lock()

    @property
    def ready(self) -> bool:
        """True once a fix has set T_map_odom."""
        return self.T_map_odom is not None

    @property
    def newest_ns(self) -> int | None:
        """Stamp of the newest buffered odometry sample, or None."""
        return self._stamps[-1] if self._stamps else None

    def add(self, stamp_ns: int, T_odom_lidar: np.ndarray) -> None:
        """Buffer one odometry sample (stamps must not go backwards; older samples are dropped)."""
        with self._lock:
            if self._stamps and stamp_ns <= self._stamps[-1]:
                return
            self._stamps.append(stamp_ns)
            self._poses.append(np.array(T_odom_lidar, dtype=float))
            while self._stamps[0] < stamp_ns - self.buffer_ns:
                self._stamps.popleft()
                self._poses.popleft()

    def odom_at(self, stamp_ns: int) -> np.ndarray | None:
        """Odometry pose at ``stamp_ns`` (interpolated; held for a short time after the newest)."""
        with self._lock:
            if not self._stamps or stamp_ns < self._stamps[0]:
                return None
            if stamp_ns > self._stamps[-1]:
                return self._poses[-1].copy() if stamp_ns - self._stamps[-1] <= self.max_extrap_ns else None
            if stamp_ns == self._stamps[-1]:
                return self._poses[-1].copy()
            stamps = list(self._stamps)
            i = max(bisect_left(stamps, stamp_ns), 1)
            gap = stamps[i] - stamps[i - 1]
            if gap > self.max_gap_ns:
                return None
            return interp_pose(self._poses[i - 1], self._poses[i], (stamp_ns - stamps[i - 1]) / gap)

    def predict(self, stamp_ns: int) -> np.ndarray | None:
        """Predicted map pose of the lidar, or None without a fix or odometry."""
        T_odom = self.odom_at(stamp_ns)
        T_map_odom = self.T_map_odom
        return None if T_odom is None or T_map_odom is None else T_map_odom @ T_odom

    def set_fix(self, stamp_ns: int, T_map_lidar: np.ndarray) -> bool:
        """Set T_map_odom from a map pose measured at ``stamp_ns``. False without odometry there."""
        T_odom = self.odom_at(stamp_ns)
        if T_odom is None:
            return False
        self.T_map_odom = np.asarray(T_map_lidar, float) @ invert_T(T_odom)
        return True

    def reset(self) -> None:
        """Forget T_map_odom (re-initialisation)."""
        self.T_map_odom = None


class SubmapRegistrars:
    """Fine registrars on map crops: one per grid cell, built lazily, kept in an LRU cache.

    The full-map structures (points, horizontal coordinates) are prepared once. A crop is a
    camera_init cylinder of ``radius_m`` around the centre of the cell holding the prediction,
    so the registrar is reused while the robot stays in that cell. ``radius_m <= 0`` always
    uses ``full_fine``.
    """

    def __init__(self, map_ctx: MapContext, fine_cfg: Mapping[str, Any], full_fine: Any, radius_m: float,
                 cell_m: float, cache_size: int, min_points: int) -> None:
        if radius_m > 0 and radius_m <= cell_m:
            raise ValueError("crop_radius_m must exceed crop_cell_m")
        self.map_ctx, self.fine_cfg, self.full_fine = map_ctx, fine_cfg, full_fine
        self.radius, self.cell, self.cache_size, self.min_points = radius_m, cell_m, cache_size, min_points
        self._cache: OrderedDict[tuple[int, int], Any] = OrderedDict()
        self._up = map_ctx.up_axis

    def get(self, T_ci_pred: np.ndarray) -> tuple[Any, dict[str, Any]]:
        """(registrar, info) for a camera_init prediction; info has crop size and build time."""
        if self.radius <= 0:
            return self.full_fine, {"crop_points": len(self.map_ctx.points), "build_s": 0.0, "cache_hit": True,
                                    "full_map": True}
        h = [a for a in range(3) if a != self._up]
        key = tuple(int(round(T_ci_pred[a, 3] / self.cell)) for a in h)
        if key in self._cache:
            self._cache.move_to_end(key)
            reg, n = self._cache[key]
            return reg, {"crop_points": n, "build_s": 0.0, "cache_hit": True, "full_map": False}
        centre = np.zeros(3)
        centre[h] = np.asarray(key, float) * self.cell
        crop = crop_cylinder(self.map_ctx.points, centre, self.radius, self._up)
        if len(crop) < self.min_points:
            return self.full_fine, {"crop_points": len(self.map_ctx.points), "build_s": 0.0, "cache_hit": True,
                                    "full_map": True}
        start = time.perf_counter()
        reg = build_fine(self.fine_cfg, MapContext(crop, self.map_ctx.frames, self._up))
        build_s = time.perf_counter() - start
        self._cache[key] = (reg, len(crop))
        while len(self._cache) > self.cache_size:
            self._cache.popitem(last=False)
        return reg, {"crop_points": len(crop), "build_s": build_s, "cache_hit": False, "full_map": False}


class RuntimeLocalizer:
    """One gated ICP correction seeded from the predictor, on a cropped map."""

    def __init__(self, frames: Frames, map_tree: Any, gate_features_cfg: Mapping[str, Any],
                 th: RuntimeThresholds, registrars: SubmapRegistrars, predictor: Any) -> None:
        self.frames, self.tree, self.feat_cfg, self.th = frames, map_tree, gate_features_cfg, th
        self.registrars, self.predictor = registrars, predictor

    def attempt(self, scan_cam: np.ndarray, stamp_ns: int) -> dict[str, Any] | None:
        """Register ``scan_cam`` (LeGO camera axes) at its stamp; None if there is no prediction.

        On acceptance the predictor is updated with the corrected pose. The returned dict is
        the status record (``T_map_lidar`` and ``T_map_pred`` are 4x4 arrays; drop them before JSON).
        """
        T_pred = self.predictor.predict(stamp_ns)
        if T_pred is None:
            return None
        start = time.perf_counter()
        registrar, info = self.registrars.get(self.frames.from_map(T_pred))
        result = registrar.register(scan_cam, self.frames.from_map(T_pred))
        t_icp = time.perf_counter()
        scorer = ScanScorer(scan_cam, self.tree, self.frames, self.feat_cfg)
        features = scorer.features(result.T)
        T_new = self.frames.to_map(result.T)
        dxy, dyaw = correction_size(T_pred, T_new)
        accepted, reasons = gate_correction(features, result.converged, result.failed, dxy, dyaw, self.th)
        if accepted and not self.predictor.set_fix(stamp_ns, T_new):
            accepted, reasons = False, ["no_odometry_at_stamp"]
        end = time.perf_counter()
        return {"accepted": accepted, "reasons": reasons, "features": features_dict(features),
                "icp_s": result.runtime_s, "build_s": info["build_s"], "gate_s": end - t_icp,
                "total_s": end - start, "iterations": result.iterations, "converged": result.converged,
                "failed": result.failed, "correction_xy_m": dxy, "correction_yaw_deg": dyaw,
                "crop_points": info["crop_points"], "crop_cache_hit": info["cache_hit"],
                "full_map": info["full_map"], "T_map_lidar": T_new, "T_map_pred": T_pred}
