"""Verification gate: accept a refined pose only when it fits and is unambiguous.

Features are computed from the scan, the map and the candidate pose only.
The score is the fraction of non-ground scan points within a tight
distance of the map: ground matches almost everywhere, which is why Phase 1's
2.5 m inlier fraction stayed at 97.6 % for a wrong pose.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any, Mapping

import numpy as np
from scipy.spatial import cKDTree

from .frames import Frames, transform_points, wrap_deg, xyz_rpy
from .global_reg.multistart import estimate_sensor_height
from .global_reg.scan_prep import GlobalFailure
from .preprocess import range_filter, remove_nonfinite, voxel_downsample


@dataclass
class GateFeatures:
    """Everything the gate looks at for one refined pose."""

    struct_inlier: float
    all_inlier_tight: float
    all_inlier_loose: float
    struct_rmse_m: float | None
    abs_roll_deg: float
    abs_pitch_deg: float
    ground_found: bool
    n_struct_points: int


class ScanScorer:
    """Pre-splits one scan into ground / structure, then scores poses against the map."""

    def __init__(self, scan_cam: np.ndarray, map_tree: cKDTree, frames: Frames,
                 cfg: Mapping[str, Any]) -> None:
        s = cfg["scan"]
        pts = voxel_downsample(range_filter(remove_nonfinite(scan_cam), s["min_range"], s["max_range"]),
                               s["voxel"])
        g = cfg["ground"]
        try:
            _, ground = estimate_sensor_height(pts, np.random.default_rng(g["seed"]), g["ground_thresh"],
                                               g["iterations"], g["max_tilt_deg"], g["min_inlier_frac"])
            self.ground_found = True
        except GlobalFailure:
            ground = np.zeros(len(pts), dtype=bool)
            self.ground_found = False
        self.points = pts
        self.struct = pts[~ground]
        self.tree = map_tree
        self.frames = frames
        self.tight = float(cfg["tight_distance_m"])
        self.loose = float(cfg["loose_distance_m"])

    def features(self, T_ci_cam: np.ndarray) -> GateFeatures:
        """Fit statistics of the scan placed at ``T_ci_cam``."""
        d_all, _ = self.tree.query(transform_points(T_ci_cam, self.points), k=1)
        d_st, _ = self.tree.query(transform_points(T_ci_cam, self.struct), k=1) if len(self.struct) \
            else (np.array([]), None)
        good = d_st < self.tight
        rpy = np.degrees(xyz_rpy(self.frames.to_map(T_ci_cam))[3:])
        return GateFeatures(
            struct_inlier=float(good.mean()) if len(d_st) else 0.0,
            all_inlier_tight=float((d_all < self.tight).mean()),
            all_inlier_loose=float((d_all < self.loose).mean()),
            struct_rmse_m=float(np.sqrt(np.mean(d_st[good] ** 2))) if good.any() else None,
            abs_roll_deg=float(abs(wrap_deg(rpy[0]))), abs_pitch_deg=float(abs(wrap_deg(rpy[1]))),
            ground_found=self.ground_found, n_struct_points=int(len(self.struct)))


def distinct(frames: Frames, A: np.ndarray, B: np.ndarray, cfg: Mapping[str, Any]) -> bool:
    """True when two poses are different hypotheses (far apart in XY or yaw)."""
    a, b = xyz_rpy(frames.to_map(A)), xyz_rpy(frames.to_map(B))
    return bool(np.linalg.norm(a[:2] - b[:2]) >= cfg["min_xy_m"]
                or abs(wrap_deg(np.degrees(a[5] - b[5]))) >= cfg["min_yaw_deg"])


def ambiguity_ratio(scores: list[float], poses: list[np.ndarray], frames: Frames,
                    cfg: Mapping[str, Any]) -> tuple[int, float, float]:
    """(best index, second-best score among poses distinct from best, ratio second/best)."""
    best = int(np.argmax(scores))
    others = [s for i, s in enumerate(scores) if i != best and distinct(frames, poses[i], poses[best], cfg)]
    second = max(others) if others else 0.0
    return best, float(second), float(second / scores[best]) if scores[best] > 0 else 1.0


def decide(f: GateFeatures, ratio: float, converged: bool, th: Mapping[str, Any]) -> tuple[bool, list[str]]:
    """Accept/reject with the list of failed checks (empty list = accept)."""
    reasons = []
    if f.struct_inlier < th["min_struct_inlier"]:
        reasons.append("low_structure_inliers")
    if f.abs_roll_deg > th["max_abs_roll_deg"]:
        reasons.append("roll")
    if f.abs_pitch_deg > th["max_abs_pitch_deg"]:
        reasons.append("pitch")
    if ratio > th["max_ambiguity_ratio"]:
        reasons.append("ambiguous")
    if th.get("require_converged") and not converged:
        reasons.append("not_converged")
    return not reasons, reasons


def features_dict(f: GateFeatures) -> dict[str, Any]:
    """Flat dict with a ``gate_`` prefix for logging."""
    return {f"gate_{k}": v for k, v in asdict(f).items()}
