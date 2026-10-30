"""Scan preprocessing and run/timing helpers shared by every global method.

All methods call ``ScanPrep`` with the same config block, so the comparison
only differs in the registration algorithm itself.
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any, Callable, Mapping

import numpy as np
import open3d as o3d

from ..preprocess import range_filter, remove_nonfinite, to_o3d, voxel_downsample
from .base import Candidate, GlobalResult


class GlobalFailure(Exception):
    """Raised inside a method body to report a clean, expected failure."""


@dataclass(frozen=True)
class ScanPrep:
    """NaN removal, range filter and voxel downsampling from the ``scan:`` config block."""

    min_range: float
    max_range: float
    voxel: float

    @classmethod
    def from_config(cls, cfg: Mapping[str, Any]) -> "ScanPrep":
        """Build from ``{min_range, max_range, voxel}``."""
        return cls(float(cfg["min_range"]), float(cfg["max_range"]), float(cfg["voxel"]))

    def __call__(self, scan_cam: np.ndarray) -> np.ndarray:
        """Return the filtered, downsampled (N, 3) scan; raises GlobalFailure if empty."""
        pts = remove_nonfinite(np.asarray(scan_cam, dtype=float).reshape(-1, 3))
        pts = voxel_downsample(range_filter(pts, self.min_range, self.max_range), self.voxel)
        if len(pts) < 3:
            raise GlobalFailure(f"scan has {len(pts)} usable points after preprocessing")
        return pts


@dataclass(frozen=True)
class FPFHParams:
    """Normal and FPFH radii shared by the feature-based methods."""

    voxel: float
    normal_radius: float
    normal_max_nn: int
    feature_radius: float
    feature_max_nn: int

    def map_args(self) -> tuple[float, float, int, float, int]:
        """Positional arguments of ``MapContext.fpfh``."""
        return (self.voxel, self.normal_radius, self.normal_max_nn, self.feature_radius,
                self.feature_max_nn)


def scan_fpfh(points: np.ndarray, p: FPFHParams) -> tuple[o3d.geometry.PointCloud, np.ndarray]:
    """Open3D cloud (with normals) and (N, 33) FPFH of an already-downsampled scan."""
    pc = to_o3d(points)
    pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=p.normal_radius,
                                                             max_nn=p.normal_max_nn))
    feat = o3d.pipelines.registration.compute_fpfh_feature(
        pc, o3d.geometry.KDTreeSearchParamHybrid(radius=p.feature_radius, max_nn=p.feature_max_nn))
    return pc, np.asarray(feat.data).T.copy()


def to_feature(descriptors: np.ndarray) -> o3d.pipelines.registration.Feature:
    """(N, D) descriptor array -> Open3D Feature (which stores D x N)."""
    feat = o3d.pipelines.registration.Feature()
    feat.data = np.ascontiguousarray(np.asarray(descriptors, dtype=float).T)
    return feat


def run_method(name: str, body: Callable[[], tuple[list[Candidate], dict[str, float]]]) -> GlobalResult:
    """Time ``body`` and wrap it: exceptions or non-finite poses become a failed result."""
    start = time.perf_counter()
    stage_times: dict[str, float] = {}
    candidates: list[Candidate] = []
    reason = ""
    try:
        candidates, stage_times = body()
        n_all = len(candidates)
        candidates = [c for c in candidates
                      if np.all(np.isfinite(c.T)) and np.isfinite(c.score)]
        if not candidates:
            reason = "no valid candidate" if n_all == 0 else "only non-finite candidates"
    except GlobalFailure as error:
        reason = str(error)
    except Exception as error:  # third-party solvers raise assorted errors on degenerate input
        reason = f"{type(error).__name__}: {str(error)[-300:]}"
    return GlobalResult(method=name, candidates=candidates,
                        runtime_s=time.perf_counter() - start, stage_times=stage_times,
                        failed=bool(reason), failure_reason=reason)
