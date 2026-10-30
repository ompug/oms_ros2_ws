"""Shared pieces of the fine-registration wrappers: scan cleaning, stages, results."""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any, Mapping, Sequence

import numpy as np

from ..icp import ICPResult
from ..preprocess import range_filter, remove_nonfinite


@dataclass(frozen=True)
class Stage:
    """One coarse-to-fine stage: scan voxel, max correspondence distance, iteration cap."""

    voxel: float
    max_corr_dist: float
    max_iter: int


def parse_stages(items: Sequence[Mapping[str, Any]]) -> list[Stage]:
    """Config list of ``{voxel, max_corr_dist, max_iter}`` -> stages, coarse first."""
    if not items:
        raise ValueError("at least one stage is required")
    return [Stage(float(s["voxel"]), float(s["max_corr_dist"]), int(s["max_iter"])) for s in items]


def clean_scan(scan_cam: np.ndarray, scan_cfg: Mapping[str, Any]) -> np.ndarray:
    """Drop NaN rows, then keep points inside ``[min_range, max_range]`` (config ``scan``)."""
    points = remove_nonfinite(np.asarray(scan_cam, dtype=float).reshape(-1, 3))
    return range_filter(points, float(scan_cfg["min_range"]), float(scan_cfg["max_range"]))


def failure(T_init: np.ndarray, reason: str, runtime_s: float,
            stages: list[ICPResult] | None = None) -> ICPResult:
    """A failed result that carries the initial pose back unchanged."""
    return ICPResult(np.asarray(T_init, dtype=float), 0, False, runtime_s, 0.0, 0, 0,
                     failed=True, failure_reason=reason, stages=stages or [])


def finite_pose(T: np.ndarray) -> bool:
    """True if ``T`` is a finite 4x4 matrix."""
    return T.shape == (4, 4) and bool(np.all(np.isfinite(T)))


class StagedRegistrar:
    """Base class: clean the scan, run the stages in order, aggregate into one ICPResult.

    Subclasses set ``name``, ``stages``, ``scan_cfg`` and implement ``_run_stage``
    (which returns an ICPResult whose ``runtime_s`` covers that stage's scan
    preprocessing plus alignment).
    """

    name: str
    stages: list[Stage]
    scan_cfg: Mapping[str, Any]
    precompute_s: float

    def _run_stage(self, scan: np.ndarray, stage: Stage, T: np.ndarray) -> ICPResult:
        raise NotImplementedError

    def register(self, scan_cam: np.ndarray, T_init: np.ndarray) -> ICPResult:
        """Align ``scan_cam`` (LeGO camera axes, NaN rows allowed) starting from ``T_init``."""
        start = time.perf_counter()
        T_init = np.asarray(T_init, dtype=float)
        scan = clean_scan(scan_cam, self.scan_cfg)
        if len(scan) < int(self.scan_cfg["min_points"]):
            return failure(T_init, f"only {len(scan)} finite in-range scan points",
                           time.perf_counter() - start)
        T, results = T_init, []
        for stage in self.stages:
            try:
                result = self._run_stage(scan, stage, T)
            except Exception as error:  # library errors on degenerate input
                result = failure(T, str(error)[-300:], 0.0)
            results.append(result)
            if result.failed:
                break
            T = result.T
        last = results[-1]
        counts = [r.iterations for r in results]
        return ICPResult(
            T=last.T,
            iterations=sum(counts) if min(counts) >= 0 else -1,
            iteration_limit_reached=last.iteration_limit_reached,
            runtime_s=time.perf_counter() - start,
            match_ratio=last.match_ratio,
            source_points_filtered=last.source_points_filtered,
            map_points_filtered=last.map_points_filtered,
            failed=last.failed,
            failure_reason=last.failure_reason,
            stages=results if len(results) > 1 else [],
        )
