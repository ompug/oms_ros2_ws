"""Open3D fine registration: robust point-to-plane ICP and generalized ICP.

The map is downsampled once in the constructor (point-to-plane: normals via
``MapContext.o3d_cloud``; GICP: covariances on a private copy). Each call
voxel-downsamples the scan per stage.

Limitation: ``open3d.pipelines.registration`` does not expose an iteration
count or a hit-the-cap flag. ``iterations`` is therefore -1 (unknown) and
``iteration_limit_reached`` is always False. ``match_ratio`` is Open3D's
fitness (inlier correspondences / scan points). ``precompute_s`` includes
the MapContext build when that product was not yet cached.
"""
from __future__ import annotations

import time
from typing import Any, Mapping

import numpy as np
import open3d as o3d

from ..icp import ICPResult
from ..mapctx import MapContext
from ..preprocess import to_o3d
from ._common import Stage, StagedRegistrar, failure, finite_pose, parse_stages

_REG = o3d.pipelines.registration


def make_kernel(kernel_cfg: Mapping[str, Any]) -> "_REG.RobustKernel":
    """Robust kernel from ``{type: tukey|huber, scale}``."""
    kind, scale = str(kernel_cfg["type"]).lower(), float(kernel_cfg["scale"])
    if kind == "tukey":
        return _REG.TukeyLoss(scale)
    if kind == "huber":
        return _REG.HuberLoss(scale)
    raise ValueError(f"unknown robust kernel {kind!r}; use tukey or huber")


class _Open3DBase(StagedRegistrar):
    """Common stage loop: downsample scan, run ``_estimate``, fill the ICPResult."""

    def __init__(self, cfg: Mapping[str, Any]) -> None:
        self.name = str(cfg["name"])
        self.scan_cfg = cfg["scan"]
        self.stages = parse_stages(cfg["stages"])
        conv = cfg["convergence"]
        self._conv = (float(conv["relative_fitness"]), float(conv["relative_rmse"]))
        self._target: "o3d.geometry.PointCloud"

    def _prepare_scan(self, pc: "o3d.geometry.PointCloud") -> None:
        """Hook for per-scan attributes (GICP covariances)."""

    def _run_stage(self, scan: np.ndarray, stage: Stage, T: np.ndarray) -> ICPResult:
        start = time.perf_counter()
        source = to_o3d(scan).voxel_down_sample(stage.voxel) if stage.voxel > 0 else to_o3d(scan)
        self._prepare_scan(source)
        criteria = _REG.ICPConvergenceCriteria(self._conv[0], self._conv[1], stage.max_iter)
        res = self._register(source, stage, T, criteria)
        T_out = np.asarray(res.transformation, dtype=float)
        n_source = len(source.points)
        runtime = time.perf_counter() - start
        if not finite_pose(T_out):
            return failure(T, "nonfinite transform", runtime)
        if len(res.correspondence_set) == 0:
            return failure(T, "no correspondences within max_corr_dist", runtime)
        return ICPResult(T=T_out, iterations=-1, iteration_limit_reached=False,
                         runtime_s=runtime, match_ratio=float(res.fitness),
                         source_points_filtered=n_source,
                         map_points_filtered=len(self._target.points))

    def _register(self, source: Any, stage: Stage, T: np.ndarray, criteria: Any) -> Any:
        raise NotImplementedError


class Open3DPointToPlane(_Open3DBase):
    """Point-to-plane ICP with a robust kernel (Tukey or Huber, scale from config)."""

    def __init__(self, cfg: Mapping[str, Any], map_ctx: MapContext) -> None:
        super().__init__(cfg)
        start = time.perf_counter()
        m = cfg["map"]
        self._target = map_ctx.o3d_cloud(float(m["voxel"]), float(m["normal_radius"]),
                                         int(m["normal_max_nn"]))
        self._estimation = _REG.TransformationEstimationPointToPlane(make_kernel(cfg["kernel"]))
        self.precompute_s = time.perf_counter() - start

    def _register(self, source: Any, stage: Stage, T: np.ndarray, criteria: Any) -> Any:
        return _REG.registration_icp(source, self._target, stage.max_corr_dist, T,
                                     self._estimation, criteria)


class Open3DGICP(_Open3DBase):
    """Generalized ICP (``registration_generalized_icp``); covariances from k-NN."""

    def __init__(self, cfg: Mapping[str, Any], map_ctx: MapContext) -> None:
        super().__init__(cfg)
        start = time.perf_counter()
        m = cfg["map"]
        self._cov_knn = int(cfg["covariance_knn"])
        self._target = to_o3d(map_ctx.downsampled(float(m["voxel"])))
        self._target.estimate_covariances(o3d.geometry.KDTreeSearchParamKNN(self._cov_knn))
        kernel = make_kernel(cfg["kernel"]) if cfg.get("kernel") else None
        epsilon = float(cfg["epsilon"])
        self._estimation = (_REG.TransformationEstimationForGeneralizedICP(epsilon, kernel)
                            if kernel else _REG.TransformationEstimationForGeneralizedICP(epsilon))
        self.precompute_s = time.perf_counter() - start

    def _prepare_scan(self, pc: "o3d.geometry.PointCloud") -> None:
        pc.estimate_covariances(o3d.geometry.KDTreeSearchParamKNN(self._cov_knn))

    def _register(self, source: Any, stage: Stage, T: np.ndarray, criteria: Any) -> Any:
        return _REG.registration_generalized_icp(source, self._target, stage.max_corr_dist, T,
                                                 self._estimation, criteria)
