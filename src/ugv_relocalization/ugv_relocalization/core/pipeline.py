"""Startup relocalisation: global proposal -> fine ICP -> verification gate.

Uses only the scan and the map. Challenger hypotheses (yaw flips and shifts
around the best refined pose) give every method a second-best pose for the
ambiguity ratio, including methods that return a single candidate.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Mapping

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from .frames import Frames
from .gate import GateFeatures, ScanScorer, ambiguity_ratio, decide
from .global_reg.base import GlobalResult
from .icp import ICPResult
from .mapctx import MapContext


@dataclass
class Refined:
    """One fine-ICP run from one hypothesis."""

    source: str
    T_init: np.ndarray
    icp: ICPResult
    features: GateFeatures

    @property
    def score(self) -> float:
        return 0.0 if self.icp.failed else self.features.struct_inlier


@dataclass
class StartupResult:
    """Full trace of one startup attempt."""

    global_result: GlobalResult
    refined: list[Refined] = field(default_factory=list)
    best: int = -1
    second_score: float = 0.0
    ambiguity: float = 1.0
    accepted: bool = False
    reasons: list[str] = field(default_factory=list)
    times: dict[str, float] = field(default_factory=dict)

    @property
    def final(self) -> Refined | None:
        return self.refined[self.best] if self.best >= 0 else None


def challengers(frames: Frames, T_ci: np.ndarray, cfg: Mapping[str, Any]) -> list[tuple[str, np.ndarray]]:
    """Yaw-flipped and shifted copies of a pose (map frame), as camera_init poses."""
    T = frames.to_map(T_ci)
    out = []
    for yaw in cfg["yaw_offsets_deg"]:
        A = T.copy()
        A[:3, :3] = Rotation.from_euler("z", yaw, degrees=True).as_matrix() @ T[:3, :3]
        out.append((f"challenger_yaw{yaw:g}", frames.from_map(A)))
    s = float(cfg["shifts_m"])
    for name, d in (("+x", (s, 0)), ("-x", (-s, 0)), ("+y", (0, s)), ("-y", (0, -s))):
        A = T.copy()
        A[:2, 3] += d
        out.append((f"challenger_shift{name}", frames.from_map(A)))
    return out


class StartupPipeline:
    """One global method + one fine method + the gate."""

    def __init__(self, global_reg: Any, fine: Any, map_ctx: MapContext, map_tree: cKDTree,
                 pipe_cfg: Mapping[str, Any], gate_cfg: Mapping[str, Any],
                 thresholds: Mapping[str, Any]) -> None:
        self.global_reg, self.fine = global_reg, fine
        self.map_ctx, self.tree = map_ctx, map_tree
        self.cfg, self.gate_cfg, self.th = pipe_cfg, gate_cfg, thresholds
        self.frames = map_ctx.frames

    def refine_all(self, scan_cam: np.ndarray, scorer: ScanScorer,
                   hyps: list[tuple[str, np.ndarray]]) -> list[Refined]:
        out = []
        for source, T0 in hyps:
            res = self.fine.register(scan_cam, T0)
            out.append(Refined(source, T0, res, scorer.features(res.T)))
        return out

    def run(self, scan_cam: np.ndarray, global_result: GlobalResult | None = None) -> StartupResult:
        """Relocalise one scan. Pass ``global_result`` to reuse a cached global stage."""
        g = global_result if global_result is not None else self.global_reg.propose(scan_cam, self.map_ctx)
        result = StartupResult(g, times={"global_s": g.runtime_s})
        if g.failed or not g.candidates:
            result.reasons = ["global_failed"]
            return result
        t1 = time.perf_counter()
        scorer = ScanScorer(scan_cam, self.tree, self.frames, self.gate_cfg["features"])
        hyps = [(f"global_{i}", c.T) for i, c in enumerate(g.candidates[: self.cfg["max_global_candidates"]])]
        result.refined = self.refine_all(scan_cam, scorer, hyps)
        lead = max(range(len(result.refined)), key=lambda i: result.refined[i].score)
        result.refined += self.refine_all(scan_cam, scorer,
                                          challengers(self.frames, result.refined[lead].icp.T,
                                                      self.cfg["challengers"]))
        t2 = time.perf_counter()
        scores = [r.score for r in result.refined]
        poses = [r.icp.T for r in result.refined]
        result.best, result.second_score, result.ambiguity = ambiguity_ratio(
            scores, poses, self.frames, self.gate_cfg["distinct"])
        best = result.final
        result.accepted, result.reasons = decide(best.features, result.ambiguity, best.icp.converged, self.th)
        if best.icp.failed:
            result.accepted, result.reasons = False, ["fine_failed"]
        fine_s = sum(r.icp.runtime_s for r in result.refined)
        end = time.perf_counter()
        # gate_s = scan split + scoring + decision, i.e. everything after global that is not ICP.
        result.times.update(fine_s=fine_s, gate_s=(end - t1) - fine_s, total_s=g.runtime_s + (end - t1))
        return result
