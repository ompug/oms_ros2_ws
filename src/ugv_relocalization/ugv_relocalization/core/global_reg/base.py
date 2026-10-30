"""Interface shared by every global registration method.

A method receives only the scan (LeGO camera axes, NaNs allowed) and the map
context. It returns ranked pose candidates T (camera_init <- camera). Scores
are method-specific and NOT comparable across methods; the verification gate
re-scores every candidate with one common metric after fine ICP.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Protocol

import numpy as np

from ..mapctx import MapContext


@dataclass
class Candidate:
    """One pose hypothesis from a global method."""

    T: np.ndarray
    score: float
    info: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.T = np.array(self.T, dtype=float)  # own a writable copy (libraries return views)


@dataclass
class GlobalResult:
    """Ranked candidates (best first) plus timing."""

    method: str
    candidates: list[Candidate]
    runtime_s: float
    stage_times: dict[str, float] = field(default_factory=dict)
    failed: bool = False
    failure_reason: str = ""

    @property
    def best(self) -> Candidate | None:
        """Highest-ranked candidate, or None when the method produced nothing."""
        return self.candidates[0] if self.candidates else None


class GlobalRegistrar(Protocol):
    """``propose(scan_cam, map_ctx) -> GlobalResult``; must never see reference poses."""

    name: str

    def propose(self, scan_cam: np.ndarray, map_ctx: MapContext) -> GlobalResult: ...
