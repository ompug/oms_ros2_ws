"""Registration result model retained from the experiment; source-built backends omitted."""
from __future__ import annotations

import pathlib
import time
from dataclasses import dataclass, field
from typing import Sequence

import numpy as np



@dataclass
class ICPResult:
    """Outcome of one registration call. ``T`` is camera_init <- camera."""

    T: np.ndarray
    iterations: int
    iteration_limit_reached: bool
    runtime_s: float
    match_ratio: float
    source_points_filtered: int
    map_points_filtered: int
    failed: bool = False
    failure_reason: str = ""
    stages: list["ICPResult"] = field(default_factory=list)

    def __post_init__(self) -> None:
        # Libraries may hand back read-only views (Open3D); own a float64 copy.
        self.T = np.array(self.T, dtype=float)

    @property
    def converged(self) -> bool:
        """Phase 1 definition: a finite pose and the iteration cap not hit."""
        return not self.failed and not self.iteration_limit_reached
