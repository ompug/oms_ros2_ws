"""File I/O: PCD clouds, scan metadata, YAML configs and result folders."""
from __future__ import annotations

import csv
import datetime as _dt
import json
import pathlib
import shutil
import subprocess
from dataclasses import dataclass
from typing import Any, Iterable, Mapping

import numpy as np
import yaml

from ament_index_python.packages import get_package_share_directory
PACKAGE_ROOT = pathlib.Path(get_package_share_directory("ugv_relocalization"))
WORKSPACE_ROOT = PACKAGE_ROOT
CONFIG_DIR = PACKAGE_ROOT / "config"

_PCD_TYPES = {"F": "f", "I": "i", "U": "u"}


def read_pcd(path: str | pathlib.Path) -> tuple[np.ndarray, np.ndarray | None]:
    """Read a binary or ASCII PCD. Returns (xyz float64 (N,3), intensity or None).

    Nonfinite rows are kept; callers decide when to remove them.
    """
    path = pathlib.Path(path)
    with path.open("rb") as stream:
        header: dict[str, list[str]] = {}
        while True:
            raw = stream.readline()
            if not raw:
                raise ValueError(f"Incomplete PCD header: {path}")
            line = raw.decode("ascii").strip()
            if not line or line.startswith("#"):
                continue
            key, *values = line.split()
            header[key.upper()] = values
            if key.upper() == "DATA":
                break
        fields = header["FIELDS"]
        sizes = [int(s) for s in header["SIZE"]]
        counts = [int(c) for c in header.get("COUNT", ["1"] * len(fields))]
        if any(c != 1 for c in counts):
            raise ValueError(f"Multi-count PCD fields are unsupported: {path}")
        dtype = np.dtype([(f, _PCD_TYPES[t] + str(s))
                          for f, t, s in zip(fields, header["TYPE"], sizes)])
        count = int(header["POINTS"][0])
        encoding = header["DATA"][0].lower()
        if encoding == "binary":
            points = np.fromfile(stream, dtype=dtype, count=count)
        elif encoding == "ascii":
            rows = np.loadtxt(stream, dtype=np.float64, ndmin=2)
            points = np.empty(len(rows), dtype=dtype)
            for i, field in enumerate(fields):
                points[field] = rows[:, i]
        else:
            raise ValueError(f"Unsupported PCD DATA {encoding}: {path}")
    if len(points) != count:
        raise ValueError(f"Expected {count} points, found {len(points)}: {path}")
    xyz = np.column_stack([points[a] for a in ("x", "y", "z")]).astype(np.float64)
    intensity = points["intensity"].astype(np.float32) if "intensity" in fields else None
    return xyz, intensity


def load_yaml(path: str | pathlib.Path) -> dict[str, Any]:
    """Load a YAML mapping."""
    with pathlib.Path(path).open() as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"{path} is not a YAML mapping")
    return data


def resolve(path: str | pathlib.Path) -> pathlib.Path:
    """Resolve a config path: absolute stays, relative is from the workspace root."""
    path = pathlib.Path(path)
    return path if path.is_absolute() else WORKSPACE_ROOT / path


def config_path(name: str | pathlib.Path) -> pathlib.Path:
    """Resolve a config file: absolute, workspace-relative, or inside configs/."""
    path = pathlib.Path(name)
    if path.is_absolute():
        return path
    for candidate in (CONFIG_DIR / path, WORKSPACE_ROOT / path, pathlib.Path.cwd() / path):
        if candidate.exists():
            return candidate
    raise FileNotFoundError(f"Config not found: {name}")
