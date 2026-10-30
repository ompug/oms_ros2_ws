"""Installed version of the experiment's multistart -> fine -> structure/ambiguity gate."""
from pathlib import Path
import numpy as np
import yaml
from .core.frames import Frames
from .core.io import read_pcd
from .core.mapctx import MapContext
from .core.fine.open3d_icp import Open3DPointToPlane
from .core.global_reg.multistart import MultiStartSearch
from .core.global_reg.base import GlobalResult, Candidate
from .core.pipeline import StartupPipeline


def load(path):
    with open(path) as f:
        return yaml.safe_load(f)


def build_fine(cfg, ctx):
    return Open3DPointToPlane(cfg, ctx)


class Engine:
    def __init__(self, config_dir, map_path):
        c = Path(config_dir)
        self.frames = Frames.from_config(load(c/'frames.yaml'))
        self.points = read_pcd(map_path)[0]
        self.points = self.points[np.isfinite(self.points).all(axis=1)]
        if len(self.points) < 100:
            raise ValueError('reference map is empty or too small')
        self.ctx = MapContext(self.points, self.frames)
        self.gate = load(c/'gate.yaml')
        self.tree = self.ctx.kdtree(self.gate['features']['map_voxel'])
        self.fine_cfg = load(c/'fine.yaml')
        self.fine = build_fine(self.fine_cfg, self.ctx)
        self.pipeline = StartupPipeline(None, self.fine, self.ctx, self.tree,
            load(c/'pipeline.yaml'), self.gate, self.gate['thresholds'])
        self.multi = None
        self.multi_cfg = load(c/'multistart.yaml')

    def startup(self, scan_cam, mode, seed):
        if mode == 'rough':
            g = GlobalResult('rough_fine_only', [Candidate(self.frames.from_map(seed), 1.)], 0.)
        elif mode == 'multistart':
            if self.multi is None:
                cfg = dict(self.multi_cfg)
                cfg.pop('method')
                self.multi = MultiStartSearch(**cfg)
            g = self.multi.propose(scan_cam, self.ctx)
        else:
            raise ValueError(mode)
        result = self.pipeline.run(scan_cam, g)
        return result, (self.frames.to_map(result.final.icp.T) if result.final else None)
