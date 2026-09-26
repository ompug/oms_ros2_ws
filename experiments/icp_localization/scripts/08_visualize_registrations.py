#!/usr/bin/env python3
"""Overlay saved before/after scan clouds on a local frozen-map window."""
import argparse
import pathlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from cloud_io import read_pcd, xyz_array

ROOT = pathlib.Path(__file__).resolve().parents[1]
MAP_ROT = Rotation.from_euler('ZYX', [np.pi/2, 0, np.pi/2]).as_matrix()


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--map', type=pathlib.Path, default=ROOT / 'map/reference_map.pcd')
    p.add_argument('--results-dir', type=pathlib.Path, default=ROOT / 'results')
    p.add_argument('--method', default='point_to_point')
    p.add_argument('--snapshots', nargs='+', default=['scan_001', 'scan_005', 'scan_010'])
    args = p.parse_args()
    _, cloud = read_pcd(args.map)
    map_xyz = xyz_array(cloud) @ MAP_ROT.T
    output = args.results_dir / 'plots'
    output.mkdir(parents=True, exist_ok=True)
    for snapshot in args.snapshots:
        prefix = args.results_dir / 'aligned_clouds' / args.method / snapshot
        before = prefix.with_name(prefix.name + '_before.pcd')
        after = prefix.with_name(prefix.name + '_after.pcd')
        if not before.exists() or not after.exists():
            print('Missing saved aligned clouds for', snapshot)
            continue
        _, b = read_pcd(before)
        _, a = read_pcd(after)
        before_xyz = xyz_array(b) @ MAP_ROT.T
        after_xyz = xyz_array(a) @ MAP_ROT.T
        center = np.median(after_xyz[:, :2], axis=0)
        local = map_xyz[np.linalg.norm(map_xyz[:, :2] - center, axis=1) < 35]
        fig, axes = plt.subplots(1, 2, figsize=(12, 5), sharex=True, sharey=True)
        for ax, scan, label in [(axes[0], before_xyz, 'Initial pose'),
                                 (axes[1], after_xyz, 'After ICP')]:
            ax.scatter(local[:, 0], local[:, 1], s=1, c='0.55', alpha=.45, rasterized=True)
            ax.scatter(scan[::2, 0], scan[::2, 1], s=1, c='crimson', alpha=.5, rasterized=True)
            ax.set(xlabel='Map X (m)', ylabel='Map Y (m)', title=label,
                   xlim=(center[0]-35, center[0]+35), ylim=(center[1]-35, center[1]+35))
            ax.set_aspect('equal'); ax.grid(alpha=.2)
        fig.suptitle(f'{snapshot}: gray map, red scan ({args.method})')
        fig.tight_layout(); fig.savefig(output / f'{snapshot}_{args.method}_overlay.png', dpi=160)
        plt.close(fig)
    print('Wrote overlays to', output)


if __name__ == '__main__':
    main()
