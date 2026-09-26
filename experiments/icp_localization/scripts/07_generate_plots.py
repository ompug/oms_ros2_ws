#!/usr/bin/env python3
import argparse
import csv
import json
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
    p.add_argument('--results', type=pathlib.Path, default=ROOT / 'results/icp_results.csv')
    p.add_argument('--trajectory', type=pathlib.Path, default=ROOT / 'trajectory/aft_mapped_to_init.csv')
    p.add_argument('--map', type=pathlib.Path, default=ROOT / 'map/reference_map.pcd')
    args = p.parse_args()
    out = args.results.parent / 'plots'
    out.mkdir(parents=True, exist_ok=True)
    rows = list(csv.DictReader(args.results.open(newline='')))
    baseline = [r for r in rows if r['mode'] == 'baseline' and r['failed'].lower() == 'false']
    p2p = [r for r in baseline if r['method'] == 'point_to_point']
    plane = [r for r in baseline if r['method'] == 'point_to_plane']
    coarse = [r for r in baseline if r['method'] == 'point_to_point_coarse']
    traj = list(csv.DictReader(args.trajectory.open(newline='')))
    coords = np.array([[float(r[k]) for k in 'xyz'] for r in traj]) @ MAP_ROT.T
    fig, ax = plt.subplots(figsize=(8, 7))
    ax.plot(coords[:, 0], coords[:, 1], 'k-', lw=1, label='LeGO trajectory')
    for group, marker, label in [(p2p, 'o', 'Point-to-point ICP'), (plane, 'x', 'Point-to-plane ICP'),
                                 (coarse, '+', 'Coarse point-to-point ICP')]:
        ax.scatter([float(r['icp_map_x_m']) for r in group],
                   [float(r['icp_map_y_m']) for r in group], marker=marker, s=45, label=label)
    ax.set(xlabel='Map X (m)', ylabel='Map Y (m)', title='LeGO reference and ICP estimates')
    ax.axis('equal'); ax.grid(True); ax.legend(); fig.tight_layout()
    fig.savefig(out / 'trajectory_xy.png', dpi=160); plt.close(fig)

    for key, ylabel, filename in [
        ('translation_error_xy_m', 'Horizontal error vs LeGO (m)', 'translation_error.png'),
        ('yaw_error_deg', 'Yaw error vs LeGO (deg)', 'yaw_error.png'),
        ('runtime_s', 'ICP compute time (s)', 'runtime.png'),
        ('fitness_rmse_m', 'Inlier nearest-map RMSE (m)', 'fitness.png')]:
        fig, ax = plt.subplots(figsize=(9, 4))
        for group, label in [(p2p, 'Point-to-point'), (plane, 'Point-to-plane'),
                             (coarse, 'Coarse point-to-point')]:
            if group:
                ax.plot([r['snapshot_id'] for r in group],
                        [float(r[key]) for r in group], marker='o', label=label)
        ax.set(xlabel='Snapshot', ylabel=ylabel)
        ax.grid(True); ax.legend(); fig.autofmt_xdate(); fig.tight_layout()
        fig.savefig(out / filename, dpi=160); plt.close(fig)

    offsets = [f'dx_{v:g}m' for v in (.25, .5, 1.)] + [f'heading_{v:g}deg' for v in (5., 10., 20.)]
    with (args.results.parent / 'summary.json').open() as stream:
        summary = json.load(stream)
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.bar(offsets, [summary[k]['same_solution']/max(1, summary[k]['total']) for k in offsets])
    ax.set(xlabel='Initial perturbation', ylabel='Fraction reaching baseline solution', ylim=(0, 1.05))
    ax.grid(axis='y'); fig.autofmt_xdate(); fig.tight_layout()
    fig.savefig(out / 'capture_range.png', dpi=160); plt.close(fig)

    timing_file = args.results.parent / 'timestamp_sensitivity.csv'
    if timing_file.exists():
        timing = [row for row in csv.DictReader(timing_file.open(newline=''))
                  if row['method'] == 'point_to_point']
        fig, axes = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
        labels = [row['snapshot_id'] for row in timing]
        axes[0].plot(labels, [float(row['nearest_xy_error_m']) for row in timing], 'o-', label='Nearest')
        axes[0].plot(labels, [float(row['interpolated_xy_error_m']) for row in timing], 'o-', label='Interpolated')
        axes[0].set(ylabel='Horizontal error vs LeGO (m)')
        axes[1].plot(labels, [float(row['nearest_yaw_error_deg']) for row in timing], 'o-', label='Nearest')
        axes[1].plot(labels, [float(row['interpolated_yaw_error_deg']) for row in timing], 'o-', label='Interpolated')
        axes[1].set(xlabel='Snapshot', ylabel='Yaw error vs LeGO (deg)')
        for ax in axes:
            ax.grid(True); ax.legend()
        fig.suptitle('Sensitivity to LeGO timestamp alignment')
        fig.autofmt_xdate(); fig.tight_layout()
        fig.savefig(out / 'timestamp_sensitivity.png', dpi=160); plt.close(fig)

    _, points = read_pcd(args.map)
    xyz = xyz_array(points) @ MAP_ROT.T
    if len(xyz) > 300000:
        xyz = xyz[::int(np.ceil(len(xyz)/300000))]
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    im = axes[0].scatter(xyz[:, 0], xyz[:, 1], c=xyz[:, 2], s=.5, cmap='viridis', rasterized=True)
    axes[0].set(xlabel='Map X (m)', ylabel='Map Y (m)', title='Full LeGO map, top view')
    axes[0].axis('equal'); fig.colorbar(im, ax=axes[0], label='Height (m)')
    axes[1].scatter(xyz[:, 0], xyz[:, 2], s=.5, alpha=.25, rasterized=True)
    axes[1].set(xlabel='Map X (m)', ylabel='Height (m)', title='Side projection')
    fig.tight_layout(); fig.savefig(out / 'map_overview.png', dpi=160); plt.close(fig)
    print(f'Saved plots to {out}')


if __name__ == '__main__':
    main()
