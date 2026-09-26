#!/usr/bin/env python3
"""Numerical checks for frozen map and final optimized keyframe path."""
import argparse
import json
import pathlib
import numpy as np
from scipy.spatial import cKDTree
from cloud_io import bag_messages, cloud_arrays, read_pcd, xyz_array

ROOT = pathlib.Path(__file__).resolve().parents[1]


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--map', type=pathlib.Path, default=ROOT / 'map/reference_map.pcd')
    p.add_argument('--bag', type=pathlib.Path, default=ROOT / 'bags/lego_2026_09_11_full')
    p.add_argument('--output', type=pathlib.Path, default=ROOT / 'metadata/map_validation.json')
    args = p.parse_args()
    _, pts = read_pcd(args.map)
    xyz = xyz_array(pts)
    valid = xyz[np.isfinite(xyz).all(axis=1)]
    keyposes = None
    for _, msg, _ in bag_messages(args.bag, ['/key_pose_origin']):
        keyposes, _ = cloud_arrays(msg)
    if keyposes is None:
        raise SystemExit('No keyposes')
    steps = np.linalg.norm(np.diff(keyposes, axis=0), axis=1)
    tree = cKDTree(valid)
    sample = valid[::max(1, len(valid)//30000)]
    distances, _ = tree.query(sample, k=2, workers=2)
    nearest = distances[:, 1]
    result = {
        'map_points': len(xyz), 'finite_points': len(valid),
        'bbox_min_camera_init_m': valid.min(axis=0).tolist(),
        'bbox_max_camera_init_m': valid.max(axis=0).tolist(),
        'height_y_percentiles_m': np.percentile(valid[:, 1], [0, 1, 25, 50, 75, 99, 100]).tolist(),
        'sample_nearest_neighbor_percentiles_m': np.percentile(nearest, [50, 90, 99, 100]).tolist(),
        'sample_isolated_over_2m': int((nearest > 2).sum()),
        'sample_count': len(sample), 'saved_keyframes': len(keyposes),
        'keyframe_step_percentiles_m': np.percentile(steps, [0, 50, 90, 99, 100]).tolist(),
        'keyframe_steps_over_5m': int((steps > 5).sum()),
        'max_keyframe_distance_from_final_m': float(np.linalg.norm(keyposes-keyposes[-1], axis=1).max())
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
