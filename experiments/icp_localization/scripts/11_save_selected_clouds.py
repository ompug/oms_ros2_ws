#!/usr/bin/env python3
"""Save inspectable PCDs for selected baseline and failed perturbation runs."""
import argparse
import csv
import math
import pathlib
import numpy as np
from scipy.spatial.transform import Rotation
from cloud_io import read_pcd, write_pcd, xyz_array

ROOT = pathlib.Path(__file__).resolve().parents[1]
AXIS = np.array([[0., 1., 0.], [0., 0., 1.], [1., 0., 0.]])


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--results', type=pathlib.Path, default=ROOT / 'results/icp_results.csv')
    p.add_argument('--scans-dir', type=pathlib.Path, default=ROOT / 'scans')
    p.add_argument('--cases', nargs='+', default=['scan_006:baseline', 'scan_004:heading_20deg'])
    args = p.parse_args()
    wanted = {tuple(item.split(':', 1)) for item in args.cases}
    rows = [row for row in csv.DictReader(args.results.open(newline=''))
            if (row['snapshot_id'], row['mode']) in wanted and
            row['method'] == 'point_to_point' and row['failed'] == 'False']
    for row in rows:
        snapshot, mode = row['snapshot_id'], row['mode']
        _, pts = read_pcd(args.scans_dir / f'{snapshot}.pcd')
        raw = xyz_array(pts) @ AXIS.T
        intensity = pts['intensity'] if 'intensity' in pts.dtype.names else None
        ref_rot = Rotation.from_quat([float(row[f'reference_{k}']) for k in ('qx', 'qy', 'qz', 'qw')])
        heading = math.radians(float(row['initial_heading_offset_deg']))
        init_rot = Rotation.from_euler('y', heading) * ref_rot
        init_xyz = np.array([float(row[f'initial_{k}']) for k in 'xyz'])
        result_rot = Rotation.from_quat([float(row[k]) for k in ('qx', 'qy', 'qz', 'qw')])
        result_xyz = np.array([float(row[k]) for k in 'xyz'])
        prefix = args.results.parent / 'aligned_clouds' / 'point_to_point' / (
            snapshot if mode == 'baseline' else f'{snapshot}_{mode}')
        write_pcd(prefix.with_name(prefix.name+'_before.pcd'), init_rot.apply(raw)+init_xyz, intensity)
        write_pcd(prefix.with_name(prefix.name+'_after.pcd'), result_rot.apply(raw)+result_xyz, intensity)
        print('Saved', prefix)


if __name__ == '__main__':
    main()
