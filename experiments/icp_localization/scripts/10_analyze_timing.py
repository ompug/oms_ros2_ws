#!/usr/bin/env python3
"""Supplement nearest-pose metrics with interpolation and local motion rates."""
import argparse
import csv
import json
import math
import pathlib
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

ROOT = pathlib.Path(__file__).resolve().parents[1]
MAP_ROT = Rotation.from_euler('ZYX', [math.pi/2, 0, math.pi/2]).as_matrix()


def wrap(value):
    return (value + 180) % 360 - 180


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--trajectory', type=pathlib.Path, default=ROOT / 'trajectory/aft_mapped_to_init.csv')
    p.add_argument('--results', type=pathlib.Path, default=ROOT / 'results/icp_results.csv')
    p.add_argument('--output', type=pathlib.Path, default=ROOT / 'results/timestamp_sensitivity.csv')
    args = p.parse_args()
    poses = list(csv.DictReader(args.trajectory.open(newline='')))
    stamps = np.array([int(r['timestamp_ns']) for r in poses], dtype=np.int64)
    xyz = np.array([[float(r[k]) for k in 'xyz'] for r in poses])
    quat = np.array([[float(r[k]) for k in ('qx', 'qy', 'qz', 'qw')] for r in poses])
    results = [r for r in csv.DictReader(args.results.open(newline=''))
               if r['mode'] == 'baseline' and r['failed'] == 'False']
    rows = []
    for run in results:
        stamp = int(run['scan_timestamp_ns'])
        upper = int(np.clip(np.searchsorted(stamps, stamp), 1, len(stamps)-1))
        lower = upper - 1
        alpha = np.clip((stamp-stamps[lower]) / (stamps[upper]-stamps[lower]), 0, 1)
        interpolated_xyz = xyz[lower] * (1-alpha) + xyz[upper] * alpha
        interp_rot = Slerp([0, 1], Rotation.from_quat(quat[[lower, upper]]))([alpha])[0]
        icp_xyz = np.array([float(run[k]) for k in 'xyz'])
        icp_rot = Rotation.from_quat([float(run[k]) for k in ('qx', 'qy', 'qz', 'qw')])
        icp_yaw = Rotation.from_matrix(MAP_ROT @ icp_rot.as_matrix()).as_euler('xyz', degrees=True)[2]
        interp_yaw = Rotation.from_matrix(MAP_ROT @ interp_rot.as_matrix()).as_euler('xyz', degrees=True)[2]
        dt = (stamps[upper]-stamps[lower])/1e9
        angle_change = (Rotation.from_quat(quat[upper]) * Rotation.from_quat(quat[lower]).inv()).magnitude()
        rows.append(dict(snapshot_id=run['snapshot_id'], method=run['method'],
                         nearest_delta_ms=run['timestamp_delta_ms'],
                         bracket_gap_ms=dt*1000,
                         local_translation_rate_m_s=np.linalg.norm(xyz[upper]-xyz[lower])/dt,
                         local_rotation_rate_deg_s=np.degrees(angle_change)/dt,
                         nearest_xy_error_m=run['translation_error_xy_m'],
                         interpolated_xy_error_m=np.linalg.norm((MAP_ROT @ (icp_xyz-interpolated_xyz))[:2]),
                         nearest_yaw_error_deg=run['yaw_error_deg'],
                         interpolated_yaw_error_deg=wrap(icp_yaw-interp_yaw)))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader(); writer.writerows(rows)
    summary_path = args.results.parent / 'summary.json'
    summary = json.loads(summary_path.read_text())
    for method in sorted(set(row['method'] for row in rows)):
        group = [row for row in rows if row['method'] == method]
        summary[method]['interpolated_reference'] = {}
        for key in ('interpolated_xy_error_m', 'interpolated_yaw_error_deg'):
            values = np.array([row[key] for row in group], dtype=float)
            if key.endswith('yaw_error_deg'):
                values = np.abs(values)
            summary[method]['interpolated_reference'][key] = dict(
                mean=float(values.mean()), median=float(np.median(values)),
                rmse=float(np.sqrt(np.mean(values**2))), max=float(values.max()))
    summary_path.write_text(json.dumps(summary, indent=2) + '\n')
    for row in rows:
        print(row['snapshot_id'], row['method'], 'nearest yaw', round(float(row['nearest_yaw_error_deg']), 2),
              'interpolated yaw', round(row['interpolated_yaw_error_deg'], 2),
              'turn rate', round(row['local_rotation_rate_deg_s'], 2))


if __name__ == '__main__':
    main()
