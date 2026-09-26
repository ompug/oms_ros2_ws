#!/usr/bin/env python3
import argparse
import csv
import pathlib
import numpy as np


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--scans', required=True)
    p.add_argument('--trajectory', required=True)
    p.add_argument('--output', required=True)
    p.add_argument('--max-delta-ms', type=float, default=300.0)
    args = p.parse_args()
    scans = list(csv.DictReader(open(args.scans, newline='')))
    poses = list(csv.DictReader(open(args.trajectory, newline='')))
    if not poses:
        raise SystemExit('Empty trajectory')
    stamps = np.array([int(pose['timestamp_ns']) for pose in poses], dtype=np.int64)
    if np.any(np.diff(stamps) < 0):
        raise SystemExit('Trajectory header stamps are not monotonic')
    rows = []
    for scan in scans:
        stamp = int(scan['timestamp_ns'])
        index = int(np.searchsorted(stamps, stamp))
        nearest = min((max(0, index - 1), min(len(poses)-1, index)),
                      key=lambda i: abs(int(stamps[i]) - stamp))
        pose = poses[nearest]
        delta_ms = abs(int(stamps[nearest]) - stamp) / 1e6
        row = dict(scan, pose_timestamp_ns=pose['timestamp_ns'],
                   abs_timestamp_delta_ms=f'{delta_ms:.3f}',
                   reference_frame_id=pose['frame_id'],
                   reference_child_frame_id=pose['child_frame_id'])
        row.update({f'reference_{key}': pose[key] for key in
                    ['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'roll', 'pitch', 'yaw']})
        row['match_valid'] = delta_ms <= args.max_delta_ms
        rows.append(row)
        print(scan['snapshot_id'], f'{delta_ms:.1f} ms', 'OK' if row['match_valid'] else 'REJECT')
    out = pathlib.Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)


if __name__ == '__main__':
    main()
