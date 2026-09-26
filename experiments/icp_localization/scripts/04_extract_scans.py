#!/usr/bin/env python3
"""Sample raw RoboSense messages across the run; preserve header stamps/frames in CSV."""
import argparse
import csv
import pathlib
import numpy as np
from cloud_io import bag_messages, cloud_arrays, message_stamp_ns, write_pcd


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--bag', required=True)
    p.add_argument('--output-dir', required=True)
    p.add_argument('--count', type=int, default=10)
    args = p.parse_args()
    out = pathlib.Path(args.output_dir)
    if list(out.glob('scan_*.pcd')):
        raise SystemExit(f'Refusing to overwrite existing scans in {out}')
    stamps = [message_stamp_ns(msg) for _, msg, _ in bag_messages(args.bag, ['/rslidar_points'])]
    if len(stamps) < args.count:
        raise SystemExit(f'Only {len(stamps)} scans found')
    targets = np.linspace(0.05, 0.95, args.count)
    indices = set(np.unique(np.rint(targets * (len(stamps) - 1)).astype(int)))
    out.mkdir(parents=True, exist_ok=True)
    meta = out.parent / 'metadata' / 'scans.csv'
    meta.parent.mkdir(parents=True, exist_ok=True)
    with meta.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=['snapshot_id', 'timestamp_ns',
                                'timestamp_sec', 'pcd_file', 'frame_id', 'point_count'])
        writer.writeheader()
        for index, (_, msg, _) in enumerate(bag_messages(args.bag, ['/rslidar_points'])):
            if index not in indices:
                continue
            xyz, intensity = cloud_arrays(msg)
            stamp = message_stamp_ns(msg)
            name = f'scan_{len(list(out.glob("scan_*.pcd"))) + 1:03d}'
            file = out / f'{name}.pcd'
            write_pcd(file, xyz, intensity)
            writer.writerow(dict(snapshot_id=name, timestamp_ns=stamp,
                                 timestamp_sec=f'{stamp/1e9:.9f}', pcd_file=str(file.resolve()),
                                 frame_id=msg.header.frame_id, point_count=len(xyz)))
            print(name, stamp, msg.header.frame_id, len(xyz))
    print(f'Saved {len(indices)} scans; metadata: {meta}')


if __name__ == '__main__':
    main()
