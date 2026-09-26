#!/usr/bin/env python3
"""Quantify changes from each keyframe's first published position to its final one."""
import argparse
import csv
import json
import pathlib
import numpy as np
from cloud_io import bag_messages, cloud_arrays, message_stamp_ns

ROOT = pathlib.Path(__file__).resolve().parents[1]


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--bag', type=pathlib.Path, default=ROOT / 'bags/lego_2026_09_11_full')
    p.add_argument('--scans', type=pathlib.Path, default=ROOT / 'metadata/scans.csv')
    p.add_argument('--output', type=pathlib.Path, default=ROOT / 'metadata/pose_revisions.json')
    args = p.parse_args()
    stamps = []
    original = []
    final = None
    for _, msg, _ in bag_messages(args.bag, ['/key_pose_origin']):
        xyz, _ = cloud_arrays(msg)
        stamp = message_stamp_ns(msg)
        for index in range(len(original), len(xyz)):
            original.append(xyz[index].copy())
            stamps.append(stamp)
        final = xyz
    if final is None or len(final) != len(original):
        raise SystemExit('Keypose history was incomplete')
    revision = np.linalg.norm(final - np.array(original), axis=1)
    scans = list(csv.DictReader(args.scans.open(newline='')))
    sampled = {}
    for scan in scans:
        index = int(np.argmin(np.abs(np.array(stamps, dtype=np.int64) - int(scan['timestamp_ns']))))
        sampled[scan['snapshot_id']] = dict(nearest_keyframe_index=index,
                                             keyframe_stamp_ns=stamps[index],
                                             time_delta_ms=abs(stamps[index]-int(scan['timestamp_ns']))/1e6,
                                             exact_keyframe=stamps[index] == int(scan['timestamp_ns']),
                                             position_revision_m=float(revision[index]))
    result = dict(keyframe_count=len(final),
                  position_revision_percentiles_m=np.percentile(revision, [0, 50, 90, 99, 100]).tolist(),
                  keyframes_revised_over_0_1m=int((revision > .1).sum()),
                  keyframes_revised_over_0_5m=int((revision > .5).sum()),
                  sampled_scans=sampled)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
