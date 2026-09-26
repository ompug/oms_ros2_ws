#!/usr/bin/env python3
"""Measure surrounding-map coverage and the number of 1 m pose voxels."""
import argparse
import json
import numpy as np
from cloud_io import bag_messages, cloud_arrays, message_stamp_ns


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--bag', required=True)
    args = p.parse_args()
    keyposes = {}
    surround = []
    for topic, msg, _ in bag_messages(args.bag, ['/key_pose_origin', '/laser_cloud_surround']):
        stamp = message_stamp_ns(msg)
        if topic == '/key_pose_origin':
            xyz, intensity = cloud_arrays(msg)
            keyposes[stamp] = (xyz, intensity)
        else:
            surround.append((stamp, msg.width * msg.height))
    rows = []
    for stamp, points in surround:
        if stamp not in keyposes:
            continue
        xyz, intensity = keyposes[stamp]
        if len(xyz) == 0:
            continue
        inside = xyz[np.linalg.norm(xyz - xyz[-1], axis=1) < 500]
        voxels = np.unique(np.floor(inside).astype(np.int32), axis=0)
        rows.append(dict(stamp_ns=stamp, map_points=points, saved_keyframes=len(xyz),
                         in_500m_radius=len(inside), selected_pose_voxels=len(voxels),
                         farthest_keyframe_m=float(np.max(np.linalg.norm(xyz-xyz[-1], axis=1)))))
    print(json.dumps(dict(surround_messages=len(surround), matched_messages=len(rows),
                          first=rows[0] if rows else None, last=rows[-1] if rows else None), indent=2))


if __name__ == '__main__':
    main()
