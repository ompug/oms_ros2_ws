#!/usr/bin/env python3
"""Export the explicitly requested all-keyframe LeGO cloud."""
import argparse
import datetime
import pathlib
import shutil
import numpy as np
from cloud_io import bag_messages, cloud_arrays, message_stamp_ns, write_pcd


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--bag', required=True)
    p.add_argument('--output', required=True)
    p.add_argument('--prior-pcd')
    p.add_argument('--topic', default='/laser_cloud_full', choices=['/laser_cloud_full', '/laser_cloud_surround'])
    args = p.parse_args()
    out = pathlib.Path(args.output)
    if out.exists():
        raise SystemExit(f'Refusing to overwrite {out}')
    if args.prior_pcd:
        prior = out.parent / 'previous_surround_export.pcd'
        if not prior.exists():
            shutil.copy2(args.prior_pcd, prior)
    last_map = last_poses = None
    map_count = 0
    for topic, msg, _ in bag_messages(args.bag, [args.topic, '/key_pose_origin']):
        if topic == args.topic:
            last_map = msg
            map_count += 1
        else:
            last_poses = msg
    if last_map is None or last_poses is None:
        raise SystemExit(f'Processed bag lacks {args.topic} or keypose cloud')
    xyz, intensity = cloud_arrays(last_map)
    poses, _ = cloud_arrays(last_poses)
    finite = np.isfinite(xyz).all(axis=1)
    xyz, intensity = xyz[finite], intensity[finite] if intensity is not None else None
    if len(xyz) == 0 or len(poses) == 0:
        raise SystemExit('Empty map or keypose cloud')
    dist = np.linalg.norm(poses - poses[-1], axis=1)
    max_dist = float(dist.max())
    write_pcd(out, xyz, intensity)
    metadata = out.with_name(out.stem + '_metadata.txt')
    metadata.write_text(
        f'generated_utc: {datetime.datetime.now(datetime.timezone.utc).isoformat()}\n'
        f'source_bag: {args.bag}\nsource_topic: {args.topic}\n'
        f'procedure: {"/export_full_map after playback; all saved corner, surface and outlier keyframes" if args.topic == "/laser_cloud_full" else "last radius-limited, keypose-downsampled visualization cloud"}; 0.4 m voxel\n'
        f'frame_id: {last_map.header.frame_id}\nstamp_ns: {message_stamp_ns(last_map)}\n'
        f'map_messages: {map_count}\nkeyposes_at_end: {len(poses)}\n'
        f'max_keypose_distance_from_final_m: {max_dist:.6f}\n'
        f'point_count: {len(xyz)}\nfields: x y z' + (' intensity' if intensity is not None else '') + '\n'
        f'voxel_resolution_m: 0.4 output cloud' +
        ('; 1.0 keypose selection\n' if args.topic == '/laser_cloud_surround' else '; all keyframes\n') +
        f'bbox_min_m: {xyz.min(axis=0).tolist()}\n'
        f'bbox_max_m: {xyz.max(axis=0).tolist()}\n')
    print(metadata.read_text())


if __name__ == '__main__':
    main()
