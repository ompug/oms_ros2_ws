#!/usr/bin/env python3
"""Export LeGO odometry using message header stamps and ROS ZYX Euler angles."""
import argparse
import csv
import pathlib
from scipy.spatial.transform import Rotation
from cloud_io import bag_messages, message_stamp_ns


FIELDS = ['timestamp_ns', 'timestamp_sec', 'frame_id', 'child_frame_id',
          'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'roll', 'pitch', 'yaw']


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--bag', required=True)
    p.add_argument('--output', required=True)
    args = p.parse_args()
    out = pathlib.Path(args.output)
    if out.exists():
        raise SystemExit(f'Refusing to overwrite {out}')
    out.parent.mkdir(parents=True, exist_ok=True)
    count = 0
    with out.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        for _, msg, _ in bag_messages(args.bag, ['/aft_mapped_to_init']):
            pose = msg.pose.pose
            q = pose.orientation
            quat = [q.x, q.y, q.z, q.w]
            roll, pitch, yaw = Rotation.from_quat(quat).as_euler('xyz')
            stamp = message_stamp_ns(msg)
            writer.writerow(dict(timestamp_ns=stamp, timestamp_sec=f'{stamp/1e9:.9f}',
                                 frame_id=msg.header.frame_id, child_frame_id=msg.child_frame_id,
                                 x=pose.position.x, y=pose.position.y, z=pose.position.z,
                                 qx=q.x, qy=q.y, qz=q.z, qw=q.w,
                                 roll=roll, pitch=pitch, yaw=yaw))
            count += 1
    print(f'Exported {count} poses to {out}')


if __name__ == '__main__':
    main()
