#!/usr/bin/env python3
"""Merge raw scans and processed LeGO odometry by SENSOR HEADER time into a small test bag.

Inputs remain read-only. Bag recording times differ by weeks; never synchronize on those.
"""
import argparse
from pathlib import Path
import sqlite3
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from ugv_relocalization.ros_common import stamp_to_ns


def records(bag, topic, cls):
    for path in sorted(Path(bag).glob('*.db3')):
        with sqlite3.connect(f'file:{path}?mode=ro',uri=True) as con:
            for data, in con.execute('SELECT data FROM messages JOIN topics ON messages.topic_id=topics.id WHERE topics.name=? ORDER BY timestamp',(topic,)):
                yield stamp_to_ns(deserialize_message(data,cls).header.stamp),bytes(data)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--raw',required=True); p.add_argument('--lego',required=True)
    p.add_argument('--output',required=True); p.add_argument('--duration',type=float,default=30.)
    a=p.parse_args()
    rows=[]; start=None
    for stamp,data in records(a.raw,'/rslidar_points',PointCloud2):
        start=stamp if start is None else start
        if stamp>start+a.duration*1e9:
            break
        rows.append((stamp,'/rslidar_points',data))
    if start is None:
        raise ValueError('raw bag has no LiDAR scans')
    for stamp,data in records(a.lego,'/integrated_to_init',Odometry):
        if stamp>start+a.duration*1e9:
            break
        if stamp>=start:
            rows.append((stamp,'/integrated_to_init',data))
    if not any(t=='/integrated_to_init' for _,t,_ in rows):
        raise ValueError('no overlapping sensor header time in LeGO bag')
    writer=rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=a.output,storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
    for name,typ in [('/rslidar_points','sensor_msgs/msg/PointCloud2'),('/integrated_to_init','nav_msgs/msg/Odometry')]:
        writer.create_topic(rosbag2_py.TopicMetadata(name=name,type=typ,serialization_format='cdr'))
    for stamp,topic,data in sorted(rows,key=lambda x:x[0]):
        writer.write(topic,data,stamp)
    del writer
    print(f'{a.output}: {len(rows)} messages, first header {start}; original message headers unchanged')


if __name__=='__main__':
    main()
