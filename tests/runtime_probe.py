#!/usr/bin/env python3
"""Publish deterministic VLP-16-like scans and measure LeGO-LOAM outputs."""

import argparse
import json
import math
import struct
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from tf2_msgs.msg import TFMessage


FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
]


def cloud_message(node: Node, points: list[tuple[float, float, float, float]]) -> PointCloud2:
    msg = PointCloud2()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = "velodyne"
    msg.height = 1
    msg.width = len(points)
    msg.fields = FIELDS
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = msg.point_step * msg.width
    msg.data = b"".join(struct.pack("<ffff", *point) for point in points)
    msg.is_dense = all(math.isfinite(value) for point in points for value in point[:3])
    return msg


def synthetic_scan(scan_index: int) -> list[tuple[float, float, float, float]]:
    points = []
    for column in range(720):
        azimuth = 2.0 * math.pi * column / 720.0
        sector = column // 90
        wall_range = (10.0, 15.0, 8.0, 18.0, 12.0, 7.0, 16.0, 11.0)[sector]
        wall_range += 0.02 * scan_index * math.cos(azimuth)
        for ring in range(16):
            vertical = math.radians(-15.0 + 2.0 * ring)
            if ring <= 7:
                horizontal_range = 1.2 / max(-math.tan(vertical), 0.02)
                z = -1.2
            else:
                horizontal_range = wall_range
                z = horizontal_range * math.tan(vertical)
            x = horizontal_range * math.cos(azimuth)
            y = horizontal_range * math.sin(azimuth)
            points.append((x, y, z, float((column + ring) % 255)))
    return points


class Probe(Node):
    def __init__(self, reliability: str) -> None:
        super().__init__(f"lego_runtime_probe_{reliability}")
        policy = (
            ReliabilityPolicy.BEST_EFFORT
            if reliability == "best_effort"
            else ReliabilityPolicy.RELIABLE
        )
        input_qos = QoSProfile(
            reliability=policy,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        output_qos = QoSProfile(depth=20)
        self.publisher = self.create_publisher(PointCloud2, "/velodyne_points", input_qos)
        self.counts: dict[str, int] = {}
        self.nonempty: dict[str, int] = {}
        self.odom_stamps: list[int] = []
        self.odom_valid = True
        self.tf_parents: dict[str, set[str]] = {}

        for topic in (
            "/segmented_cloud",
            "/ground_cloud",
            "/laser_cloud_sharp",
            "/laser_cloud_less_sharp",
            "/laser_cloud_flat",
            "/laser_cloud_less_flat",
            "/key_pose_origin",
            "/laser_cloud_surround",
        ):
            self.create_subscription(
                PointCloud2,
                topic,
                lambda msg, topic=topic: self.cloud_callback(topic, msg),
                output_qos,
            )
        self.create_subscription(TFMessage, "/tf", self.tf_callback, output_qos)
        static_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self.create_subscription(TFMessage, "/tf_static", self.tf_callback, static_qos)
        for topic in ("/laser_odom_to_init", "/aft_mapped_to_init", "/integrated_to_init"):
            self.create_subscription(
                Odometry,
                topic,
                lambda msg, topic=topic: self.odom_callback(topic, msg),
                output_qos,
            )

    def cloud_callback(self, topic: str, msg: PointCloud2) -> None:
        self.counts[topic] = self.counts.get(topic, 0) + 1
        if msg.width * msg.height > 0:
            self.nonempty[topic] = self.nonempty.get(topic, 0) + 1

    def odom_callback(self, topic: str, msg: Odometry) -> None:
        self.counts[topic] = self.counts.get(topic, 0) + 1
        pose = msg.pose.pose
        values = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        norm = math.sqrt(sum(value * value for value in values[3:]))
        self.odom_valid &= all(math.isfinite(value) for value in values)
        self.odom_valid &= abs(norm - 1.0) < 1e-3
        if topic == "/integrated_to_init":
            self.odom_stamps.append(msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec)

    def tf_callback(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            self.tf_parents.setdefault(transform.child_frame_id, set()).add(
                transform.header.frame_id
            )


def spin_for(node: Node, seconds: float) -> None:
    deadline = time.monotonic() + seconds
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--qos", choices=("best_effort", "reliable"), required=True)
    parser.add_argument("--scans", type=int, default=14)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    rclpy.init()
    node = Probe(args.qos)
    spin_for(node, 2.0)

    # Exercise empty, all-invalid, and missing-intensity rejection first.
    node.publisher.publish(cloud_message(node, []))
    node.publisher.publish(cloud_message(node, [(math.nan, math.nan, math.nan, 0.0)]))
    missing = cloud_message(node, [(1.0, 0.0, 0.0, 1.0)])
    missing.fields = FIELDS[:3]
    node.publisher.publish(missing)
    spin_for(node, 0.5)

    for scan_index in range(args.scans):
        node.publisher.publish(cloud_message(node, synthetic_scan(scan_index)))
        spin_for(node, 0.11)
    spin_for(node, 4.0)

    advancing_stamps = len(node.odom_stamps) >= 2 and all(
        newer > older for older, newer in zip(node.odom_stamps, node.odom_stamps[1:])
    )
    result = {
        "qos": args.qos,
        "published_valid_scans": args.scans,
        "counts": node.counts,
        "nonempty_counts": node.nonempty,
        "odometry_finite_and_normalized": node.odom_valid,
        "integrated_timestamps_advancing": advancing_stamps,
        "tf_parent_sets": {
            child: sorted(parents) for child, parents in sorted(node.tf_parents.items())
        },
        "tf_children_have_one_parent": all(
            len(parents) == 1 for parents in node.tf_parents.values()
        ),
        "passed": (
            node.nonempty.get("/segmented_cloud", 0) > 0
            and node.nonempty.get("/ground_cloud", 0) > 0
            and node.nonempty.get("/laser_cloud_less_flat", 0) > 0
            and node.counts.get("/laser_odom_to_init", 0) > 0
            and node.odom_valid
            and advancing_stamps
            and all(len(parents) == 1 for parents in node.tf_parents.values())
            and node.tf_parents.get("camera_init") == {"map"}
            and node.tf_parents.get("velodyne") == {"base_link"}
        ),
    }
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
