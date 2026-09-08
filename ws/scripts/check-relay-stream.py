#!/usr/bin/env python3
"""Read-only stream check: count arrivals and unique original sensor timestamps."""

import argparse
import json
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan, PointCloud2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--warmup", type=float, default=3.0)
    parser.add_argument("--require-scan", action="store_true")
    args = parser.parse_args()
    if not 0 < args.duration <= 300 or not 0 <= args.warmup <= 60:
        parser.error("duration must be 0–300 seconds; warmup must be 0–60")
    rclpy.init()
    node = rclpy.create_node("go2_read_only_stream_check")
    streams = {}
    commands = []
    measuring = False

    def callback(name):
        def receive(msg):
            if measuring:
                stamp = msg.header.stamp
                streams[name].append((time.monotonic(), stamp.sec * 10**9 + stamp.nanosec))
        return receive

    for name, topic, msg_type in (
        ("cloud", "/utlidar/cloud_deskewed", PointCloud2),
        ("odom", "/utlidar/robot_odom", Odometry),
        ("imu", "/utlidar/imu", Imu),
        ("scan", "/scan", LaserScan),
    ):
        streams[name] = []
        node.create_subscription(msg_type, topic, callback(name), qos_profile_sensor_data)
    node.create_subscription(Twist, "/cmd_vel", lambda msg: commands.append(msg) if measuring else None,
                             qos_profile_sensor_data)
    try:
        deadline = time.monotonic() + args.warmup
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        measuring = True
        start = time.monotonic()
        while time.monotonic() - start < args.duration:
            rclpy.spin_once(node, timeout_sec=0.05)
        elapsed = time.monotonic() - start
        report = {"duration_sec": round(elapsed, 3), "cmd_vel_messages": len(commands)}
        for name, samples in streams.items():
            unique = len({stamp for _, stamp in samples})
            gaps = [b[0] - a[0] for a, b in zip(samples, samples[1:])]
            report[name] = {"messages": len(samples), "unique_stamps": unique,
                            "duplicates": len(samples) - unique,
                            "hz": round(len(samples) / elapsed, 2),
                            "max_arrival_gap_ms": round(max(gaps, default=0) * 1000, 1)}
        print(json.dumps(report, indent=2), flush=True)
        required = ["cloud", "odom"] + (["scan"] if args.require_scan else [])
        return 0 if all(report[name]["messages"] > 0 and report[name]["duplicates"] == 0
                        for name in required) else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
