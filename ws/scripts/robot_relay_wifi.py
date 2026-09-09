#!/usr/bin/env python3
"""Relay Unitree topics from internal DDS (eth0) to Wi-Fi CycloneDDS for the laptop.

Sensor relay (robot -> laptop):
  sub (internal) -> socket -> pub (Wi-Fi)

cmd_vel relay (laptop -> robot, for teleop / Nav2):
  sub_cmd (Wi-Fi) -> socket -> pub_cmd (internal) -> sport_bridge on robot

Run via: bash ~/robot-relay-wifi.sh
"""

from __future__ import annotations

import argparse
import json
import os
import re
import signal
import socket
import struct
import sys
import time
import zlib
from typing import List, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message

# (topic, ros type string, qos: "sensor" | "default")
RELAY_TOPICS_CANDIDATES: List[Tuple[str, str, str]] = [
    ("/utlidar/cloud_deskewed", "sensor_msgs/msg/PointCloud2", "sensor"),
    ("/utlidar/robot_odom", "nav_msgs/msg/Odometry", "default"),
    ("/utlidar/imu", "sensor_msgs/msg/Imu", "sensor"),
    ("/sportmodestate", "unitree_go/msg/SportModeState", "default"),
    ("/lf/lowstate", "unitree_go/msg/LowState", "default"),  # → go2_joint_state_bridge → legs in RViz
    ("/go2_front_camera/image_raw", "sensor_msgs/msg/Image", "sensor"),  # VideoClient bridge, mono8 low-res
]

_RELAY_TOPICS: List[Tuple[str, str, str]] | None = None


def relay_topics() -> List[Tuple[str, str, str]]:
    """Active topics — skip types missing from Python env (e.g. unitree_go on Foxy robot)."""
    global _RELAY_TOPICS
    if _RELAY_TOPICS is not None:
        return _RELAY_TOPICS
    profile = os.environ.get("GO2_RELAY_PROFILE", "default")
    if profile == "lidar3d":
        # Preserve native sensor coordinates and original per-point time.
        candidates = [
            ("/utlidar/cloud", "sensor_msgs/msg/PointCloud2", "sensor"),
            ("/utlidar/robot_odom", "nav_msgs/msg/Odometry", "default"),
            ("/utlidar/imu", "sensor_msgs/msg/Imu", "imu"),
            ("/lf/lowstate", "unitree_go/msg/LowState", "default"),
        ]
    elif profile == "lidar3d-map":
        # Jetson computes locally. Wi-Fi carries visualization only, never raw IMU/cloud.
        candidates = [
            ("/lidar3d/registered", "sensor_msgs/msg/PointCloud2", "sensor"),
            ("/lidar3d/odom", "nav_msgs/msg/Odometry", "sensor"),
            ("/lidar3d/path", "nav_msgs/msg/Path", "sensor"),
            ("/lf/lowstate", "unitree_go/msg/LowState", "default"),
        ]
    elif profile == "default":
        candidates = RELAY_TOPICS_CANDIDATES
    else:
        raise ValueError("GO2_RELAY_PROFILE must be default, lidar3d or lidar3d-map")
    active: List[Tuple[str, str, str]] = []
    for topic, type_str, qos_kind in candidates:
        try:
            get_message(type_str)
            active.append((topic, type_str, qos_kind))
        except (ModuleNotFoundError, ImportError, AttributeError, ValueError) as exc:
            print(f"WARN: skip relay {topic} ({type_str}): {exc}", file=sys.stderr)
    if not active:
        print("ERROR: no relay topics available (check ROS env)", file=sys.stderr)
        sys.exit(1)
    _RELAY_TOPICS = active
    return _RELAY_TOPICS

SOCKET_PATH = os.environ.get("GO2_RELAY_SOCKET", "/tmp/go2-relay-wifi.sock")
READY_PATH = os.environ.get("GO2_RELAY_READY", "/tmp/go2-relay-wifi.ready")
CMD_SOCKET_PATH = os.environ.get("GO2_RELAY_CMD_SOCKET", "/tmp/go2-relay-wifi-cmd.sock")
CMD_READY_PATH = os.environ.get("GO2_RELAY_CMD_READY", "/tmp/go2-relay-wifi-cmd.ready")
CMD_VEL_TOPIC = "/cmd_vel"
CMD_VEL_TYPE = "geometry_msgs/msg/Twist"
FRAME_HDR = struct.Struct("!BI")  # topic_id (1 byte used), payload_len


def _qos(kind: str) -> QoSProfile:
    if kind in ("sensor", "imu"):
        return QoSProfile(
            depth=200 if kind == "imu" else 1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        )
    return QoSProfile(depth=10)


MAX_FRAME_SIZE = 32 * 1024 * 1024


def configure_domain(role: str) -> int:
    """Separate DDS domains prevent internal subscribers hearing relayed copies."""
    value = os.environ.get("GO2_RELAY_DOMAIN_ID", "64")
    if not re.fullmatch(r"[0-9]{1,3}", value) or not 1 <= int(value) <= 101:
        raise ValueError("GO2_RELAY_DOMAIN_ID must be an integer from 1 to 101")
    domain = 0 if role in ("sub", "pub_cmd") else int(value)
    os.environ["ROS_DOMAIN_ID"] = str(domain)
    if domain == 0:
        os.environ.pop("CYCLONEDDS_URI", None)
    return domain


def _shutdown(node: Node) -> None:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def _send_frame(sock: socket.socket, topic_id: int, payload: bytes) -> None:
    if not 0 <= topic_id <= 255 or len(payload) > MAX_FRAME_SIZE:
        raise ValueError("invalid relay frame")
    sock.sendall(FRAME_HDR.pack(topic_id, len(payload)) + payload)


def _recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        if not rclpy.ok():
            raise ExternalShutdownException()
        try:
            chunk = sock.recv(n - len(buf))
        except socket.timeout:
            continue  # Retain partial frames across timeouts.
        if not chunk:
            raise ConnectionError("relay socket closed")
        buf.extend(chunk)
    return bytes(buf)


def _wait_for_ready(ready_path: str, timeout_sec: float = 30.0) -> bool:
    deadline = time.monotonic() + timeout_sec
    while rclpy.ok() and time.monotonic() < deadline:
        if os.path.isfile(ready_path):
            return True
        time.sleep(0.2)
    return False


def _connect(socket_path: str, attempts: int = 60) -> socket.socket:
    last_error = None
    for _ in range(attempts):
        if not rclpy.ok():
            raise ExternalShutdownException()
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.settimeout(1.0)
        try:
            sock.connect(socket_path)
            return sock
        except OSError as exc:
            last_error = exc
            sock.close()
            time.sleep(0.5)
    raise ConnectionError(f"cannot connect to relay at {socket_path}") from last_error


def _listen(socket_path: str, ready_path: str) -> socket.socket:
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    bound = False
    try:
        server.bind(socket_path)
        bound = True
        server.listen(1)
        server.settimeout(0.25)
        # Publish readiness only once connections can actually be accepted.
        with open(ready_path, "w", encoding="ascii"):
            pass
        return server
    except BaseException:
        server.close()
        if bound:
            _remove_paths(ready_path, socket_path)
        raise


def _accept(server: socket.socket) -> socket.socket:
    while rclpy.ok():
        try:
            conn, _ = server.accept()
            conn.settimeout(0.25)
            return conn
        except socket.timeout:
            continue
    raise ExternalShutdownException()


def _remove_paths(*paths: str) -> None:
    for path in paths:
        try:
            os.unlink(path)
        except FileNotFoundError:
            pass


def _run_subscriber(socket_path: str, ready_path: str, topics: List, name: str) -> None:
    rclpy.init()
    node = Node(name)
    sock = None
    trace = None
    try:
        trace_prefix = os.environ.get('GO2_RELAY_TRACE_PREFIX')
        if trace_prefix:
            trace = open(trace_prefix + '-source.jsonl', 'x', buffering=65536)
        if not _wait_for_ready(ready_path):
            if not rclpy.ok():
                raise ExternalShutdownException()
            raise ConnectionError(f"publisher not ready ({ready_path})")
        sock = _connect(socket_path)
        node.get_logger().info(f"connected to publisher at {socket_path}")
        counts = [0] * len(topics)
        last_sent = [float('-inf')] * len(topics)
        last_log = time.monotonic()

        def make_cb(topic_id: int):
            def cb(msg) -> None:
                if os.environ.get('GO2_RELAY_PROFILE') == 'lidar3d-map':
                    interval = {'/lidar3d/registered': .2, '/lidar3d/path': 1.0, '/lf/lowstate': .05}.get(topics[topic_id][0], 0)
                    now = time.monotonic()
                    if now - last_sent[topic_id] < interval:
                        return
                    last_sent[topic_id] = now
                # A broken/blocked transport must stop this process, not log forever.
                if trace is not None:
                    wall, mono = time.time_ns(), time.monotonic_ns()
                # Visualization is already CDR. Skip Python reconstruction of the
                # growing Path for frames that will not be sent over Wi-Fi.
                payload = msg if isinstance(msg, bytes) else serialize_message(msg)
                if trace is not None and hasattr(msg, 'header'):
                    stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
                    crc = zlib.crc32(msg.data) if hasattr(msg, 'point_step') else None
                    trace.write(json.dumps([topics[topic_id][0], stamp, wall, mono, len(payload), crc]) + '\n')
                _send_frame(sock, topic_id, payload)
                counts[topic_id] += 1
            return cb

        for topic_id, (topic, type_str, qos_kind) in enumerate(topics):
            node.create_subscription(get_message(type_str), topic, make_cb(topic_id), _qos(qos_kind),
                                     raw=os.environ.get('GO2_RELAY_PROFILE') == 'lidar3d-map')
            node.get_logger().info(f"subscribe {topic} ({type_str})")
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            elapsed = now - last_log
            if elapsed >= 5.0:
                parts = [f"{topic.split('/')[-1]}={counts[i] / elapsed:.1f} Hz"
                         for i, (topic, _, _) in enumerate(topics)]
                node.get_logger().info("relay rates: " + ", ".join(parts))
                counts[:] = [0] * len(topics)
                last_log = now
    finally:
        if trace is not None:
            trace.close()
        if sock is not None:
            sock.close()
        _shutdown(node)


def _run_publisher(socket_path: str, ready_path: str, topics: List, name: str) -> None:
    # The shell supervisor owns the instance lock and removes stale paths.
    rclpy.init()
    node = Node(name)
    server = conn = None
    owns_socket = False
    try:
        pubs = []
        types = []
        for topic, type_str, qos_kind in topics:
            msg_type = get_message(type_str)
            types.append(msg_type)
            pubs.append(node.create_publisher(msg_type, topic, _qos(qos_kind)))
            node.get_logger().info(f"publish {topic} ({type_str})")
        server = _listen(socket_path, ready_path)
        owns_socket = True
        node.get_logger().info(f"ready on {socket_path}; domain={os.environ['ROS_DOMAIN_ID']}")
        conn = _accept(server)
        while rclpy.ok():
            topic_id, plen = FRAME_HDR.unpack(_recv_exact(conn, FRAME_HDR.size))
            if topic_id >= len(pubs) or plen > MAX_FRAME_SIZE:
                raise ValueError("invalid relay frame header")
            payload = _recv_exact(conn, plen)
            pubs[topic_id].publish(deserialize_message(payload, types[topic_id]))
            rclpy.spin_once(node, timeout_sec=0)
    finally:
        if conn is not None:
            conn.close()
        if server is not None:
            server.close()
        if owns_socket:
            _remove_paths(ready_path, socket_path)
        _shutdown(node)


def run_subscriber(socket_path: str) -> None:
    _run_subscriber(socket_path, READY_PATH, relay_topics(), "go2_relay_wifi_sub")


def run_publisher(socket_path: str) -> None:
    _run_publisher(socket_path, READY_PATH, relay_topics(), "go2_relay_wifi_pub")


def run_cmd_publisher(socket_path: str) -> None:
    _run_publisher(socket_path, CMD_READY_PATH,
                   [(CMD_VEL_TOPIC, CMD_VEL_TYPE, "default")], "go2_relay_cmd_pub")


def run_cmd_subscriber(socket_path: str) -> None:
    _run_subscriber(socket_path, CMD_READY_PATH,
                    [(CMD_VEL_TOPIC, CMD_VEL_TYPE, "default")], "go2_relay_cmd_sub")


def _interrupt(_signum, _frame) -> None:
    raise KeyboardInterrupt()


def main() -> None:
    parser = argparse.ArgumentParser(description="Go2 Wi-Fi topic relay")
    parser.add_argument("--role", choices=("sub", "pub", "pub_cmd", "sub_cmd", "both"),
                        default="both", help="both launches the sensor relay only")
    args = parser.parse_args()
    configure_domain(args.role)
    signal.signal(signal.SIGTERM, _interrupt)
    signal.signal(signal.SIGINT, _interrupt)
    runners = {"sub": (run_subscriber, SOCKET_PATH), "pub": (run_publisher, SOCKET_PATH),
               "sub_cmd": (run_cmd_subscriber, CMD_SOCKET_PATH),
               "pub_cmd": (run_cmd_publisher, CMD_SOCKET_PATH)}
    try:
        if args.role != "both":
            runner, path = runners[args.role]
            runner(path)
            return

        import subprocess
        if not os.environ.get("CYCLONEDDS_URI"):
            raise ValueError("CYCLONEDDS_URI must be set for Wi-Fi publisher")
        children = []
        try:
            script = os.path.abspath(__file__)
            for role in ("pub", "sub"):
                children.append(subprocess.Popen([sys.executable, script, "--role", role]))
            while all(child.poll() is None for child in children):
                time.sleep(0.2)
            raise RuntimeError("relay child exited unexpectedly")
        finally:
            for child in children:
                if child.poll() is None:
                    child.terminate()
            for child in children:
                try:
                    child.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
