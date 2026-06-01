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
import os
import socket
import struct
import sys
import time
from typing import Callable, List, Tuple, Type

import rclpy
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
    active: List[Tuple[str, str, str]] = []
    for topic, type_str, qos_kind in RELAY_TOPICS_CANDIDATES:
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
    if kind == "sensor":
        return QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        )
    return QoSProfile(depth=10)


def _send_frame(sock: socket.socket, topic_id: int, payload: bytes) -> None:
    hdr = FRAME_HDR.pack(topic_id & 0xFF, len(payload))
    sock.sendall(hdr + payload)


def _recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("relay socket closed")
        buf.extend(chunk)
    return bytes(buf)


def _wait_for_ready(ready_path: str, timeout_sec: float = 30.0) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if os.path.isfile(ready_path):
            return True
        time.sleep(0.2)
    return False


def run_subscriber(socket_path: str) -> None:
    if not _wait_for_ready(READY_PATH):
        print(f"ERROR: publisher not ready ({READY_PATH})", file=sys.stderr)
        sys.exit(1)

    rclpy.init()
    node = Node("go2_relay_wifi_sub")
    topics = relay_topics()
    counts = [0] * len(topics)
    last_log = time.monotonic()

    sock: socket.socket | None = None
    for _attempt in range(60):
        try:
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            sock.connect(socket_path)
            node.get_logger().info(f"connected to publisher at {socket_path}")
            break
        except OSError:
            time.sleep(0.5)
    if sock is None:
        node.get_logger().error(f"cannot connect to {socket_path} — is pub process up?")
        rclpy.shutdown()
        sys.exit(1)

    def make_cb(topic_id: int, topic: str) -> Callable:
        def cb(msg) -> None:
            try:
                _send_frame(sock, topic_id, serialize_message(msg))
                counts[topic_id] += 1
            except OSError as exc:
                node.get_logger().error(f"send failed on {topic}: {exc}")

        return cb

    for topic_id, (topic, type_str, qos_kind) in enumerate(topics):
        msg_type = get_message(type_str)
        node.create_subscription(
            msg_type, topic, make_cb(topic_id, topic), _qos(qos_kind)
        )
        node.get_logger().info(f"subscribe {topic} ({type_str})")

    def spin_once_with_log() -> None:
        nonlocal last_log
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.monotonic()
        if now - last_log >= 5.0:
            parts = [
                f"{topics[i][0].split('/')[-1]}={counts[i]}"
                for i in range(len(topics))
            ]
            node.get_logger().info("relay rates (5s): " + ", ".join(parts))
            last_log = now

    try:
        while rclpy.ok():
            spin_once_with_log()
    finally:
        sock.close()
        node.destroy_node()
        rclpy.shutdown()


def run_publisher(socket_path: str) -> None:
    for path in (READY_PATH, socket_path):
        try:
            os.unlink(path)
        except FileNotFoundError:
            pass

    rclpy.init()
    node = Node("go2_relay_wifi_pub")
    pubs: List = []
    types: List[Type] = []

    topics = relay_topics()
    for _topic, type_str, qos_kind in topics:
        msg_type = get_message(type_str)
        types.append(msg_type)
        pubs.append(node.create_publisher(msg_type, _topic, _qos(qos_kind)))
        node.get_logger().info(f"publish {_topic} ({type_str})")

    open(READY_PATH, "w", encoding="ascii").close()
    print(f"[relay-pub] ready ({len(topics)} topics)", flush=True)

    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(socket_path)
    server.listen(1)
    print(f"[relay-pub] waiting for subscriber on {socket_path}", flush=True)

    conn, _ = server.accept()
    print("[relay-pub] subscriber connected", flush=True)

    try:
        while rclpy.ok():
            hdr = _recv_exact(conn, FRAME_HDR.size)
            topic_id, plen = FRAME_HDR.unpack(hdr)
            payload = _recv_exact(conn, plen)
            if topic_id >= len(pubs):
                node.get_logger().warn(f"bad topic_id {topic_id}")
                continue
            msg = deserialize_message(payload, types[topic_id])
            pubs[topic_id].publish(msg)
            rclpy.spin_once(node, timeout_sec=0)
    except (ConnectionError, OSError) as exc:
        node.get_logger().error(f"relay pub stopped: {exc}")
    finally:
        conn.close()
        server.close()
        for path in (READY_PATH, socket_path):
            try:
                os.unlink(path)
            except FileNotFoundError:
                pass
        node.destroy_node()
        rclpy.shutdown()


def run_cmd_publisher(socket_path: str) -> None:
    """Internal DDS: laptop cmd_vel -> local /cmd_vel for sport_bridge."""
    for path in (CMD_READY_PATH, socket_path):
        try:
            os.unlink(path)
        except FileNotFoundError:
            pass

    rclpy.init()
    node = Node("go2_relay_cmd_pub")
    msg_type = get_message(CMD_VEL_TYPE)
    pub = node.create_publisher(msg_type, CMD_VEL_TOPIC, 10)
    node.get_logger().info(f"publish {CMD_VEL_TOPIC} ({CMD_VEL_TYPE}) from relay")

    open(CMD_READY_PATH, "w", encoding="ascii").close()
    print("[relay-cmd-pub] ready", flush=True)

    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(socket_path)
    server.listen(1)
    print(f"[relay-cmd-pub] waiting on {socket_path}", flush=True)

    conn, _ = server.accept()
    print("[relay-cmd-pub] subscriber connected", flush=True)

    try:
        while rclpy.ok():
            hdr = _recv_exact(conn, FRAME_HDR.size)
            _topic_id, plen = FRAME_HDR.unpack(hdr)
            payload = _recv_exact(conn, plen)
            msg = deserialize_message(payload, msg_type)
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0)
    except (ConnectionError, OSError) as exc:
        node.get_logger().error(f"relay cmd pub stopped: {exc}")
    finally:
        conn.close()
        server.close()
        for path in (CMD_READY_PATH, socket_path):
            try:
                os.unlink(path)
            except FileNotFoundError:
                pass
        node.destroy_node()
        rclpy.shutdown()


def run_cmd_subscriber(socket_path: str) -> None:
    """Wi-Fi DDS: subscribe /cmd_vel from laptop, forward to internal relay."""
    if not _wait_for_ready(CMD_READY_PATH):
        print(f"ERROR: cmd publisher not ready ({CMD_READY_PATH})", file=sys.stderr)
        sys.exit(1)

    rclpy.init()
    node = Node("go2_relay_cmd_sub")
    counts = 0
    last_log = time.monotonic()

    sock: socket.socket | None = None
    for _attempt in range(60):
        try:
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            sock.connect(socket_path)
            node.get_logger().info(f"cmd relay connected to {socket_path}")
            break
        except OSError:
            time.sleep(0.5)
    if sock is None:
        node.get_logger().error(f"cannot connect to {socket_path}")
        rclpy.shutdown()
        sys.exit(1)

    msg_type = get_message(CMD_VEL_TYPE)

    def cb(msg) -> None:
        nonlocal counts
        try:
            _send_frame(sock, 0, serialize_message(msg))
            counts += 1
        except OSError as exc:
            node.get_logger().error(f"cmd send failed: {exc}")

    node.create_subscription(msg_type, CMD_VEL_TOPIC, cb, _qos("default"))
    node.get_logger().info(f"subscribe {CMD_VEL_TOPIC} on Wi-Fi -> robot sport_bridge")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if now - last_log >= 5.0:
                node.get_logger().info(f"cmd_vel relay rate (5s): {counts}")
                counts = 0
                last_log = now
    finally:
        sock.close()
        node.destroy_node()
        rclpy.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser(description="Go2 Wi-Fi topic relay")
    parser.add_argument(
        "--role",
        choices=("sub", "pub", "pub_cmd", "sub_cmd", "both"),
        default="both",
        help="sub/pub=sensors, pub_cmd/sub_cmd=cmd_vel, both=launch pub then sub",
    )
    args = parser.parse_args()

    if args.role == "sub":
        run_subscriber(SOCKET_PATH)
    elif args.role == "pub":
        run_publisher(SOCKET_PATH)
    elif args.role == "pub_cmd":
        run_cmd_publisher(CMD_SOCKET_PATH)
    elif args.role == "sub_cmd":
        run_cmd_subscriber(CMD_SOCKET_PATH)
    else:
        import subprocess

        env = os.environ.copy()
        cdds = env.get("CYCLONEDDS_URI", "")
        if not cdds:
            print("ERROR: CYCLONEDDS_URI must be set for pub (robot-relay-wifi.sh)", file=sys.stderr)
            sys.exit(1)

        pub_env = env.copy()
        sub_env = env.copy()
        sub_env.pop("CYCLONEDDS_URI", None)

        script = os.path.abspath(__file__)
        pub = subprocess.Popen(
            [sys.executable, script, "--role", "pub"],
            env=pub_env,
        )
        time.sleep(0.5)
        if pub.poll() is not None:
            print("ERROR: pub process exited early", file=sys.stderr)
            sys.exit(1)

        try:
            subprocess.run(
                [sys.executable, script, "--role", "sub"],
                env=sub_env,
                check=False,
            )
        finally:
            pub.terminate()
            try:
                pub.wait(timeout=3)
            except subprocess.TimeoutExpired:
                pub.kill()


if __name__ == "__main__":
    main()
