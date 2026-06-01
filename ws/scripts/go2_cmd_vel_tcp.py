#!/usr/bin/env python3
"""TCP /cmd_vel bridge for Wi-Fi (Humble laptop -> Foxy robot).

Binary vx, vy, vyaw (12 bytes). Robot: --role server. Laptop: --role client.
Client sends at 20 Hz (heartbeat) so the link and sport_bridge stay alive.
"""

from __future__ import annotations

import argparse
import os
import socket
import struct
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

VEL_PACK = struct.Struct("!fff")
VEL_SIZE = VEL_PACK.size
CMD_TOPIC = "/cmd_vel"
DEFAULT_PORT = 17999


def _cmd_vel_hz() -> float:
    raw = os.environ.get("GO2_CMD_VEL_HZ", os.environ.get("GO2_CMD_TCP_HZ", "20"))
    return max(1.0, min(100.0, float(raw)))


def _sock_keepalive(sock: socket.socket) -> None:
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    except OSError:
        pass


def _recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("tcp closed")
        buf.extend(chunk)
    return bytes(buf)


def _twist_from_vel(vx: float, vy: float, vyaw: float) -> Twist:
    msg = Twist()
    msg.linear.x = vx
    msg.linear.y = vy
    msg.angular.z = vyaw
    return msg


def run_server(bind_host: str, port: int) -> None:
    rclpy.init()
    hz = _cmd_vel_hz()
    node = Node("go2_cmd_vel_tcp_server")
    pub = node.create_publisher(Twist, CMD_TOPIC, 10)
    node.get_logger().info(
        f"TCP cmd server {bind_host}:{port} -> {CMD_TOPIC} ({VEL_SIZE}B @ ~{hz:.0f} Hz)"
    )

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((bind_host, port))
    server.listen(1)
    server.settimeout(0.5)

    conn_lock = threading.Lock()
    conn: socket.socket | None = None
    stop = threading.Event()
    counts = 0
    counts_lock = threading.Lock()

    def recv_loop() -> None:
        nonlocal conn, counts
        while not stop.is_set():
            with conn_lock:
                c = conn
            if c is None:
                time.sleep(0.05)
                continue
            try:
                vx, vy, vyaw = VEL_PACK.unpack(_recv_exact(c, VEL_SIZE))
                pub.publish(_twist_from_vel(vx, vy, vyaw))
                with counts_lock:
                    counts += 1
            except (TimeoutError, ConnectionError, OSError, struct.error):
                if not stop.is_set():
                    node.get_logger().debug("cmd tcp client disconnected")
                with conn_lock:
                    if conn is c:
                        try:
                            conn.close()
                        except OSError:
                            pass
                        conn = None

    def accept_loop() -> None:
        nonlocal conn
        while not stop.is_set():
            try:
                new_conn, addr = server.accept()
            except (TimeoutError, OSError):
                continue
            _sock_keepalive(new_conn)
            new_conn.settimeout(None)
            with conn_lock:
                if conn is not None:
                    try:
                        conn.close()
                    except OSError:
                        pass
                conn = new_conn
            node.get_logger().info(f"cmd tcp connected from {addr[0]}:{addr[1]}")

    recv_t = threading.Thread(target=recv_loop, name="cmd_tcp_recv", daemon=True)
    acc_t = threading.Thread(target=accept_loop, name="cmd_tcp_accept", daemon=True)
    recv_t.start()
    acc_t.start()

    last_log = time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if now - last_log >= 5.0:
                with counts_lock:
                    n = counts
                    counts = 0
                if n:
                    node.get_logger().info(f"cmd_vel tcp (5s): {n}")
                last_log = now
    finally:
        stop.set()
        with conn_lock:
            if conn is not None:
                try:
                    conn.close()
                except OSError:
                    pass
        server.close()
        recv_t.join(timeout=1.0)
        acc_t.join(timeout=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def run_client(robot_host: str, port: int) -> None:
    rclpy.init()
    hz = _cmd_vel_hz()
    node = Node("go2_cmd_vel_tcp_client")
    sock_lock = threading.Lock()
    sock: socket.socket | None = None
    stop = threading.Event()
    last_vel = [0.0, 0.0, 0.0]
    vel_lock = threading.Lock()
    last_status_log = 0.0
    connected_once = False

    def _close_sock() -> None:
        nonlocal sock
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass
            sock = None

    def connect() -> None:
        nonlocal sock, last_status_log, connected_once
        _close_sock()
        last_err: OSError | None = None
        for _ in range(120):
            if stop.is_set():
                raise ConnectionError("shutdown")
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                _sock_keepalive(s)
                s.connect((robot_host, port))
                sock = s
                now = time.monotonic()
                if not connected_once or now - last_status_log > 15.0:
                    node.get_logger().info(f"cmd tcp -> {robot_host}:{port}")
                    last_status_log = now
                connected_once = True
                return
            except OSError as exc:
                last_err = exc
                time.sleep(0.25)
        raise ConnectionError(str(last_err))

    def sender_loop() -> None:
        nonlocal last_status_log
        period = 1.0 / hz
        fail_streak = 0
        while not stop.is_set():
            t0 = time.monotonic()
            with vel_lock:
                vx, vy, vyaw = last_vel
            payload = VEL_PACK.pack(vx, vy, vyaw)
            try:
                with sock_lock:
                    if sock is None:
                        connect()
                    assert sock is not None
                    sock.sendall(payload)
                fail_streak = 0
            except (ConnectionError, OSError):
                fail_streak += 1
                with sock_lock:
                    _close_sock()
                now = time.monotonic()
                if fail_streak >= 5 and now - last_status_log > 15.0:
                    node.get_logger().warn(
                        f"cmd tcp unstable ({robot_host}:{port}) — "
                        "one teleop client; relay running on robot?"
                    )
                    last_status_log = now
                time.sleep(min(0.25, period))
                continue
            dt = time.monotonic() - t0
            time.sleep(max(0.0, period - dt))

    def on_vel(msg: Twist) -> None:
        with vel_lock:
            last_vel[0] = float(msg.linear.x)
            last_vel[1] = float(msg.linear.y)
            last_vel[2] = float(msg.angular.z)

    reliable_qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
    )
    node.create_subscription(Twist, CMD_TOPIC, on_vel, qos_profile_sensor_data)
    node.create_subscription(Twist, CMD_TOPIC, on_vel, reliable_qos)

    sender = threading.Thread(target=sender_loop, name="cmd_vel_tcp_sender", daemon=True)
    sender.start()

    try:
        rclpy.spin(node)
    finally:
        stop.set()
        sender.join(timeout=2.0)
        with sock_lock:
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser(description="Go2 /cmd_vel TCP bridge")
    parser.add_argument("--role", choices=("server", "client"), required=True)
    parser.add_argument("--host", default="192.168.1.58")
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    args = parser.parse_args()
    if args.role == "server":
        run_server(args.bind, args.port)
    else:
        run_client(args.host, args.port)


if __name__ == "__main__":
    main()
