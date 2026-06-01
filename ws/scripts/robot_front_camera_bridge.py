#!/usr/bin/env python3
"""Go2 front camera → sensor_msgs/Image (low-res mono8) for RViz + Wi-Fi relay.

Run ON ROBOT (CYCLONEDDS_URI unset). Backends (first match wins):
  1) unitree_sdk2_python VideoClient  (pip install -e ~/unitree_sdk2_python)
  2) ~/go2_camera_jpeg_cli            (bash ~/robot-build-camera-cli.sh)

Env: GO2_CAM_WIDTH=160 GO2_CAM_HEIGHT=120 GO2_CAM_FPS=10
"""

from __future__ import annotations

import os
import struct
import subprocess
import sys
import threading
import time
from typing import Optional, Protocol

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

CLI = os.environ.get("GO2_CAMERA_JPEG_CLI", "")


def resolve_camera_cli() -> Optional[str]:
    """Find JPEG CLI binary (not the go2_camera_jpeg_cli/ source folder from scp -r)."""
    if CLI:
        p = os.path.expanduser(CLI)
        if os.path.isfile(p) and os.access(p, os.X_OK):
            return p
    home = os.path.expanduser("~")
    for candidate in (
        os.path.join(home, "bin", "go2_camera_jpeg_cli"),
        os.path.join(home, "go2_camera_jpeg_cli"),  # file, if not shadowed by dir
        os.path.join(home, "go2_camera_jpeg_cli", "go2_camera_jpeg_cli"),
        os.path.join(home, ".go2_camera_build", "out", "go2_camera_jpeg_cli"),
    ):
        if os.path.isfile(candidate) and os.access(candidate, os.X_OK):
            return candidate
    return None

TOPIC = os.environ.get("GO2_CAMERA_TOPIC", "/go2_front_camera/image_raw")
FRAME_ID = os.environ.get("GO2_CAMERA_FRAME_ID", "front_camera")
WIDTH = int(os.environ.get("GO2_CAM_WIDTH", "160"))
HEIGHT = int(os.environ.get("GO2_CAM_HEIGHT", "120"))
TARGET_FPS = float(os.environ.get("GO2_CAM_FPS", "10"))


def _qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        durability=DurabilityPolicy.VOLATILE,
    )


class JpegSource(Protocol):
    def read(self) -> Optional[bytes]: ...
    def close(self) -> None: ...


class SdkJpegSource:
    """unitree_sdk2_python VideoClient — no C++ build required."""

    def __init__(self, logger) -> None:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.go2.video.video_client import VideoClient

        iface = os.environ.get("GO2_CAMERA_NET_IFACE", "")
        if iface:
            ChannelFactoryInitialize(0, iface)
        else:
            ChannelFactoryInitialize(0)
        self._client = VideoClient()
        self._client.SetTimeout(1.0)
        self._client.Init()
        logger.info("camera backend: unitree_sdk2_python VideoClient")

    def read(self) -> Optional[bytes]:
        code, data = self._client.GetImageSample()
        if code != 0 or not data:
            return None
        return bytes(data)

    def close(self) -> None:
        pass


class CliJpegSource:
    """JPEG stream from go2_camera_jpeg_cli (4-byte BE len + jpeg)."""

    def __init__(self, logger, cli_path: str) -> None:
        self._cli = cli_path
        self._logger = logger
        self._proc: Optional[subprocess.Popen] = None
        self._latest: Optional[bytes] = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._decode_fails = 0
        self._start()
        logger.info(f"camera backend: {self._cli}")

    def _cli_env(self) -> dict:
        env = os.environ.copy()
        sdk = os.path.expanduser(os.environ.get("UNITREE_SDK2", "~/unitree_sdk2"))
        arch = os.uname().machine
        paths = [
            os.path.join(sdk, "lib"),
            os.path.join(sdk, "build", "lib"),
            os.path.join(sdk, "lib", arch),
            os.path.join(sdk, "thirdparty", "lib", arch),
        ]
        extra = ":".join(p for p in paths if os.path.isdir(p))
        if extra:
            env["LD_LIBRARY_PATH"] = f"{extra}:{env.get('LD_LIBRARY_PATH', '')}"
        return env

    def _start(self) -> None:
        self._stop.clear()
        self._proc = subprocess.Popen(
            [self._cli],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=0,
            env=self._cli_env(),
        )
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()

    def _reader_loop(self) -> None:
        while not self._stop.is_set() and self._proc and self._proc.stdout:
            hdr = self._read_exact(self._proc.stdout, 4)
            if not hdr:
                if self._proc.poll() is not None:
                    err = ""
                    if self._proc.stderr:
                        err = self._proc.stderr.read(4096).decode("utf-8", errors="replace")
                    self._logger.error(
                        f"jpeg cli exited (code={self._proc.returncode}): {err[:300]}"
                    )
                return
            (size,) = struct.unpack("!I", hdr)
            if size == 0 or size > 5_000_000:
                continue
            data = self._read_exact(self._proc.stdout, size)
            if not data:
                return
            with self._lock:
                self._latest = data

    @staticmethod
    def _read_exact(stream, n: int) -> Optional[bytes]:
        buf = bytearray()
        while len(buf) < n:
            chunk = stream.read(n - len(buf))
            if not chunk:
                return None
            buf.extend(chunk)
        return bytes(buf)

    def read(self) -> Optional[bytes]:
        with self._lock:
            return self._latest

    def close(self) -> None:
        self._stop.set()
        if self._proc:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self._proc.kill()


def _open_jpeg_source(logger) -> JpegSource:
    try:
        return SdkJpegSource(logger)
    except ImportError:
        pass
    except Exception as exc:
        logger.warn(f"unitree_sdk2_python failed: {exc}")
    cli = resolve_camera_cli()
    if cli:
        try:
            return CliJpegSource(logger, cli)
        except OSError as exc:
            logger.warn(f"jpeg cli failed: {exc}")
    logger.error(
        "No camera backend. Run: bash ~/robot-build-camera-cli.sh\n"
        "  (if scp -r go2_camera_jpeg_cli/ created a folder, rebuild or set\n"
        "   GO2_CAMERA_JPEG_CLI=~/go2_camera_jpeg_cli/go2_camera_jpeg_cli)"
    )
    raise SystemExit(1)


def camera_backend_available() -> bool:
    try:
        from unitree_sdk2py.go2.video.video_client import VideoClient  # noqa: F401
        return True
    except ImportError:
        pass
    return resolve_camera_cli() is not None


class FrontCameraBridge(Node):
    def __init__(self) -> None:
        super().__init__("go2_front_camera_bridge")
        self._pub = self.create_publisher(Image, TOPIC, _qos())
        self._source = _open_jpeg_source(self.get_logger())
        self._min_period = 1.0 / max(TARGET_FPS, 0.5)
        self._last_pub = 0.0
        self._frames = 0
        self.create_timer(0.05, self._spin_once)
        self.get_logger().info(
            f"{TOPIC} mono8 {WIDTH}x{HEIGHT} @ {TARGET_FPS} Hz frame={FRAME_ID}"
        )

    def _spin_once(self) -> None:
        now = time.monotonic()
        if now - self._last_pub < self._min_period:
            return
        jpeg = self._source.read()
        if not jpeg:
            return
        if len(jpeg) < 3 or jpeg[0:2] != b"\xff\xd8":
            self.get_logger().warn(
                f"jpeg magic mismatch (len={len(jpeg)}, head={jpeg[:4]!r}) — retrying",
                throttle_duration_sec=10.0,
            )
            return
        arr = np.frombuffer(jpeg, dtype=np.uint8)
        bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if bgr is None:
            self.get_logger().warn(
                f"imdecode failed (len={len(jpeg)})",
                throttle_duration_sec=10.0,
            )
            return
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        small = cv2.resize(gray, (WIDTH, HEIGHT), interpolation=cv2.INTER_AREA)

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = FRAME_ID
        msg.height = HEIGHT
        msg.width = WIDTH
        msg.encoding = "mono8"
        msg.is_bigendian = 0
        msg.step = WIDTH
        msg.data = small.tobytes()
        self._pub.publish(msg)
        self._last_pub = now
        self._frames += 1
        if self._frames == 1:
            self.get_logger().info("first frame published")
        elif self._frames % int(max(TARGET_FPS, 1) * 30) == 0:
            self.get_logger().info(f"published {self._frames} frames")

    def destroy_node(self) -> None:
        self._source.close()
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = FrontCameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
