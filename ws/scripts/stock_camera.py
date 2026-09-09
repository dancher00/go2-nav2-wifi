#!/usr/bin/env python3
"""Native Go2 JPEG -> ROS. Receipt timestamps are NOT exposure timestamps."""
import argparse
from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import threading
import time

@dataclass(frozen=True)
class Frame:
    jpeg: bytes
    received_ns: int
    monotonic_ns: int


class LatestFrame:
    """Consume each frame once, discard stale data, suppress repeated SDK JPEGs."""
    def __init__(self):
        self.lock = threading.Lock()
        self.pending = None
        self.digest = None
        self.duplicates = 0
        self.received = 0

    def put(self, jpeg, received_ns, monotonic_ns):
        digest = hashlib.sha256(jpeg).digest()
        with self.lock:
            if digest == self.digest:
                self.duplicates += 1
                return
            self.digest = digest
            self.received += 1
            self.pending = Frame(jpeg, received_ns, monotonic_ns)

    def take(self, now_ns, max_age_ns=300_000_000):
        with self.lock:
            frame, self.pending = self.pending, None
        if frame is None or not 0 <= now_ns - frame.monotonic_ns <= max_age_ns:
            return None
        return frame


def read_exact(stream, size):
    data = bytearray()
    while len(data) < size:
        part = stream.read(size - len(data))
        if not part:
            return None
        data.extend(part)
    return bytes(data)


class Source:
    def __init__(self, cli, interface, buffer):
        env = os.environ.copy()
        # SDK domain 0 stays on the robot's internal network. ROS output is separate.
        env['CYCLONEDDS_URI'] = ('<CycloneDDS><Domain Id="0"><General>'
            f'<NetworkInterfaceAddress>{interface}</NetworkInterfaceAddress>'
            '</General></Domain></CycloneDDS>')
        # Unitree's reader uses its bundled CycloneDDS ABI, not Humble's version.
        # Scope this search path to the child; the ROS publisher keeps Humble's DDS.
        sdk_lib = env.get('GO2_CAMERA_SDK_LIB')
        if sdk_lib:
            env['LD_LIBRARY_PATH'] = sdk_lib + ':' + env.get('LD_LIBRARY_PATH', '')
        self.buffer = buffer
        self.error = ''
        self.process = subprocess.Popen([cli], stdout=subprocess.PIPE,
                                        stderr=None, env=env, bufsize=0)
        self.thread = threading.Thread(target=self.read, daemon=True)
        self.thread.start()

    def read(self):
        try:
            while True:
                header = read_exact(self.process.stdout, 4)
                if header is None:
                    self.error = 'camera reader EOF'
                    return
                size = struct.unpack('!I', header)[0]
                if not 3 <= size <= 5_000_000:
                    raise ValueError('invalid JPEG framing')
                jpeg = read_exact(self.process.stdout, size)
                if jpeg is None:
                    raise ValueError('truncated JPEG')
                if not jpeg.startswith(b'\xff\xd8'):
                    raise ValueError('invalid JPEG signature')
                self.buffer.put(jpeg, time.time_ns(), time.monotonic_ns())
        except (OSError, ValueError) as exc:
            self.error = str(exc)

    def close(self):
        self.process.terminate()
        try:
            self.process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()
        self.thread.join(timeout=2)
        self.process.stdout.close()


def main():
    import cv2
    import numpy as np
    import rclpy
    from sensor_msgs.msg import CompressedImage, Image
    from std_msgs.msg import String
    from rclpy.qos import qos_profile_sensor_data

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cli', required=True)
    parser.add_argument('--interface', default='eth0')
    parser.add_argument('--fps', type=float, default=15)
    parser.add_argument('--record-dir', type=Path)
    parser.add_argument('--record-seconds', type=float, default=60)
    parser.add_argument('--duration', type=float, default=0, help='0 runs until stopped')
    args = parser.parse_args()
    if not 0 < args.fps <= 30 or not args.interface.replace('_', '').replace('-', '').isalnum():
        parser.error('invalid FPS/interface')
    if args.record_seconds <= 0 or args.duration < 0:
        parser.error('invalid duration')
    if args.record_dir:
        args.record_dir.mkdir(parents=True, exist_ok=False)
    rclpy.init(args=[])
    node = rclpy.create_node('go2_stock_camera')
    jpeg_pub = node.create_publisher(CompressedImage, '/go2_stock_camera/image/compressed', qos_profile_sensor_data)
    raw_pub = node.create_publisher(Image, '/go2_stock_camera/image', qos_profile_sensor_data)
    status_pub = node.create_publisher(String, '/go2_stock_camera/status', 1)
    buffer = LatestFrame()
    source = Source(args.cli, args.interface, buffer)
    started = time.monotonic()
    state = {'published': 0, 'decode_errors': 0, 'last_frame_monotonic': None,
             'width': None, 'height': None, 'recorded': 0}
    records = open(args.record_dir / 'frames.jsonl', 'x') if args.record_dir else None

    def status():
        last = state['last_frame_monotonic']
        age = time.monotonic() - last if last is not None else None
        result = dict(state, received_unique=buffer.received, repeated_jpegs=buffer.duplicates,
                      frame_age_sec=age, live=age is not None and age < 1 and not source.error,
                      timestamp_basis='Jetson JPEG receipt; exposure time unavailable',
                      camera_calibrated=False, lidar_extrinsics_calibrated=False,
                      fusion_enabled=False, source_error=source.error)
        result.pop('last_frame_monotonic')
        return result

    def publish():
        frame = buffer.take(time.monotonic_ns())
        if frame is None:
            return
        pixels = cv2.imdecode(np.frombuffer(frame.jpeg, np.uint8), cv2.IMREAD_COLOR)
        if pixels is None:
            state['decode_errors'] += 1
            return
        h, w = pixels.shape[:2]
        message = CompressedImage()
        message.header.frame_id = 'go2_stock_camera_optical'
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(frame.received_ns, 10**9)
        message.format = 'bgr8; jpeg compressed bgr8'
        message.data = frame.jpeg
        jpeg_pub.publish(message)
        if raw_pub.get_subscription_count():
            raw = Image()
            raw.header = message.header
            raw.height, raw.width, raw.step = h, w, w * 3
            raw.encoding = 'bgr8'
            raw.data = pixels.tobytes()
            raw_pub.publish(raw)
        state.update(width=w, height=h, published=state['published']+1,
                     last_frame_monotonic=frame.monotonic_ns*1e-9)
        if records and time.monotonic()-started < args.record_seconds:
            name = f'{frame.received_ns}.jpg'
            with open(args.record_dir / name, 'xb') as stream:
                stream.write(frame.jpeg)
            records.write(json.dumps({'file': name, 'receipt_ns': frame.received_ns,
                                      'exposure_ns': None, 'width': w, 'height': h})+'\n')
            records.flush()
            state['recorded'] += 1

    timer = node.create_timer(1/args.fps, publish)
    status_timer = node.create_timer(1., lambda: status_pub.publish(String(data=json.dumps(status()))))
    node.get_logger().warning('Stock camera has no exposure timestamp or calibration in this adapter; fusion is disabled.')
    try:
        while rclpy.ok() and (not args.duration or time.monotonic()-started < args.duration):
            rclpy.spin_once(node, timeout_sec=.2)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # Only the SDK child created here is stopped; no robot services are touched.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        final = status()
        final.update(state='stopped', live=False)
        source.close()
        if records:
            records.close()
            (args.record_dir / 'summary.json').write_text(json.dumps(final, indent=2)+'\n')
        print(json.dumps(final), flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
