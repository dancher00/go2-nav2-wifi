"""Capture raw LiDAR/camera pairs for a user-confirmed stationary robot on Jetson.
No SLAM pose, no calibration fitting, and no motion commands.
"""
import argparse
import json
import time
from pathlib import Path
import cv2
import numpy as np
import rclpy
from sensor_msgs.msg import PointCloud2
from rclpy.qos import qos_profile_sensor_data
from core import load_calibration
from native_node import xyz_from_cloud, atomic_npz


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--ipc', default='/ipc')
    p.add_argument('--output', required=True)
    p.add_argument('--pairs', type=int, default=30)
    args = p.parse_args()
    ipc = Path(args.ipc)
    folder = Path(args.output)
    folder.mkdir(parents=True, exist_ok=False)
    cfg, _, _, sensor_camera = load_calibration(ipc / 'calibration.json')
    cfg['capture_source'] = 'raw LiDAR, stationary assumption; no SLAM poses'
    (folder / 'calibration.json').write_text(json.dumps(cfg, indent=2) + '\n')
    rclpy.init(args=[])
    node = rclpy.create_node('camera_fusion_static_capture')
    latest = []

    def cloud(msg):
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        # Same nominal LiDAR->IMU translation as the Point-LIO baseline.
        xyz = xyz_from_cloud(msg) + np.array([.007698, .014655, -.00667])
        latest[:] = [(stamp, xyz, time.monotonic())]

    node.create_subscription(PointCloud2, '/camera_fusion/lio/cloud_sync', cloud, qos_profile_sensor_data)
    start = time.monotonic()
    last = None
    images = []
    try:
        while len(images) < args.pairs and time.monotonic() - start < 90:
            rclpy.spin_once(node, timeout_sec=.05)
            if not latest:
                continue
            try:
                with np.load(ipc / 'camera.npz', allow_pickle=False) as data:
                    stamp = int(data['stamp_ns'])
                    jpeg = data['jpeg'].copy()
                    age = (time.monotonic_ns() - int(data['receipt_ns'])) / 1e9
            except FileNotFoundError:
                continue
            if stamp == last or not 0 <= age < .5:
                continue
            cloud_stamp, xyz, receipt = latest[0]
            if abs(cloud_stamp - stamp) > 120000000 or time.monotonic() - receipt > .2:
                continue
            image = cv2.imdecode(jpeg, cv2.IMREAD_GRAYSCALE)
            if image is None:
                continue
            jpeg.tofile(str(folder / (str(stamp) + '.jpg')))
            atomic_npz(folder / (str(stamp) + '.npz'), xyz=xyz,
                       t_world_sensor=np.eye(4), t_world_camera=sensor_camera,
                       cloud_stamp_ns=np.int64(cloud_stamp), image_stamp_ns=np.int64(stamp))
            images.append(cv2.resize(image, (640, 360)))
            last = stamp
        flow = []
        # Compare all later frames to the first, robust to a minority of moving people.
        if len(images) > 1:
            features = cv2.goodFeaturesToTrack(images[0], 300, .01, 10)
            if features is not None:
                for image in images[1:]:
                    tracked, status, _ = cv2.calcOpticalFlowPyrLK(images[0], image, features, None)
                    if tracked is not None:
                        good = status.ravel() > 0
                        flow.extend(np.linalg.norm(tracked[good] - features[good], axis=2).ravel().tolist())
        report = {'source': cfg['capture_source'], 'pairs': len(images),
                  'tracked_displacements': len(flow),
                  'image_displacement_px_at_640x360_p50_p90': np.percentile(flow, [50, 90]).tolist() if flow else None,
                  'warning': 'Raw scans are assumed stationary; image flow is diagnostic, not proof. Exposure latency is unknown.'}
        (folder / 'capture.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2), flush=True)
        if len(images) < args.pairs:
            raise SystemExit('Incomplete capture: inspect camera and synchronized raw LiDAR freshness')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
