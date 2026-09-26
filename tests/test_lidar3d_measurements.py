"""Measurement clock conventions and the observer that adds no raw Wi-Fi copy."""
import importlib.util
from pathlib import Path
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / 'ws/scripts/compare-lidar3d-transport.py'
spec = importlib.util.spec_from_file_location('lidar3d_compare', SCRIPT)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class MeasurementTests(unittest.TestCase):
    def test_pipeline_matches_source_time_and_keeps_observer_latency_separate(self):
        offset = 100*10**9
        source, laptop = {'topics': {}}, {'side': 'pipeline', 'topics': {}}
        laptop['topics']['/go2/sensor_time_offset_ns'] = {'offset_ns': offset}
        for original, local in (('/utlidar/cloud', '/lidar3d/cloud_sync'),
                                ('/utlidar/imu', '/lidar3d/imu_sync'),
                                ('/utlidar/robot_odom', '/lidar3d/factory_odom_clock_reference')):
            rows = [[i*10**9, (1000+i)*10**9, i*10**9, 32, 42] for i in range(1,8)]
            source['topics'][original] = {'samples': rows}
            # Robot clock is 20 ms ahead; transfer including translator is 10 ms.
            translated = [[r[0]+offset, r[1]-10**7, r[2]+10**7, 32, 42] for r in rows]
            laptop['topics'][local] = {'samples': translated}
        laptop['topics']['/lidar3d/imu_sync']['samples'].pop(3)
        laptop['topics']['/lidar3d/cloud_sync']['samples'][3][4] = 99
        for name in ('/lidar3d/odom', '/lidar3d/registered'):
            laptop['topics'][name] = {'samples': []}
        clock = {'robot_minus_laptop_ns': 20*10**6, 'uncertainty_ns': 10**6}
        report = module.compare(source, laptop, clock, clock)
        cloud = report['transport']['/utlidar/cloud']
        self.assertEqual(cloud['matched'], 5)
        self.assertEqual(cloud['observer_transfer_ms']['median'], 10)
        self.assertEqual(cloud['cloud_payload_crc_mismatches'], 1)
        self.assertEqual(report['transport']['/utlidar/imu']['source_only'], 1)
        # Subtracting an offset is bookkeeping, not an independent raw->sync test.
        self.assertNotIn('/utlidar/cloud -> /lidar3d/cloud_sync', report['timestamp_pipeline'])
