"""Round-trip measurement payloads through the optional recorder, without DDS."""
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import rosbag2_py
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Imu, PointCloud2, PointField

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'go2_nav2'))
from go2_nav2.cloud_stamp_sync import record_bag


class RecordingTests(unittest.TestCase):
    def test_round_trip_preserves_measurements_headers_and_distinct_arrival_times(self):
        with tempfile.TemporaryDirectory() as directory:
            uri = str(Path(directory) / 'sensors')
            writer = rosbag2_py.SequentialWriter()
            writer.open(rosbag2_py.StorageOptions(uri=uri, storage_id='sqlite3'),
                        rosbag2_py.ConverterOptions('', ''))
            cloud = PointCloud2()
            cloud.header.frame_id = 'utlidar_lidar'
            cloud.header.stamp.sec = 100
            cloud.width = cloud.height = 1
            cloud.point_step = cloud.row_step = 4
            cloud.fields = [PointField(name='time', offset=0, datatype=PointField.FLOAT32, count=1)]
            cloud.data = [0, 0, 128, 61]  # 0.0625 seconds per-point time.
            imu = Imu()
            imu.header.frame_id = 'utlidar_imu'
            imu.header.stamp.sec = 100
            imu.header.stamp.nanosec = 4000000
            imu.angular_velocity.z = -1.2
            imu.linear_acceleration.z = 10.6
            imu.orientation.w = 1.0
            samples = [('cloud', cloud, 'sensor_msgs/msg/PointCloud2', 100123000000),
                       ('imu', imu, 'sensor_msgs/msg/Imu', 100124000000)]
            expected = []
            node = SimpleNamespace(_bag_writer=writer, _bag_topics={})
            for stream, msg, msg_type, wall in samples:
                topic = '/lidar3d/' + stream + '_sync'
                node._bag_topics[stream] = topic
                writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type=msg_type,
                                                           serialization_format='cdr'))
                payload = serialize_message(msg)
                with patch('go2_nav2.cloud_stamp_sync.time.time_ns', return_value=wall):
                    record_bag(node, stream, msg)
                self.assertEqual(serialize_message(msg), payload)
                expected.append((topic, payload, wall))
            node._bag_writer = None
            del writer
            self.assertTrue((Path(uri) / 'metadata.yaml').is_file())
            reader = rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id='sqlite3'),
                        rosbag2_py.ConverterOptions('', ''))
            actual = []
            while reader.has_next():
                actual.append(reader.read_next())
            self.assertEqual(actual, expected)


if __name__ == '__main__':
    unittest.main()
