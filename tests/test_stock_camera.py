import importlib.util
import io
from pathlib import Path
import sys
import unittest

spec = importlib.util.spec_from_file_location('stock_camera', Path(__file__).resolve().parents[1]/'ws/scripts/stock_camera.py')
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)


class StockCameraTests(unittest.TestCase):
    def test_frame_consumed_once_and_receipt_time_preserved(self):
        buffer = module.LatestFrame()
        buffer.put(b'first', 123456, 100)
        frame = buffer.take(200)
        self.assertEqual(frame.received_ns, 123456)
        self.assertIsNone(buffer.take(300))

    def test_repeated_sdk_reply_does_not_become_new_frame(self):
        buffer = module.LatestFrame()
        buffer.put(b'first', 10, 100)
        buffer.take(200)
        buffer.put(b'first', 20, 300)
        self.assertIsNone(buffer.take(400))
        self.assertEqual(buffer.duplicates, 1)

    def test_latest_frame_replaces_backlog(self):
        buffer = module.LatestFrame()
        buffer.put(b'first', 10, 100)
        buffer.put(b'second', 20, 200)
        self.assertEqual(buffer.take(300).jpeg, b'second')

    def test_stale_or_future_monotonic_frame_is_not_published(self):
        buffer = module.LatestFrame()
        buffer.put(b'old', 10, 100)
        self.assertIsNone(buffer.take(300_000_101))
        buffer.put(b'future', 20, 500)
        self.assertIsNone(buffer.take(400))

    def test_truncated_binary_stream(self):
        self.assertIsNone(module.read_exact(io.BytesIO(b'abc'), 4))
        self.assertEqual(module.read_exact(io.BytesIO(b'abcd'), 4), b'abcd')


if __name__ == '__main__':
    unittest.main()
