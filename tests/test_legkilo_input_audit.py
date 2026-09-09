import importlib.util
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location('legkilo_input_audit', Path(__file__).parents[1] / 'ws/scripts/compare-legkilo-inputs.py')
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


class InputAuditTests(unittest.TestCase):
    def read(self, text):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'trace.csv'
            path.write_text(text)
            return audit.read_trace(path)

    def test_same_stamps_but_changed_encoder_sample_is_not_identical(self):
        first = self.read('frame,0,1,2,10,123,10,123,1\nkin,0,1.5,456,789,15\n')
        second = self.read('frame,0,1,2,10,123,10,123,1\nkin,0,1.5,456,790,15\n')
        result = audit.compare(first, second)
        self.assertFalse(result['identical_inputs'])
        self.assertEqual(result['raw_cloud']['different_frames'], 0)
        self.assertEqual(result['kinematics']['different_frames'], 1)

    def test_missing_imu_observation_is_detected_even_with_all_clouds(self):
        first = self.read('frame,0,1,2,10,123,10,123,1\nkin,0,1.5,456,789,15\n')
        second = self.read('frame,0,1,2,10,123,10,123,0\n')
        self.assertFalse(audit.compare(first, second)['identical_inputs'])
        self.assertTrue(audit.compare(first, first)['identical_inputs'])

    def test_unflushed_trace_cannot_report_a_match(self):
        with self.assertRaises(ValueError):
            self.read('frame,0,1,2,10,123,10,123,2\nkin,0,1.5,456,789,15\n')
