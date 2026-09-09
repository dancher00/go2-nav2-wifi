"""Native-result validation must reject interrupted files and invalid poses."""
import importlib.util
from pathlib import Path
import struct
import tempfile
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / 'ws/scripts/export-lidar3d.py'
spec = importlib.util.spec_from_file_location('lidar3d_result', SCRIPT)
result = importlib.util.module_from_spec(spec)
spec.loader.exec_module(result)


class ResultTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        (self.root / 'PCD').mkdir()
        self.cloud = self.root / 'PCD/scans.pcd'
        self.cloud.write_bytes(b'FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA binary\n' + struct.pack('<fff', 1, 2, 3))
        (self.root / 'trajectory.tum').write_text('1 0 0 0 0 0 0 1\n2 0.1 0 0 0 0 0 1\n')

    def test_native_result_is_readable_without_backend_dependencies(self):
        report = result.validate_result(self.root)
        self.assertEqual(report['points'], 1)
        self.assertEqual(report['trajectory_poses'], 2)
        self.assertFalse(report['loop_closure'])
        self.assertAlmostEqual(report['max_displacement_from_first_m'], .1)

    def test_truncated_pcd_is_not_a_successful_map(self):
        self.cloud.write_bytes(self.cloud.read_bytes()[:-1])
        with self.assertRaises(ValueError):
            result.validate_result(self.root)
        self.assertFalse((self.root / 'result.json').exists())

    def test_invalid_pose_and_repeated_time_are_rejected(self):
        for text in ('1 0 0 0 0 0 0 0\n', '1 nan 0 0 0 0 0 1\n',
                     '1 0 0 0 0 0 0 1\n1 0 0 0 0 0 0 1\n', ''):
            (self.root / 'trajectory.tum').write_text(text)
            with self.subTest(text=text), self.assertRaises(ValueError):
                result.validate_result(self.root)

    def test_previous_report_is_not_overwritten(self):
        report = self.root / 'result.json'
        report.write_text('previous')
        with self.assertRaises(FileExistsError):
            result.validate_result(self.root)
        self.assertEqual(report.read_text(), 'previous')

class LegKiloResultTests(unittest.TestCase):
    def test_compressed_native_map_and_backend_trajectory(self):
        import json
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            (root/'export/global_map').mkdir(parents=True)
            (root/'export/trajectories').mkdir()
            (root/'input.json').write_text(json.dumps({'backend':'legkilo','loop_closure':True}))
            (root/'export/trajectories/backend_imu_tum.txt').write_text('1 0 0 0 0 0 0 1\n2 .1 0 0 0 0 0 1\n')
            # LZF literal run: one control byte (length - 1), then the bytes.
            raw=struct.pack('<fff',1.,2.,3.)
            payload=bytes([len(raw)-1])+raw
            cloud=root/'export/global_map/global_map.pcd'
            cloud.write_bytes(b'FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nPOINTS 1\nDATA binary_compressed\n'+struct.pack('<II',len(payload),len(raw))+payload)
            report=result.validate_result(root)
            self.assertTrue(report['kinematics_used'])
            self.assertFalse(report['factory_body_pose_used'])
            self.assertEqual(report['points'],1)
            (root/'result.json').unlink()
            cloud.write_bytes(cloud.read_bytes()[:-1])
            with self.assertRaises(ValueError):result.validate_result(root)
