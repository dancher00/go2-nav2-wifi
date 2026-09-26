"""Remote results must never be removed before a verified local copy exists."""
import importlib.util
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('fetch_lidar3d', Path(__file__).resolve().parents[1] / 'ws/scripts/fetch-lidar3d.py')
fetch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fetch)


class ArchiveTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.remote_home = self.root / 'remote'
        self.run = self.remote_home / 'go2-nav2-lidar3d-onboard/ws/maps/lidar3d/run-20260909-test'
        self.run.mkdir(parents=True)
        (self.run / 'result.json').write_text('{}')
        (self.run / 'scans.pcd').write_bytes(b'complete original map')
        self.destination = self.root / 'laptop'
        self.corrupt = False
        self.real_run = subprocess.run

    def transport(self, command, **kwargs):
        if command[0] == 'ssh':
            remote = shlex.split(command[-1])
            code = remote[2].replace('Path.home()', f'Path({str(self.remote_home)!r})')
            return self.real_run([sys.executable, '-c', code, remote[3]], **kwargs)
        self.assertEqual(command[0], 'scp')
        self.assertTrue(self.run.exists())
        target = Path(command[-1]) / self.run.name
        shutil.copytree(self.run, target, dirs_exist_ok=True)
        if self.corrupt:
            (target / 'scans.pcd').write_bytes(b'incomplete')
        return subprocess.CompletedProcess(command, 0)

    def archive(self):
        with patch.object(fetch.subprocess, 'run', side_effect=self.transport):
            return fetch.archive('unitree@192.0.2.1', None, self.run.name, self.destination)

    def test_verified_copy_is_kept_before_remote_removal(self):
        report = self.archive()
        self.assertFalse(self.run.exists())
        self.assertEqual((self.destination / self.run.name / 'scans.pcd').read_bytes(), b'complete original map')
        self.assertTrue(report['removed_from_jetson'])

    def test_incomplete_copy_never_removes_remote_result(self):
        self.corrupt = True
        with self.assertRaisesRegex(ValueError, 'checksum'):
            self.archive()
        self.assertTrue(self.run.exists())
        self.assertFalse((self.destination / self.run.name).exists())

    def test_different_existing_result_is_never_overwritten(self):
        target = self.destination / self.run.name
        shutil.copytree(self.run, target)
        (target / 'scans.pcd').write_bytes(b'other user result')
        with self.assertRaisesRegex(ValueError, 'differs'):
            self.archive()
        self.assertTrue(self.run.exists())
        self.assertEqual((target / 'scans.pcd').read_bytes(), b'other user result')
