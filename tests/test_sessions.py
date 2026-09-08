"""Real supervisor processes with inert children, private sockets, no ROS/robot."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

SCRIPT = Path(__file__).resolve().parents[1] / 'ws/scripts/go2_session.py'
spec = importlib.util.spec_from_file_location('sessions', SCRIPT)
sessions = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sessions)

PROGRAM = '''
import sys
from pathlib import Path
sys.path.insert(0, str(Path(sys.argv[1]).parent))
from go2_session import supervise
mode, root, fail = sys.argv[2:]
commands = [[sys.executable, '-c', 'import time; time.sleep(30)']]
preflight = [sys.executable, '-c', 'raise SystemExit(3)'] if fail == '1' else None
raise SystemExit(supervise(Path(root), mode, commands, preflight, owner='test-owner'))
'''


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.children = []
        self.addCleanup(self.cleanup)

    def cleanup(self):
        for child in self.children:
            if child.poll() is None:
                child.terminate()
            child.communicate(timeout=12)

    def start(self, mode='mapping', fail=False):
        child = subprocess.Popen([sys.executable, '-B', '-c', PROGRAM, str(SCRIPT),
                                  mode, str(self.root), str(int(fail))],
                                 stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        self.children.append(child)
        return child

    def running(self, mode='mapping'):
        deadline = time.monotonic()+5
        while time.monotonic() < deadline:
            result = sessions.request(self.root, mode, 'status')
            if result['state'] == 'running' and result['children']:
                return result
            time.sleep(.05)
        self.fail('Supervisor did not become ready')

    def test_stop_only_owned_session_and_status(self):
        mapping = self.start()
        first = self.running()
        teleop = self.start('teleop')
        self.running('teleop')
        result = sessions.request(self.root, 'mapping', 'stop')
        self.assertEqual(result['state'], 'stopped')
        self.assertEqual(mapping.wait(timeout=10), 0)
        self.assertIsNone(teleop.poll())
        self.assertEqual(sessions.request(self.root, 'mapping', 'status')['state'], 'not-managed')
        with self.assertRaises(ProcessLookupError):
            os.kill(first['children'][0], 0)

    def test_duplicate_mapping_and_navigation_refuse_without_killing_owner(self):
        owner = self.start()
        self.running()
        for mode in ('mapping', 'navigation'):
            duplicate = self.start(mode)
            self.assertNotEqual(duplicate.wait(timeout=5), 0)
            self.assertIsNone(owner.poll())
            self.assertEqual(sessions.request(self.root, 'mapping', 'status')['state'], 'running')

    def test_preflight_failure_never_starts_running_commands(self):
        child = self.start(fail=True)
        output, _ = child.communicate(timeout=10)
        self.assertNotEqual(child.returncode, 0)
        self.assertIn('preflight', output)
        self.assertFalse((self.root/'mapping.sock').exists())

    def test_sigterm_cleans_children_and_unlocks(self):
        child = self.start()
        self.running()
        child.terminate()
        self.assertEqual(child.wait(timeout=10), 0)
        self.start()
        self.running()

    def test_stale_pid_file_does_not_authorize_killing_any_pid(self):
        (self.root/'mapping.pid').write_text(str(os.getpid()))
        self.assertEqual(sessions.request(self.root, 'mapping', 'stop')['state'], 'not-managed')

    def test_automated_stop_cannot_stop_another_launchers_session(self):
        child = self.start()
        self.running()
        self.assertEqual(sessions.request(self.root, 'mapping', 'stop:other-owner')['state'], 'owner-mismatch')
        self.assertIsNone(child.poll())
        self.assertEqual(sessions.request(self.root, 'mapping', 'stop:test-owner')['state'], 'stopped')
        self.assertEqual(child.wait(timeout=10), 0)

    def test_terminal_hangup_cleans_owned_session(self):
        import signal
        child = self.start()
        self.running()
        child.send_signal(signal.SIGHUP)
        self.assertEqual(child.wait(timeout=10), 0)
        self.assertEqual(sessions.request(self.root, 'mapping', 'status')['state'], 'not-managed')

    def test_normal_window_exit_stops_session_successfully(self):
        window = [sys.executable, '-c', 'raise SystemExit(0)']
        self.assertEqual(sessions.supervise(self.root, 'mapping', [window],
                                            normal_exit_command=window), 0)
        self.assertEqual(sessions.request(self.root, 'mapping', 'status')['state'], 'not-managed')

    def test_unsafe_runtime_permissions_rejected(self):
        self.root.chmod(0o777)
        with patch.dict(os.environ, GO2_SESSION_DIR=str(self.root)):
            with self.assertRaises(ValueError):
                sessions.runtime_dir()

    def test_no_motion_process_in_mapping_command_plan(self):
        with patch.dict(os.environ, GO2_ODOM_SOURCE='utlidar'):
            commands = sessions.session_commands('mapping', None)
        self.assertEqual(len(commands), 1)
        self.assertIn('slam_mapping.launch.py', commands[0])

    def test_motion_modes_share_exclusive_lock(self):
        owner = self.start('teleop')
        self.running('teleop')
        for mode in ('transport', 'navigation'):
            self.assertNotEqual(self.start(mode).wait(timeout=5), 0)
            self.assertIsNone(owner.poll())


if __name__ == '__main__':
    unittest.main()
