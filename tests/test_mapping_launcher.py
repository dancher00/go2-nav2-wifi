"""Exercise the one-command host launcher with inert Docker/SSH/X11 commands."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / 'mapping.sh'
FAKES = r'''
docker() {
  echo "docker $*" >> "$TEST_DIR/trace"
  if [[ "$1" == inspect ]]; then
    if [[ "$*" == *State.Running* ]]; then echo true;
    else printf '%s\n' GO2_HOST_IP=192.0.2.1 GO2_ROBOT_IP=192.0.2.2 GO2_NET=wifi; fi
  elif [[ "$*" == *'id -u'* ]]; then echo 1000
  elif [[ "$*" == *'start mapping'* ]]; then sleep .2; return "${TEST_MAPPING_EXIT:-0}"
  fi
}
getent() { echo testuser:x:1000:1000::/:/bin/bash; }
xhost() { echo "xhost $*" >> "$TEST_DIR/trace"; echo SI:localuser:testuser; }
ssh() {
  echo "ssh $*" >> "$TEST_DIR/trace"
  if [[ "$*" == *'-O exit'* ]]; then touch "$TEST_DIR/closed";
  elif [[ "$*" == *--sensors-only* ]]; then
    if [[ "${TEST_RELAY_FAIL:-0}" == 1 ]]; then echo 'Relay already running'; return 1; fi
    echo 'Relay running: test'
    while [[ ! -f "$TEST_DIR/closed" ]]; do sleep .05; done
  fi
}
launcher_path="$1"
shift
source "$launcher_path" "$@"
'''


class MappingLauncherTests(unittest.TestCase):
    def run_launcher(self, *args, **overrides):
        with tempfile.TemporaryDirectory() as directory:
            script = Path(directory) / 'mapping.sh'
            shutil.copyfile(SCRIPT, script)
            result = subprocess.run(['bash', '-c', FAKES, 'test', str(script), *args],
                                    env=dict(os.environ, DISPLAY=':test', TEST_DIR=directory, **overrides),
                                    text=True, capture_output=True, timeout=8)
            trace = Path(directory) / 'trace'
            return result, trace.read_text() if trace.exists() else ''

    def test_help_does_not_touch_robot_or_docker(self):
        result, trace = self.run_launcher('--help')
        self.assertEqual(result.returncode, 0)
        self.assertEqual(trace, '')

    def test_one_command_is_sensor_only_and_cleanup_is_owner_scoped(self):
        result, trace = self.run_launcher()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('GO2_RELAY_CAMERA=0', trace)
        self.assertIn('--sensors-only', trace)
        self.assertIn('start mapping --rviz --owner mapping.', trace)
        self.assertIn('stop mapping --owner mapping.', trace)
        self.assertIn('-O exit', trace)
        self.assertNotIn('sport_bridge', trace)
        self.assertNotIn('xhost -SI', trace, 'Existing X11 permission must be preserved')

    def test_existing_relay_is_not_taken_over(self):
        result, trace = self.run_launcher(TEST_RELAY_FAIL='1')
        self.assertNotEqual(result.returncode, 0)
        self.assertNotIn('start mapping', trace)
        self.assertNotIn('stop mapping', trace)

    def test_mapping_failure_still_closes_own_relay(self):
        result, trace = self.run_launcher(TEST_MAPPING_EXIT='7')
        self.assertEqual(result.returncode, 7)
        self.assertIn('-O exit', trace)
        self.assertIn('stop mapping --owner mapping.', trace)
