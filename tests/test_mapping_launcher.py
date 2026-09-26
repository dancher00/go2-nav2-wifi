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
    if [[ "$*" == *State.Running* ]]; then
      if [[ "${TEST_CONTAINERS_STOPPED:-0}" == 1 ]]; then echo false; else echo true; fi
    else printf '%s\n' GO2_HOST_IP=192.0.2.1 GO2_ROBOT_IP=192.0.2.2 GO2_NET=wifi; fi
  elif [[ "$1" == start ]]; then return 0
  elif [[ "$*" == *'id -u'* ]]; then echo 1000
  elif [[ "$*" == *'start mapping'* ]]; then sleep .2; return "${TEST_MAPPING_EXIT:-0}"
  fi
}
getent() { echo testuser:x:1000:1000::/:/bin/bash; }
ip() { echo "192.0.2.2 dev test0 src ${TEST_ROUTE_IP:-192.0.2.1} uid 1000"; }
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
scp() { echo "scp $*" >> "$TEST_DIR/trace"; }
tar() { echo "tar $*" >> "$TEST_DIR/trace"; }
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
                                    env={**os.environ, 'DISPLAY': ':test', 'TEST_DIR': directory, **overrides},
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

    def test_view_is_camera_enabled_sensor_only_without_mapping(self):
        result, trace = self.run_launcher('--view')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('GO2_RELAY_CAMERA=1', trace)
        self.assertIn('--sensors-only', trace)
        self.assertIn('start sensors-viz --owner mapping.', trace)
        self.assertIn('stop sensors-viz --owner mapping.', trace)
        self.assertNotIn('start mapping', trace)
        self.assertNotIn('start lidar3d', trace)
        self.assertNotIn('sport_bridge', trace)

    def test_view_starts_existing_main_container(self):
        result, trace = self.run_launcher('--view', TEST_CONTAINERS_STOPPED='1')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('docker start go2-humble', trace)
        self.assertIn('docker exec go2-humble', trace)

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

    def test_lidar3d_deploys_sensor_profile_and_stops_only_its_session(self):
        result, trace = self.run_launcher('--3d', '--laptop')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('GO2_RELAY_PROFILE=lidar3d', trace)
        self.assertIn('robot_relay_wifi.py', trace)
        self.assertIn('start lidar3d --rviz --owner mapping.', trace)
        self.assertIn('stop lidar3d --owner mapping.', trace)
        self.assertNotIn('sport_bridge', trace)
        self.assertNotIn('stop mapping --owner', trace)

    def test_headless_mapping_does_not_change_x11_permissions(self):
        result, trace = self.run_launcher('--3d', '--laptop', GO2_RVIZ='0', DISPLAY='')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertNotIn('xhost', trace)
        self.assertNotIn('--rviz', trace)

    def test_jetson_default_owns_remote_compute_and_local_visualization(self):
        result, trace = self.run_launcher('--3d')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('GO2_LIDAR3D_COMPUTE=jetson', trace)
        self.assertIn('go2-lidar3d-onboard bash /ws/scripts/go2-session.sh start lidar3d --owner', trace)
        self.assertIn('start lidar3d-viz --owner mapping.', trace)
        self.assertIn('stop lidar3d-viz --owner mapping.', trace)
        self.assertIn('stop lidar3d --owner mapping.', trace)
        self.assertNotIn('start lidar3d --rviz', trace)
        self.assertNotIn('GO2_RELAY_PROFILE=lidar3d ', trace)
        self.assertNotIn('sport_bridge', trace)

    def test_stale_container_host_ip_uses_current_route_source(self):
        result, trace = self.run_launcher('--3d', TEST_ROUTE_IP='192.0.2.9')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('GO2_HOST_IP=192.0.2.9', trace)
        self.assertIn('-e GO2_HOST_IP=192.0.2.9', trace)
        self.assertIn('is stale; using route source 192.0.2.9', result.stdout)

    def test_headless_jetson_does_not_launch_laptop_backend_or_rviz(self):
        result, trace = self.run_launcher('--3d', GO2_RVIZ='0', DISPLAY='')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertNotIn('start lidar3d-viz', trace)
        self.assertNotIn('xhost', trace)
        self.assertIn('go2-lidar3d-onboard bash /ws/scripts/go2-session.sh start lidar3d', trace)

    def test_plan_preview_keeps_sensor_only_owned_session(self):
        result, trace = self.run_launcher('--3d', '--plan')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('GO2_LIDAR3D_PLAN=1', trace)
        self.assertIn('start lidar3d-viz --owner', trace)
        self.assertNotIn('sport_bridge', trace)
        self.assertNotIn('go2_cmd_vel_tcp', trace)
