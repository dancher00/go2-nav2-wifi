"""Verify the host test runner's isolation contract without starting Docker."""

import json
import os
from pathlib import Path
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'ws/scripts/test-regressions.sh'
FAKE_DOCKER = '''
docker() {
  python3 -c 'import json, sys; print(json.dumps(sys.argv[1:]))' "$@"
  return "${GO2_TEST_DOCKER_EXIT:-0}"
}
TEST_RUNNER_PATH="$1"
shift
source "$TEST_RUNNER_PATH" "$@"
'''


class RegressionRunnerTests(unittest.TestCase):
    def run_script(self, *args, exit_code=0):
        return subprocess.run(['bash', '-c', FAKE_DOCKER, 'test', str(SCRIPT), *args],
                              env=dict(os.environ, GO2_TEST_DOCKER_EXIT=str(exit_code)),
                              text=True, capture_output=True, timeout=5)

    def test_default_run_has_no_robot_network_and_no_source_writes(self):
        result = self.run_script()
        self.assertEqual(result.returncode, 0, result.stderr)
        args = json.loads(result.stdout)
        self.assertEqual(args[args.index('--network')+1], 'none')
        self.assertIn('--read-only', args)
        self.assertIn(f'{ROOT}:/repo:ro', args)
        self.assertIn('ROS_LOG_DIR=/tmp/ros-log', args)
        self.assertIn('ROS_LOCALHOST_ONLY=1', args)
        self.assertIn('ROS_DOMAIN_ID=87', args)
        self.assertIn('CYCLONEDDS_URI=', args)
        self.assertIn('go2-humble:local', args)
        self.assertNotIn('--privileged', args)

    def test_ci_image_override_and_failure_are_preserved(self):
        result = self.run_script('go2-humble:ci', exit_code=37)
        self.assertEqual(result.returncode, 37)
        self.assertIn('go2-humble:ci', json.loads(result.stdout))

    def test_help_does_not_start_docker(self):
        result = self.run_script('--help')
        self.assertEqual(result.returncode, 0)
        self.assertTrue(result.stdout.startswith('Usage:'))

    def test_extra_arguments_are_rejected(self):
        result = self.run_script('image', 'unexpected')
        self.assertEqual(result.returncode, 2)
        self.assertEqual(result.stdout, '')


if __name__ == '__main__':
    unittest.main()
