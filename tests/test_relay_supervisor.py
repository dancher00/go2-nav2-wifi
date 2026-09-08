"""Exercise the real shell supervisor with inert child processes, never robot APIs."""

import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]


class SupervisorTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        self.script = self.directory / "robot-relay-wifi.sh"
        shutil.copyfile(ROOT / "ws/scripts/robot-relay-wifi.sh", self.script)
        (self.directory / "robot_relay_wifi.py").touch()
        (self.directory / "robot-source-unitree-ros.sh").write_text('''# inert ROS setup
python3() {
echo "$*" >> "$GO2_TEST_CALLS"
if [[ "$*" == *"--role pub"* ]]; then
  [[ "${GO2_TEST_FAIL_PUB:-0}" == 1 ]] && exit 7
  touch "$GO2_RELAY_READY"
fi
exec sleep 30
}
''')
        self.env = dict(os.environ,
                        GO2_HOST_IP="127.0.0.1", GO2_WIFI_IFACE="lo", GO2_RELAY_CAMERA="0",
                        GO2_RELAY_DOMAIN_ID="64", GO2_RELAY_SOCKET=f"{self.directory}/relay.sock",
                        GO2_RELAY_READY=f"{self.directory}/ready",
                        GO2_RELAY_CYCLONEDDS_XML=f"{self.directory}/dds.xml",
                        GO2_TEST_CALLS=f"{self.directory}/calls")
        self.children = []
        self.addCleanup(self.stop_children)

    def stop_children(self):
        for child in self.children:
            if child.poll() is None:
                child.terminate()
            child.communicate(timeout=6)

    def start(self, **env):
        child = subprocess.Popen(["bash", str(self.script), "--sensors-only"],
                                 env=dict(self.env, **env), stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT, text=True)
        self.children.append(child)
        return child

    def wait_for_subscriber(self):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            calls = self.directory / "calls"
            if calls.exists() and "--role sub" in calls.read_text():
                return
            time.sleep(0.05)
        child = self.children[0]
        if child.poll() is not None:
            self.fail(f"supervisor exited: {child.communicate()[0]}")
        self.fail("subscriber did not start")

    def test_sensor_only_skips_motion_and_cleans_up_on_sigterm(self):
        child = self.start()
        self.wait_for_subscriber()
        calls = (self.directory / "calls").read_text().splitlines()
        self.assertEqual(len(calls), 2)
        self.assertTrue(all("robot_relay_wifi.py --role" in line for line in calls))
        child.terminate()
        output, _ = child.communicate(timeout=6)
        self.assertIn("SENSOR-ONLY", output)
        self.assertEqual(child.returncode, 143)
        self.assertFalse((self.directory / "ready").exists())

    def test_publisher_failure_fails_startup_without_motion(self):
        child = self.start(GO2_TEST_FAIL_PUB="1")
        output, _ = child.communicate(timeout=6)
        self.assertNotEqual(child.returncode, 0)
        self.assertIn("Publisher exited", output)
        self.assertEqual(len((self.directory / "calls").read_text().splitlines()), 1)

    def test_second_instance_does_not_remove_first_instances_ready_file(self):
        first = self.start()
        self.wait_for_subscriber()
        second = self.start()
        output, _ = second.communicate(timeout=6)
        self.assertNotEqual(second.returncode, 0)
        self.assertIn("already running", output)
        self.assertIsNone(first.poll())
        self.assertTrue((self.directory / "ready").exists())


if __name__ == "__main__":
    unittest.main()
