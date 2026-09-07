"""Exercise the client sender with simulated ROS callbacks, TCP, and time."""

import importlib.util
import os
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch


SCRIPT = Path(__file__).resolve().parents[1] / "ws/scripts/go2_cmd_vel_tcp.py"
SPEC = importlib.util.spec_from_file_location("go2_cmd_vel_tcp", SCRIPT)
tcp = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(tcp)


class SimulationComplete(Exception):
    pass


class ClientScenario:
    """Run the real sender loop deterministically, without ROS discovery or a robot."""

    def __init__(self, commands=(), duration=0.8, connect_delays=(0.0,), fail_send_at=None):
        self.now = 0.0
        self.commands = list(commands)
        self.duration = duration
        self.connect_delays = iter(connect_delays)
        self.fail_send_at = fail_send_at
        self.send_attempts = 0
        self.frames = []
        self.shutdown_frames = []
        self.spinning = True
        self.callbacks = []
        self.threads = []
        self.sockets = []

    def deliver_commands(self):
        while self.commands and self.commands[0][0] <= self.now:
            _, velocity = self.commands.pop(0)
            self.callbacks[0](tcp._twist_from_vel(*velocity))

    def sleep(self, seconds):
        self.now += seconds
        self.deliver_commands()
        if self.now >= self.duration:
            raise SimulationComplete

    def connect(self, address):
        self.now += next(self.connect_delays, 0.0)
        self.deliver_commands()

    def sendall(self, payload):
        if self.spinning:
            self.send_attempts += 1
            if self.send_attempts == self.fail_send_at:
                raise OSError("simulated link loss")
            self.frames.append((self.now, tcp.VEL_PACK.unpack(payload)))
        else:
            self.shutdown_frames.append(tcp.VEL_PACK.unpack(payload))

    def socket(self, *args):
        sock = Mock()
        sock.connect.side_effect = self.connect
        sock.sendall.side_effect = self.sendall
        self.sockets.append(sock)
        return sock

    def thread(self, *, target, **kwargs):
        thread = Mock()
        thread.run = target
        self.threads.append(thread)
        return thread

    def spin(self, node):
        self.deliver_commands()
        try:
            self.threads[0].run()
        except SimulationComplete:
            pass
        finally:
            self.spinning = False

    def run(self):
        node = Mock()
        node.create_subscription.side_effect = (
            lambda msg_type, topic, callback, qos: self.callbacks.append(callback)
        )
        clock = SimpleNamespace(monotonic=lambda: self.now, sleep=self.sleep)
        ros = SimpleNamespace(init=Mock(), spin=self.spin, ok=lambda: True, shutdown=Mock())
        with patch.dict(os.environ, {"GO2_CMD_VEL_HZ": "20", "GO2_CMD_SOURCE_TIMEOUT": "0.5"}), \
                patch.object(tcp, "Node", return_value=node), \
                patch.object(tcp, "rclpy", ros), \
                patch.object(tcp, "time", clock), \
                patch.object(tcp.socket, "socket", side_effect=self.socket), \
                patch.object(tcp.threading, "Thread", side_effect=self.thread):
            tcp.run_client("127.0.0.1", 17999)
        return self


class CommandWatchdogTests(unittest.TestCase):
    def assert_velocity(self, actual, expected):
        for value, target in zip(actual, expected):
            self.assertAlmostEqual(value, target, places=6)

    def test_no_source_command_sends_zero_heartbeats(self):
        scenario = ClientScenario(duration=0.2).run()
        self.assertGreater(len(scenario.frames), 1)
        for _, velocity in scenario.frames:
            self.assert_velocity(velocity, (0.0, 0.0, 0.0))

    def test_silent_source_expires_while_tcp_heartbeats_continue(self):
        command = (0.3, -0.2, 0.4)
        scenario = ClientScenario(commands=[(0.0, command)]).run()
        self.assert_velocity(scenario.frames[0][1], command)
        expired = [velocity for stamp, velocity in scenario.frames if stamp >= 0.55]
        self.assertTrue(expired)
        for velocity in expired:
            self.assert_velocity(velocity, (0.0, 0.0, 0.0))

    def test_new_source_command_resumes_motion_after_timeout(self):
        command = (-0.1, 0.2, -0.3)
        scenario = ClientScenario(commands=[(0.0, (0.3, 0.0, 0.0)), (0.65, command)]).run()
        self.assertTrue(any(0.55 <= stamp < 0.65 and velocity == (0.0, 0.0, 0.0)
                            for stamp, velocity in scenario.frames))
        self.assert_velocity(scenario.frames[-1][1], command)

    def test_initial_connection_delay_cannot_replay_expired_command(self):
        scenario = ClientScenario(commands=[(0.0, (0.3, 0.1, 0.4))],
                                  connect_delays=(0.75,), duration=0.9).run()
        self.assertTrue(scenario.frames)
        self.assert_velocity(scenario.frames[0][1], (0.0, 0.0, 0.0))

    def test_reconnection_rechecks_command_freshness(self):
        scenario = ClientScenario(commands=[(0.0, (0.3, 0.0, 0.0))], duration=1.0,
                                  connect_delays=(0.0, 0.75), fail_send_at=2).run()
        self.assertEqual(len(scenario.sockets), 2)
        self.assert_velocity(scenario.frames[0][1], (0.3, 0.0, 0.0))
        self.assertGreater(len(scenario.frames), 1)
        for _, velocity in scenario.frames[1:]:
            self.assert_velocity(velocity, (0.0, 0.0, 0.0))

    def test_shutdown_sends_zero_before_closing_connection(self):
        scenario = ClientScenario(commands=[(0.0, (0.3, 0.0, 0.0))], duration=0.2).run()
        self.assertEqual(scenario.shutdown_frames, [(0.0, 0.0, 0.0)])
        scenario.sockets[0].close.assert_called_once()

    def test_source_timeout_must_be_positive_and_finite(self):
        for value in ("0", "-0.5", "nan", "inf", "-inf", "invalid", ""):
            with self.subTest(value=value), patch.dict(os.environ, {"GO2_CMD_SOURCE_TIMEOUT": value}):
                with self.assertRaises(ValueError):
                    tcp._cmd_source_timeout()

    def test_source_timeout_is_configurable(self):
        with patch.dict(os.environ, {"GO2_CMD_SOURCE_TIMEOUT": "0.75"}):
            self.assertEqual(tcp._cmd_source_timeout(), 0.75)


if __name__ == "__main__":
    unittest.main()
