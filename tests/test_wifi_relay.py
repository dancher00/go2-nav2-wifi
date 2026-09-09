"""Relay regressions: sockets are local; no ROS graph or robot is started."""

import importlib.util
import os
from pathlib import Path
import socket
import tempfile
import unittest
from unittest.mock import Mock, patch

SCRIPT = Path(__file__).resolve().parents[1] / "ws/scripts/robot_relay_wifi.py"
SPEC = importlib.util.spec_from_file_location("wifi_relay", SCRIPT)
relay = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(relay)


class DomainTests(unittest.TestCase):
    def test_jetson_profile_exports_visualization_and_leg_states(self):
        with patch.dict(os.environ, GO2_RELAY_PROFILE='lidar3d-map'), \
                patch.object(relay, '_RELAY_TOPICS', None), patch.object(relay, 'get_message'):
            topics = relay.relay_topics()
        self.assertEqual([row[0] for row in topics],
                         ['/lidar3d/registered', '/lidar3d/odom', '/lidar3d/path', '/lf/lowstate'])
        for _, _, kind in topics[:3]:
            self.assertEqual(relay._qos(kind).reliability, relay.ReliabilityPolicy.BEST_EFFORT)

    def test_lidar3d_profile_only_relays_native_cloud_imu_and_clock_reference(self):
        with patch.dict(os.environ, GO2_RELAY_PROFILE='lidar3d'), \
                patch.object(relay, '_RELAY_TOPICS', None), patch.object(relay, 'get_message'):
            topics = relay.relay_topics()
        self.assertEqual([row[0] for row in topics],
                         ['/utlidar/cloud', '/utlidar/robot_odom', '/utlidar/imu', '/lf/lowstate'])
        self.assertEqual(relay._qos(topics[2][2]).depth, 200)
        self.assertEqual(relay._qos(topics[0][2]).depth, 1)

    def test_default_relay_profile_is_preserved(self):
        with patch.dict(os.environ, GO2_RELAY_PROFILE='default'), \
                patch.object(relay, '_RELAY_TOPICS', None), patch.object(relay, 'get_message'):
            self.assertEqual(relay.relay_topics(), relay.RELAY_TOPICS_CANDIDATES)

    def test_roles_never_share_a_domain(self):
        for role, expected in (("sub", 0), ("pub_cmd", 0), ("pub", 64), ("sub_cmd", 64)):
            with self.subTest(role=role), patch.dict(os.environ, {"CYCLONEDDS_URI": "wifi"}, clear=True):
                self.assertEqual(relay.configure_domain(role), expected)
                self.assertEqual(os.environ["ROS_DOMAIN_ID"], str(expected))
                self.assertEqual(os.environ.get("CYCLONEDDS_URI"), None if expected == 0 else "wifi")

    def test_custom_domain_overrides_inherited_ros_domain(self):
        with patch.dict(os.environ, {"GO2_RELAY_DOMAIN_ID": "007", "ROS_DOMAIN_ID": "0"}):
            self.assertEqual(relay.configure_domain("pub"), 7)
            self.assertEqual(os.environ["ROS_DOMAIN_ID"], "7")

    def test_invalid_domains_fail_before_ros_start(self):
        for value in ("0", "000", "-1", "102", "9999", "nan", "", "1.5", " 64"):
            with self.subTest(value=value), patch.dict(os.environ, {"GO2_RELAY_DOMAIN_ID": value}):
                with self.assertRaises(ValueError):
                    relay.configure_domain("pub")


class TransportTests(unittest.TestCase):
    def test_visualization_forwards_cdr_unchanged_without_decoding_skipped_paths(self):
        node, sock = Mock(), Mock()
        callbacks = []
        node.create_subscription.side_effect = lambda cls, topic, cb, qos, **kw: callbacks.append((cb, kw))
        payload = b'original CDR path'
        def spin(*args, **kwargs):
            callbacks[0][0](payload)
            callbacks[0][0](payload)  # Same instant: next Path is visualization-only decimated.
        with patch.dict(os.environ, GO2_RELAY_PROFILE='lidar3d-map'), \
                patch.object(relay, 'Node', return_value=node), \
                patch.object(relay.rclpy, 'init'), patch.object(relay.rclpy, 'ok', side_effect=[True, False]), \
                patch.object(relay.rclpy, 'spin_once', side_effect=spin), \
                patch.object(relay, '_wait_for_ready', return_value=True), \
                patch.object(relay, '_connect', return_value=sock), patch.object(relay, '_shutdown'), \
                patch.object(relay, '_send_frame') as send, patch.object(relay, 'get_message'), \
                patch.object(relay, 'serialize_message') as serialize:
            relay._run_subscriber('socket', 'ready', [('/lidar3d/path', 'nav_msgs/msg/Path', 'sensor')], 'test')
        send.assert_called_once_with(sock, 0, payload)
        serialize.assert_not_called()
        self.assertTrue(callbacks[0][1]['raw'])

    def test_all_failed_connections_close_sockets_and_raise(self):
        sockets = [Mock(), Mock(), Mock()]
        for sock in sockets:
            sock.connect.side_effect = ConnectionRefusedError()
        with patch.object(relay.socket, "socket", side_effect=sockets), \
                patch.object(relay.rclpy, "ok", return_value=True), patch.object(relay.time, "sleep"):
            with self.assertRaises(ConnectionError):
                relay._connect("unused", attempts=3)
        for sock in sockets:
            sock.close.assert_called_once()

    def test_successful_retry_only_returns_connected_socket(self):
        bad, good = Mock(), Mock()
        bad.connect.side_effect = ConnectionRefusedError()
        with patch.object(relay.socket, "socket", side_effect=[bad, good]), \
                patch.object(relay.rclpy, "ok", return_value=True), patch.object(relay.time, "sleep"):
            self.assertIs(relay._connect("unused", attempts=2), good)
        bad.close.assert_called_once()
        good.close.assert_not_called()

    def test_partial_frame_survives_timeout(self):
        sock = Mock()
        sock.recv.side_effect = [b"ab", socket.timeout(), b"cd"]
        with patch.object(relay.rclpy, "ok", return_value=True):
            self.assertEqual(relay._recv_exact(sock, 4), b"abcd")

    def test_recv_observes_shutdown(self):
        with patch.object(relay.rclpy, "ok", return_value=False):
            with self.assertRaises(relay.ExternalShutdownException):
                relay._recv_exact(Mock(), 1)

    def test_peer_disconnect_is_not_an_empty_frame(self):
        with patch.object(relay.rclpy, "ok", return_value=True):
            with self.assertRaises(ConnectionError):
                relay._recv_exact(Mock(recv=Mock(return_value=b"")), 1)

    def test_ready_file_means_socket_accepts_connections(self):
        with tempfile.TemporaryDirectory() as directory:
            path, ready = f"{directory}/socket", f"{directory}/ready"
            server = relay._listen(path, ready)
            try:
                self.assertTrue(Path(ready).is_file())
                with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
                    client.connect(path)
                    conn, _ = server.accept()
                    conn.close()
            finally:
                server.close()

    def test_failed_bind_does_not_remove_an_existing_socket(self):
        with tempfile.TemporaryDirectory() as directory:
            path, ready = f"{directory}/socket", f"{directory}/ready"
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as existing:
                existing.bind(path)
                with self.assertRaises(OSError):
                    relay._listen(path, ready)
                self.assertTrue(Path(path).exists())
                self.assertFalse(Path(ready).exists())

    def test_failed_ready_creation_removes_owned_socket(self):
        with tempfile.TemporaryDirectory() as directory:
            path = f"{directory}/socket"
            with self.assertRaises(OSError):
                relay._listen(path, f"{directory}/missing/ready")
            self.assertFalse(Path(path).exists())

    def test_shutdown_is_idempotent(self):
        for active in (True, False):
            node = Mock()
            with patch.object(relay.rclpy, "ok", return_value=active), \
                    patch.object(relay.rclpy, "shutdown") as shutdown:
                relay._shutdown(node)
                self.assertEqual(shutdown.call_count, int(active))
                node.destroy_node.assert_called_once()


if __name__ == "__main__":
    unittest.main()
