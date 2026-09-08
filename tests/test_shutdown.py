"""Run passive-node main functions with fake ROS contexts (no Unitree imports)."""

import ast
from pathlib import Path
import unittest
from unittest.mock import Mock

from rclpy.executors import ExternalShutdownException

ROOT = Path(__file__).resolve().parents[1]
NODES = {
    "go2_nav2/go2_nav2/cloud_stamp_sync.py": "CloudStampSync",
    "go2_nav2/go2_nav2/joint_state_bridge.py": "JointStateBridge",
    "go2_nav2/go2_nav2/odom_tf.py": "OdomTf",
    "go2_nav2/go2_nav2/sport_state_odom.py": "SportStateOdom",
    "go2_nav2/go2_nav2/waypoint_recorder.py": "WaypointRecorder",
    "ws/scripts/robot_front_camera_bridge.py": "FrontCameraBridge",
}


class ShutdownTests(unittest.TestCase):
    def test_sport_bridge_stop_precedes_context_shutdown(self):
        for filename in ("ws/scripts/robot_sport_bridge.py", "go2_nav2/go2_nav2/sport_bridge.py"):
            tree = ast.parse((ROOT / filename).read_text())
            main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "main")
            for active in (False, True):
                with self.subTest(filename=filename, active=active):
                    ros, node, signals = Mock(), Mock(), Mock()
                    signals.SIGINT, signals.SIGTERM = 2, 15
                    ros.ok.return_value = active
                    ros.spin.side_effect = KeyboardInterrupt()
                    order = []
                    node._send.side_effect = lambda _: order.append("stop")
                    node.destroy_node.side_effect = lambda: order.append("destroy")
                    ros.shutdown.side_effect = lambda: order.append("shutdown")
                    scope = {"rclpy": ros, "SportBridge": Mock(return_value=node), "signal": signals,
                             "API_STOP_MOVE": 1003, "ExternalShutdownException": ExternalShutdownException}
                    exec(compile(ast.Module(body=[main], type_ignores=[]), filename, "exec"), scope)
                    scope["main"]()
                    self.assertEqual(order, ["stop", "destroy", "shutdown"] if active else ["destroy"])
                    self.assertEqual(signals.signal.call_count, 4)

    def test_signal_shutdown_does_not_shutdown_context_twice(self):
        for filename, class_name in NODES.items():
            tree = ast.parse((ROOT / filename).read_text())
            main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "main")
            for exception in (KeyboardInterrupt, ExternalShutdownException):
                for active in (False, True):
                    with self.subTest(filename=filename, exception=exception, active=active):
                        ros = Mock()
                        ros.spin.side_effect = exception()
                        ros.ok.return_value = active
                        node = Mock()
                        signals = Mock(SIGINT=2, SIGTERM=15, SIG_IGN=0)
                        scope = {"rclpy": ros, class_name: Mock(return_value=node), "signal": signals,
                                 "ExternalShutdownException": ExternalShutdownException}
                        exec(compile(ast.Module(body=[main], type_ignores=[]), filename, "exec"), scope)
                        scope["main"]()
                        node.destroy_node.assert_called_once()
                        self.assertEqual(ros.shutdown.call_count, int(active))
                        signals.signal.assert_any_call(2, 0)
                        signals.signal.assert_any_call(15, 0)

    def test_repeated_real_signals_do_not_interrupt_cleanup(self):
        # Run just the actual main() against a slow fake destroy_node, in a child
        # interpreter so the test never changes the parent's signal handlers.
        import subprocess
        import sys
        program = '''
import ast
import os
from pathlib import Path
import signal
import sys
import time
from types import SimpleNamespace
filename = sys.argv[1]
tree = ast.parse(Path(filename).read_text())
main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
def spin(node):
    raise KeyboardInterrupt()
def destroy():
    os.kill(os.getpid(), signal.SIGINT)
    os.kill(os.getpid(), signal.SIGTERM)
    time.sleep(0.02)
node = SimpleNamespace(destroy_node=destroy)
ros = SimpleNamespace(init=lambda: None, spin=spin, ok=lambda: False)
scope = dict(rclpy=ros, signal=signal, CloudStampSync=lambda: node,
             ExternalShutdownException=RuntimeError)
exec(compile(ast.Module(body=[main], type_ignores=[]), filename, 'exec'), scope)
scope['main']()
'''
        result = subprocess.run([sys.executable, "-B", "-c", program,
                                 str(ROOT / "go2_nav2/go2_nav2/cloud_stamp_sync.py")],
                                capture_output=True, text=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
