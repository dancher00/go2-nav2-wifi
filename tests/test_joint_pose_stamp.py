from types import SimpleNamespace
from unittest.mock import Mock, patch
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from go2_nav2.joint_state_bridge import JointStateBridge


def test_3d_joints_share_pose_stamp_and_stale_measurements_are_not_republished():
    bridge = JointStateBridge.__new__(JointStateBridge)
    bridge._stamp_odom_topic = '/lidar3d/odom'
    bridge._latest_positions = None
    bridge._received_positions = 0
    bridge._pub = Mock()
    low = SimpleNamespace(motor_state=[SimpleNamespace(q=.1*i) for i in range(12)])
    odom = Odometry(); odom.header.stamp = Time(sec=100, nanosec=123)
    with patch('go2_nav2.joint_state_bridge.time.monotonic', return_value=5.):
        bridge._on_odom(odom)
        bridge._pub.publish.assert_not_called()
        bridge._on_lowstate(low)
        bridge._pub.publish.assert_not_called()
    with patch('go2_nav2.joint_state_bridge.time.monotonic', return_value=5.1):
        bridge._on_odom(odom)
    out = bridge._pub.publish.call_args.args[0]
    assert out.header.stamp == odom.header.stamp
    assert len(out.position) == 12
    assert out.position[5] == .5
    with patch('go2_nav2.joint_state_bridge.time.monotonic', return_value=5.3):
        bridge._on_odom(odom)
    assert bridge._pub.publish.call_count == 1
