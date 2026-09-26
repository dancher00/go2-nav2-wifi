import rclpy
from go2_nav2.urdf_self_filter import UrdfSelfFilter


def test_reporting_recovers_from_missing_transform_without_logger_crash():
    rclpy.init()
    node = UrdfSelfFilter()
    try:
        node.report()
        node.unfiltered = 1
        node.report()
        node.report()
        assert node.unfiltered == 0
    finally:
        node.destroy_node()
        rclpy.shutdown()
