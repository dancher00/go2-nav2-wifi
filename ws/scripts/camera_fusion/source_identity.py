"""Identify the existing odometry publisher before a color-only restart."""
import json
import time
from pathlib import Path
import rclpy


def identity(node, cfg, timeout=8):
    topic=cfg.get('odom_topic','/lidar3d/odom')
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        pubs=node.get_publishers_info_by_topic(topic)
        if len(pubs)>1: raise RuntimeError('Ambiguous odometry publishers')
        if len(pubs)==1:
            return {'odom_topic':topic,'map_frame':cfg['map_frame'],
                    'sensor_frame':cfg['sensor_frame'],
                    'gid':bytes(pubs[0].endpoint_gid).hex()}
        rclpy.spin_once(node,timeout_sec=.1)
    raise RuntimeError('No existing odometry publisher')


if __name__=='__main__':
    rclpy.init(args=[])
    node=rclpy.create_node('camera_fusion_source_check')
    try:
        cfg=json.loads(Path('/ipc/calibration.json').read_text())
        result=identity(node,cfg)
        Path('/data/session/source.json').write_text(json.dumps(result)+'\n')
        print(json.dumps(result))
    finally:
        node.destroy_node();rclpy.try_shutdown()
