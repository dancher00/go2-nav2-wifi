#!/usr/bin/env python3
"""Request a Nav2 plan in the running --3d --plan session, without following it."""
import argparse
import json
import math
import os
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('x', type=float, help='Goal X in lidar3d_map, metres')
    parser.add_argument('y', type=float, help='Goal Y in lidar3d_map, metres')
    args = parser.parse_args()
    if not all(math.isfinite(v) for v in (args.x, args.y)):
        parser.error('Goal coordinates must be finite')
    goal = {'goal': {'header': {'frame_id': 'lidar3d_map'},
                     'pose': {'position': {'x': args.x, 'y': args.y}, 'orientation': {'w': 1.0}}},
            'planner_id': 'GridBased', 'use_start': False}
    return subprocess.call([
        'docker', 'exec', os.environ.get('GO2_CONTAINER', 'go2-lidar3d'),
        'bash', '-c', 'source /ws/scripts/go2-env.sh && exec ros2 action send_goal "$@"',
        'plan-lidar3d', '/compute_path_to_pose', 'nav2_msgs/action/ComputePathToPose', json.dumps(goal)])


if __name__ == '__main__':
    raise SystemExit(main())
