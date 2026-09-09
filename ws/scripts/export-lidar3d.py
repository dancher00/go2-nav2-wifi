#!/usr/bin/env python3
"""Validate native 3D backend PCD/TUM output after its owned process has stopped."""
import argparse
import json
import math
from pathlib import Path


def validate_result(directory):
    directory = Path(directory).resolve(strict=True)
    inputs = json.loads((directory / 'input.json').read_text()) if (directory / 'input.json').is_file() else {}
    is_legkilo = inputs.get('backend') == 'legkilo'
    cloud, trajectory = directory / 'PCD/scans.pcd', directory / 'trajectory.tum'
    if is_legkilo:
        # GLog's latest-file symlinks are redundant; retain actual log files.
        for log in (directory / 'Log').rglob('*'):
            if log.is_symlink(): log.unlink()
        cloud = directory / 'export/global_map/global_map.pcd'
        trajectory = directory / 'export/trajectories/backend_imu_tum.txt'

    header = {}
    with cloud.open('rb') as stream:
        for _ in range(40):
            line = stream.readline().decode('ascii').strip()
            if line and not line.startswith('#'):
                key, *values = line.split()
                header[key] = values
                if key == 'DATA':
                    payload_offset = stream.tell()
                    break
        else:
            raise ValueError('Missing PCD DATA header')
    points = int(header['POINTS'][0])
    point_size = sum(int(s)*int(c) for s, c in zip(header['SIZE'], header['COUNT']))
    if points <= 0:
        raise ValueError('Empty PCD')
    if header['DATA'] == ['binary']:
        if cloud.stat().st_size - payload_offset != points * point_size:
            raise ValueError('Truncated PCD')
    elif header['DATA'] == ['binary_compressed'] and is_legkilo:
        import struct
        with cloud.open('rb') as stream:
            stream.seek(payload_offset)
            compressed_size, raw_size = struct.unpack('<II', stream.read(8))
            payload = stream.read()
        if raw_size != points * point_size or len(payload) != compressed_size:
            raise ValueError('Truncated compressed PCD')
        # LZF is the PCD binary_compressed encoding. Validate it without an
        # additional system library; payload coordinates themselves stay untouched.
        decoded = bytearray()
        cursor = 0
        try:
            while cursor < len(payload):
                control = payload[cursor]; cursor += 1
                if control < 32:
                    size = control + 1
                    if cursor + size > len(payload): raise ValueError('Invalid LZF literal')
                    decoded.extend(payload[cursor:cursor+size]); cursor += size
                else:
                    size = control >> 5
                    back = (control & 31) << 8
                    if size == 7:
                        size += payload[cursor]; cursor += 1
                    back += payload[cursor] + 1; cursor += 1
                    size += 2
                    if back > len(decoded): raise ValueError('Invalid LZF reference')
                    pattern = bytes(decoded[-back:])
                    decoded.extend((pattern * ((size + back - 1)//back))[:size])
                if len(decoded) > raw_size: raise ValueError('Invalid LZF size')
        except IndexError as exc:
            raise ValueError('Truncated LZF') from exc
        if len(decoded) != raw_size: raise ValueError('Invalid compressed PCD')

    else:
        raise ValueError('Unsupported PCD')
    poses = [list(map(float, line.split())) for line in trajectory.read_text().splitlines() if line.strip() and not line.startswith('#')]
    if not poses or any(len(p) != 8 or not all(math.isfinite(v) for v in p) or abs(sum(v*v for v in p[4:])-1) > .01 for p in poses):
        raise ValueError('Empty or invalid TUM trajectory')
    if any(b[0] <= a[0] for a, b in zip(poses, poses[1:])):
        raise ValueError('Non-increasing trajectory stamps')
    displacement = max(math.sqrt(sum((p[i]-poses[0][i])**2 for i in (1,2,3))) for p in poses)
    report = {'backend': 'CMU Point-LIO ROS2 adaptation', 'loop_closure': False,
              'map': str(cloud), 'trajectory': str(trajectory), 'points': points,
              'trajectory_poses': len(poses), 'trajectory_duration_s': poses[-1][0]-poses[0][0],
              'max_displacement_from_first_m': displacement,
              'trajectory_format': 'TUM: stamp x y z qx qy qz qw; camera_init -> aft_mapped (initial IMU coordinates)',
              'note': 'File integrity and nonempty geometry do not establish mapping accuracy.'}
    if (directory / 'input.json').is_file():
        inputs = json.loads((directory / 'input.json').read_text())
        for key in ('imu_enabled', 'imu_acceleration_used', 'imu_as_input', 'replay_rate', 'compute'):
            if key in inputs:
                report[key] = inputs[key]
    if is_legkilo:
        report.update(backend='Leg-KILO ROS2 / Go2 kinematics adaptation', loop_closure=inputs['loop_closure'],
                      kinematics_used=True, factory_body_pose_used=False,
                      trajectory_format='TUM: stamp x y z qx qy qz qw; lidar3d_map -> body; backend optimized trajectory',
                      live_visualization=inputs.get('live_visualization', 'Frontend local map/trajectory; exported result uses optimized backend submaps'),
                      contact_thresholds_provisional=True)
    if (directory / 'output-frame.json').is_file():
        frame = json.loads((directory / 'output-frame.json').read_text())
        report['output_frame'] = frame
        report['trajectory_format'] = 'TUM: stamp x y z qx qy qz qw; ' + frame['frame'] + ' -> aft_mapped'
    with (directory / 'result.json').open('x') as stream:
        json.dump(report, stream, indent=2)
        stream.write('\n')
    print(json.dumps(report, indent=2), flush=True)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', help='Result directory of a stopped session')
    args = parser.parse_args()
    try:
        validate_result(args.directory)
    except (OSError, ValueError, KeyError) as exc:
        parser.exit(1, 'Result validation failed (native files preserved): ' + str(exc) + '\n')


if __name__ == '__main__':
    main()
