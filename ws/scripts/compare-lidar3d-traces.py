#!/usr/bin/env python3
"""Compare in-process relay/clock traces without adding DDS subscribers."""
import argparse
import importlib.util
import json
from pathlib import Path

spec = importlib.util.spec_from_file_location('transport_compare', Path(__file__).with_name('compare-lidar3d-transport.py'))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source_trace')
    parser.add_argument('clock_trace')
    parser.add_argument('clock_before')
    parser.add_argument('clock_after')
    parser.add_argument('--output', required=True)
    parser.add_argument('--start-wall-ns', type=int, help='Optional laptop wall-clock window start')
    parser.add_argument('--end-wall-ns', type=int, help='Optional laptop wall-clock window end')
    args = parser.parse_args()
    clocks = [json.loads(Path(p).read_text()) for p in (args.clock_before, args.clock_after)]
    theta = sum(c['robot_minus_laptop_ns'] for c in clocks)/2
    def in_window(wall):
        return (args.start_wall_ns is None or wall >= args.start_wall_ns) and (args.end_wall_ns is None or wall <= args.end_wall_ns)
    original = {'cloud': '/utlidar/cloud', 'imu': '/utlidar/imu', 'odom': '/utlidar/robot_odom'}
    translated = {'cloud': '/lidar3d/cloud_sync', 'imu': '/lidar3d/imu_sync', 'odom': '/lidar3d/factory_odom_clock_reference'}
    source = {'topics': {t: {'samples': []} for t in original.values()}}
    laptop = {'side': 'pipeline', 'topics': {t: {'samples': []} for t in translated.values()}}
    for line in Path(args.source_trace).read_text().splitlines():
        topic, *row = json.loads(line)
        if in_window(row[1]-theta):
            source['topics'][topic]['samples'].append(row)
    offsets = set()
    for line in Path(args.clock_trace).read_text().splitlines():
        stream, stamp, wall, mono, corrected, crc = json.loads(line)
        if not in_window(wall):
            continue
        offsets.add(corrected-stamp)
        laptop['topics'][translated[stream]]['samples'].append([corrected, wall, mono, 0, crc])
    if len(offsets) != 1:
        raise ValueError('Expected one immutable sensor-clock offset')
    laptop['topics']['/go2/sensor_time_offset_ns'] = {'offset_ns': offsets.pop()}
    for topic in ('/lidar3d/odom', '/lidar3d/registered'):
        laptop['topics'][topic] = {'samples': []}
    report = module.compare(source, laptop, *clocks)
    report.pop('timestamp_pipeline')
    report['latency_scope'] = 'Existing robot relay subscription callback -> existing laptop clock translator before publish. Includes relay IPC, DDS/Wi-Fi and laptop scheduling; not acquisition-to-map latency.'
    report['loss_scope'] = 'Relay-input stamps missing from accepted clock outputs in trimmed overlap; includes transport loss, queueing, stale/reference policy. No additional DDS observer.'
    report['laptop_wall_window_ns'] = [args.start_wall_ns, args.end_wall_ns]
    with Path(args.output).open('x') as output:
        json.dump(report, output, indent=2)
        output.write('\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
