#!/usr/bin/env python3
"""Compare source/laptop observers by immutable sensor stamps, with clock bounds."""
import argparse
import json
import math
from pathlib import Path
import statistics


def stats(values):
    if not values:
        return {}
    values = sorted(values)
    return {'min': values[0], 'median': statistics.median(values),
            'p95': values[min(len(values)-1, math.ceil(.95*len(values))-1)], 'max': values[-1]}


def compare(source, laptop, before, after):
    theta = (before['robot_minus_laptop_ns'] + after['robot_minus_laptop_ns']) / 2
    uncertainty = max(before['uncertainty_ns'], after['uncertainty_ns']) + abs(
        before['robot_minus_laptop_ns'] - after['robot_minus_laptop_ns']) / 2
    report = {'clock_robot_minus_laptop_ms': theta/1e6,
              'clock_uncertainty_ms': uncertainty/1e6,
              'latency_scope': 'Source observer callback to laptop observer callback for identical source stamps. Includes DDS/relay/network/executor scheduling; not sensor acquisition-to-map latency.',
              'loss_scope': 'Source-only/laptop-only are differences between BEST_EFFORT observers, not wire-packet loss counters.',
              'transport': {}, 'timestamp_pipeline': {}}
    pipeline = laptop.get('side') == 'pipeline'
    endpoint = dict(laptop['topics'])
    offset = laptop['topics'].get('/go2/sensor_time_offset_ns', {}).get('offset_ns')
    if pipeline:
        if offset is None:
            raise ValueError('Pipeline observer requires the latched source-clock offset')
        for original, translated in (('/utlidar/cloud', '/lidar3d/cloud_sync'),
                                     ('/utlidar/imu', '/lidar3d/imu_sync'),
                                     ('/utlidar/robot_odom', '/lidar3d/factory_odom_clock_reference')):
            endpoint[original] = {'samples': [[r[0]-offset, *r[1:]] for r in laptop['topics'][translated]['samples']]}
        report['latency_scope'] = 'Source observer callback to laptop corrected-sensor observer callback; includes relay, network, clock-translator policy/executor and observer scheduling. Subtract fixed offset only to match source stamps; not acquisition-to-map latency.'
    for topic in ('/utlidar/cloud', '/utlidar/imu', '/utlidar/robot_odom'):
        src = {r[0]: r for r in source['topics'][topic]['samples']}
        dst = {r[0]: r for r in endpoint[topic]['samples']}
        if not src or not dst:
            report['transport'][topic] = {'error': 'missing stream'}
            continue
        low, high = max(min(src), min(dst))+10**9, min(max(src), max(dst))-10**9
        a = {k:v for k,v in src.items() if low<=k<=high}
        b = {k:v for k,v in dst.items() if low<=k<=high}
        common = sorted(a.keys() & b.keys())
        report['transport'][topic] = {
            'overlap_s': (high-low)/1e9, 'source_observed': len(a), 'laptop_observed': len(b),
            'matched': len(common), 'source_only': len(a.keys()-b.keys()),
            'laptop_only': len(b.keys()-a.keys()),
            'observer_transfer_ms': stats([(b[k][1]-a[k][1]+theta)/1e6 for k in common]),
            'cloud_payload_crc_mismatches': sum(a[k][4]!=b[k][4] for k in common),
        }
    offset = laptop['topics'].get('/go2/sensor_time_offset_ns', {}).get('offset_ns')
    report['locked_sensor_translation_ns'] = offset
    if offset is not None:
        pairs = [('/utlidar/cloud', '/lidar3d/cloud_sync', offset),
                 ('/utlidar/imu', '/lidar3d/imu_sync', offset),
                 ('/lidar3d/odom', '/lidar3d/registered', 0)]
        if pipeline:
            pairs = [('/lidar3d/odom', '/lidar3d/registered', 0)]
        for first, second, translation in pairs:
            a = {r[0]+translation:r for r in laptop['topics'][first]['samples']}
            b = {r[0]:r for r in laptop['topics'][second]['samples']}
            if not a or not b:
                report['timestamp_pipeline'][first+' -> '+second] = {'error': 'missing stream'}
                continue
            low, high = max(min(a),min(b))+10**9, min(max(a),max(b))-10**9
            ka, kb = {k for k in a if low<=k<=high}, {k for k in b if low<=k<=high}
            common = sorted(ka & kb)
            report['timestamp_pipeline'][first+' -> '+second] = {
                'matched': len(common), 'input_only': len(ka-kb), 'output_only': len(kb-ka),
                'cloud_data_crc_mismatches': sum(a[k][4] != b[k][4] for k in common) if second.endswith('cloud_sync') else None,
                'observer_processing_ms': stats([(b[k][2]-a[k][2])/1e6 for k in common])}
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source')
    parser.add_argument('laptop')
    parser.add_argument('clock_before')
    parser.add_argument('clock_after')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    result = compare(*(json.loads(Path(path).read_text()) for path in (
        args.source, args.laptop, args.clock_before, args.clock_after)))
    with Path(args.output).open('x') as stream:
        json.dump(result, stream, indent=2)
        stream.write('\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
