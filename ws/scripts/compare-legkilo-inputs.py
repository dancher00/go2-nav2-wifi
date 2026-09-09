#!/usr/bin/env python3
"""Compare finalized GO2_LEGKILO_INPUT_TRACE files, before estimator updates."""
import argparse
import csv
import json
from pathlib import Path


def read_trace(path):
    frames = {}
    current = None
    for row in csv.reader(Path(path).read_text().splitlines()):
        if len(row) == 9 and row[0] == 'frame':
            frame = int(row[1])
            if frame in frames:
                raise ValueError(f'Duplicate frame {frame} in {path}')
            current = {'fields': row[2:], 'kin': []}
            frames[frame] = current
        elif len(row) == 6 and row[0] == 'kin':
            if current is None or int(row[1]) not in frames or frames[int(row[1])] is not current:
                raise ValueError(f'Kinematic row outside its frame in {path}')
            current['kin'].append(row[2:])
        else:
            raise ValueError(f'Malformed trace row in {path}: {row[:2]}')
    if not frames:
        raise ValueError(f'Empty trace: {path}')
    for frame, value in frames.items():
        if int(value['fields'][6]) != len(value['kin']):
            raise ValueError(f'Incomplete frame {frame} in {path}; stop the session before comparing')
    return frames


def compare(a, b):
    common = sorted(a.keys() & b.keys())
    result = {'frames': [len(a), len(b)],
              'only_first': sorted(a.keys() - b.keys()),
              'only_second': sorted(b.keys() - a.keys())}
    for name, indices in [('times', (0, 1)), ('raw_cloud', (2, 3)), ('downsampled_cloud', (4, 5))]:
        changed = [i for i in common if any(a[i]['fields'][j] != b[i]['fields'][j] for j in indices)]
        result[name] = {'different_frames': len(changed), 'first': changed[0] if changed else None}
    changed = [i for i in common if a[i]['kin'] != b[i]['kin']]
    result['kinematics'] = {'different_frames': len(changed), 'first': changed[0] if changed else None}
    result['identical_inputs'] = not (result['only_first'] or result['only_second'] or
                                     any(result[k]['different_frames'] for k in
                                         ('times', 'raw_cloud', 'downsampled_cloud', 'kinematics')))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('first')
    parser.add_argument('second')
    args = parser.parse_args()
    result = compare(read_trace(args.first), read_trace(args.second))
    print(json.dumps(result, indent=2))
    return 0 if result['identical_inputs'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
