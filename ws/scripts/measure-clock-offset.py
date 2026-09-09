#!/usr/bin/env python3
"""Bound robot-minus-laptop wall-clock offset with an existing SSH control socket."""
import argparse
import json
from pathlib import Path
import shlex
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('target', help='SSH user@host; authentication stays in SSH')
    parser.add_argument('--socket', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--count', type=int, default=30)
    args = parser.parse_args()
    code = 'import sys,time\nfor line in sys.stdin:\n a=time.time_ns(); b=time.time_ns(); print(a,b,flush=True)'
    command = ['ssh', '-S', args.socket, '-o', 'BatchMode=yes', args.target,
               'python3 -u -c ' + shlex.quote(code)]
    with Path(args.output).open('x') as output:
        child = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
        rows = []
        try:
            for _ in range(args.count):
                t0 = time.time_ns()
                child.stdin.write('sample\n')
                child.stdin.flush()
                line = child.stdout.readline()
                t3 = time.time_ns()
                if not line:
                    raise RuntimeError('SSH clock probe ended without a reply')
                t1, t2 = map(int, line.split())
                rows.append({'t0': t0, 't1': t1, 't2': t2, 't3': t3,
                             'offset_ns': (t1-t0+t2-t3)/2,
                             'uncertainty_ns': ((t3-t0)-(t2-t1))/2})
        finally:
            child.stdin.close()
            child.wait(timeout=10)
        best = min(rows, key=lambda row: row['uncertainty_ns'])
        result = {'robot_minus_laptop_ns': best['offset_ns'],
                  'uncertainty_ns': best['uncertainty_ns'], 'wall_ns': best['t3'], 'samples': rows,
                  'method': 'Four timestamps over SSH; midpoint estimate assumes symmetric paths. Half RTT bounds asymmetry; no system clocks changed.'}
        json.dump(result, output, indent=2)
        output.write('\n')
        print(json.dumps({k:v for k,v in result.items() if k!='samples'}, indent=2))


if __name__ == '__main__':
    main()
