#!/usr/bin/env python3
"""Archive a stopped Jetson run on the laptop; remove remote data only after SHA256 verification."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shlex
import subprocess


def manifest(directory):
    directory = Path(directory)
    if directory.is_symlink():
        raise ValueError('Unexpected symlink for result directory')
    json.loads((directory / 'result.json').read_text())
    if (directory / 'sensors').exists() and not (directory / 'sensors/metadata.yaml').is_file():
        raise ValueError('Bag has not been finalized')
    result = {}
    for path in sorted(directory.rglob('*')):
        if path.is_symlink():
            raise ValueError('Unexpected symlink in result')
        if path.is_file():
            digest = hashlib.sha256()
            with path.open('rb') as stream:
                for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                    digest.update(chunk)
            result[str(path.relative_to(directory))] = [path.stat().st_size, digest.hexdigest()]
    return result


def archive(target, ssh_socket, run, destination, keep_remote=False):
    if not re.fullmatch(r'run-[a-zA-Z0-9_-]+', run):
        raise ValueError('Expected a generated run directory name')
    if not re.fullmatch(r'[a-zA-Z0-9_-]+@[a-zA-Z0-9_.:-]+', target):
        raise ValueError('Expected user@host')
    # Same implementation runs on both machines; only a generated run directory
    # below our isolated experiment may be removed. No shell-expanded paths.
    import inspect
    source = 'import hashlib,json,sys,shutil\nfrom pathlib import Path\n' + inspect.getsource(manifest)
    source += '\nroot=Path.home()/"go2-nav2-lidar3d-onboard/ws/maps/lidar3d"/sys.argv[1]\n'
    ssh = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8']
    if ssh_socket:
        ssh += ['-S', ssh_socket]
    def remote(code, data=None):
        return subprocess.run(ssh + [target, shlex.join(['python3', '-c', source + code, run])],
                              input=data, text=True, capture_output=True, check=True).stdout
    expected = json.loads(remote('print(json.dumps(manifest(root)))\n'))
    destination = Path(destination).resolve()
    destination.mkdir(parents=True, exist_ok=True)
    final = destination / run
    if final.exists():
        if manifest(final) != expected:
            raise ValueError('Existing laptop result differs; both copies retained')
    else:
        incoming = destination / ('.incoming-jetson-' + run)
        incoming.mkdir(exist_ok=True)
        scp = ['scp', '-q', '-r', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8']
        if ssh_socket:
            scp += ['-o', 'ControlPath=' + ssh_socket]
        subprocess.run(scp + [target + ':go2-nav2-lidar3d-onboard/ws/maps/lidar3d/' + run,
                              str(incoming)], check=True)
        if manifest(incoming / run) != expected:
            raise ValueError('Transfer checksum mismatch; Jetson copy retained')
        (incoming / run).rename(final)
        incoming.rmdir()
    if not keep_remote:
        remote('expected=json.load(sys.stdin)\n'
               'if manifest(root)!=expected: raise RuntimeError("Remote result changed; retained")\n'
               'shutil.rmtree(root)\n', json.dumps(expected))
    report = {'archived_to': str(final), 'removed_from_jetson': not keep_remote,
              'files_verified_sha256': len(expected), 'bytes': sum(v[0] for v in expected.values())}
    print(json.dumps(report), flush=True)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--target', required=True)
    parser.add_argument('--ssh-socket')
    parser.add_argument('--run', required=True)
    parser.add_argument('--destination', default=str(Path(__file__).resolve().parents[1] / 'maps/lidar3d'))
    parser.add_argument('--keep-remote', action='store_true')
    args = parser.parse_args()
    try:
        archive(args.target, args.ssh_socket, args.run, args.destination, args.keep_remote)
    except (OSError, ValueError, subprocess.CalledProcessError) as exc:
        parser.exit(1, f'Archive incomplete; Jetson data retained: {exc}\n')


if __name__ == '__main__':
    main()
