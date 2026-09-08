"""Validate localization inputs and publish new map bundles without overwriting."""

import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import re
import subprocess
import tempfile

import yaml


def validate_map(path):
    path = Path(path).expanduser().resolve()
    if path.suffix not in ('.yaml', '.yml') or not path.is_file():
        raise ValueError(f'Map YAML not found: {path}')
    with path.open() as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError('Map YAML must be a mapping')
    resolution = config.get('resolution')
    origin = config.get('origin')
    if not isinstance(resolution, (int, float)) or not math.isfinite(resolution) or resolution <= 0:
        raise ValueError('Map resolution must be positive and finite')
    if not isinstance(origin, list) or len(origin) != 3 or not all(
            isinstance(v, (int, float)) and math.isfinite(v) for v in origin):
        raise ValueError('Map origin must contain three finite numbers')
    image = config.get('image')
    if not isinstance(image, str) or not image:
        raise ValueError('Map YAML has no image path')
    files = [path.parent / image, path.with_suffix('.posegraph'), path.with_suffix('.data')]
    for item in files:
        if not item.is_file() or item.stat().st_size == 0:
            raise ValueError(f'Incomplete map bundle: missing/empty {item}')
    return path


def publish_bundle(staged, destination):
    """Exclusive hard links on the same filesystem; YAML is the commit marker."""
    published = []
    try:
        for suffix in ('.pgm', '.posegraph', '.data', '.yaml'):
            source = staged.with_suffix(suffix)
            target = destination.with_suffix(suffix)
            os.link(source, target)  # EEXIST, including symlinks: never replace.
            published.append((target, source.stat().st_ino))
    except BaseException:
        for target, inode in reversed(published):
            try:
                if target.lstat().st_ino == inode:
                    target.unlink()
            except FileNotFoundError:
                pass
        raise


def save_map(name, directory):
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]*', name):
        raise ValueError('Use a map name containing only letters, digits, _ and -')
    directory = Path(directory).expanduser().resolve()
    directory.mkdir(parents=True, exist_ok=True)
    destination = directory / name
    lock_fd = os.open(str(directory / ('.' + name + '.save.lock')),
                      os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    try:
        fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        for suffix in ('.yaml', '.pgm', '.posegraph', '.data'):
            if os.path.lexists(destination.with_suffix(suffix)):
                raise ValueError(f'Map already exists: {destination}{suffix}; choose a new name')
        with tempfile.TemporaryDirectory(prefix='.' + name + '-saving-', dir=directory) as stage:
            staged = Path(stage) / name
            print('Saving a new map bundle; existing maps will not be overwritten.', flush=True)
            subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(staged),
                            '--ros-args', '-p', 'save_map_timeout:=10.0'], check=True, timeout=25)
            subprocess.run(['ros2', 'service', 'call', '/slam_toolbox/serialize_map',
                            'slam_toolbox/srv/SerializePoseGraph',
                            json.dumps({'filename': str(staged)})], check=True, timeout=25)
            validate_map(staged.with_suffix('.yaml'))
            # map_saver may write an absolute staging path: make the bundle portable.
            config = yaml.safe_load(staged.with_suffix('.yaml').read_text())
            config['image'] = name + '.pgm'
            staged.with_suffix('.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
            publish_bundle(staged, destination)
        print(f'Saved complete map: {destination}.yaml (.pgm, .posegraph, .data)', flush=True)
    finally:
        os.close(lock_fd)  # Keep lock inode to prevent concurrent-lock races.


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation', choices=('check', 'save'))
    parser.add_argument('name_or_path')
    parser.add_argument('directory', nargs='?', default='/ws/maps')
    args = parser.parse_args()
    try:
        if args.operation == 'check':
            print(validate_map(args.name_or_path))
        else:
            save_map(args.name_or_path, args.directory)
    except (ValueError, OSError, yaml.YAMLError, subprocess.SubprocessError) as exc:
        parser.exit(1, f'ERROR: {exc}\n')


if __name__ == '__main__':
    main()
