#!/usr/bin/env python3
"""Foreground session supervisor; control sockets, owned children, no PID-file kills."""

import argparse
import fcntl
import json
import os
from pathlib import Path
import select
import signal
import socket
import stat
import subprocess
import sys
import time

MODES = ('mapping', 'navigation', 'teleop', 'transport')
RESOURCES = {'mapping': ('stack',), 'navigation': ('stack', 'motion'),
             'teleop': ('motion',), 'transport': ('motion',)}


def runtime_dir():
    domain = int(os.environ.get('GO2_RELAY_DOMAIN_ID', '64')) if os.environ.get('GO2_NET', 'wifi') == 'wifi' else 0
    if not 0 <= domain <= 232:
        raise ValueError('Invalid ROS domain')
    root = Path(os.environ.get('GO2_SESSION_DIR', f'/tmp/go2-sessions-{os.getuid()}-{domain}'))
    root.mkdir(mode=0o700, parents=True, exist_ok=True)
    info = root.lstat()
    if not stat.S_ISDIR(info.st_mode) or info.st_uid != os.getuid() or info.st_mode & 0o077:
        raise ValueError(f'Session directory must be owned by this UID and private (0700): {root}')
    return root


def acquire_locks(root, mode):
    locks = []
    try:
        for name in RESOURCES[mode]:
            fd = os.open(str(root / (name + '.lock')), os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
            locks.append(fd)
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        return locks
    except BaseException:
        for fd in locks:
            os.close(fd)
        raise


def request(root, mode, action):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
        client.settimeout(12 if action == 'stop' else 1)
        try:
            client.connect(str(root / (mode + '.sock')))
        except (FileNotFoundError, ConnectionRefusedError):
            return {'mode': mode, 'state': 'not-managed'}
        client.sendall(action.encode())
        return json.loads(client.recv(8192))


def stop_children(children):
    # Only groups created by this supervisor, never matches from pgrep/ROS graph.
    # Keep leaders unreaped until group cleanup has finished, preventing PID reuse.
    for sig, duration in ((signal.SIGINT, 5), (signal.SIGTERM, 2), (signal.SIGKILL, 0)):
        for child in reversed(children):
            try:
                os.killpg(child.pid, sig)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            if all(os.waitid(os.P_PID, p.pid, os.WEXITED | os.WNOHANG | os.WNOWAIT)
                   for p in children):
                break
            time.sleep(.05)
    for child in children:
        child.wait()


def supervise(root, mode, commands, preflight=None):
    locks = acquire_locks(root, mode)
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    path = root / (mode + '.sock')
    children = []
    stopping = False
    stop_client = None
    bound = False
    handlers = {}

    def stop_signal(*_):
        nonlocal stopping
        stopping = True

    try:
        # The lock is held; a socket left by a crashed owner is no longer active.
        if path.exists():
            if request(root, mode, 'status')['state'] != 'not-managed':
                raise RuntimeError('An existing session still owns the control socket')
            path.unlink()
        server.bind(str(path))
        bound = True
        server.listen(4)
        for sig in (signal.SIGINT, signal.SIGTERM):
            handlers[sig] = signal.signal(sig, stop_signal)
        pending = list(commands)
        state = 'preflight' if preflight else 'running'
        if preflight:
            children.append(subprocess.Popen(preflight, start_new_session=True))
        print(f'{mode}: {state}; use go2-session.sh stop {mode} or Ctrl+C', flush=True)
        while not stopping:
            if state == 'running' and pending:
                children.extend(subprocess.Popen(command, start_new_session=True) for command in pending)
                pending = []
            exited = [(child, os.waitid(os.P_PID, child.pid, os.WEXITED | os.WNOHANG | os.WNOWAIT))
                      for child in children]
            finished = [(child, info) for child, info in exited if info is not None]
            if finished:
                child, info = finished[0]
                if state == 'preflight' and info.si_code == os.CLD_EXITED and info.si_status == 0:
                    child.wait()
                    children.remove(child)
                    state = 'running'
                    continue
                print(f'{mode}: child exited; stopping owned session', flush=True)
                return 1
            readable, _, _ = select.select([server], [], [], .1)
            if readable:
                client, _ = server.accept()
                client.settimeout(.5)
                try:
                    action = client.recv(64).decode()
                    if action == 'stop':
                        stop_client = client
                        stopping = True
                    else:
                        client.sendall(json.dumps({'mode': mode, 'state': state,
                                                   'pid': os.getpid(), 'children': [p.pid for p in children]}).encode())
                except (OSError, UnicodeError):
                    pass
                finally:
                    if client is not stop_client:
                        client.close()
        return 0
    finally:
        stop_children(children)
        if stop_client:
            try:
                stop_client.sendall(json.dumps({'mode': mode, 'state': 'stopped'}).encode())
            except OSError:
                pass
            stop_client.close()
        server.close()
        if bound:
            path.unlink(missing_ok=True)
        for fd in locks:
            os.close(fd)
        for sig, handler in handlers.items():
            signal.signal(sig, handler)


def session_commands(mode, map_path):
    scripts = Path(__file__).resolve().parent
    odom = os.environ.get('GO2_ODOM_SOURCE', 'utlidar')
    if odom != 'utlidar':
        raise ValueError('Managed sessions require GO2_ODOM_SOURCE=utlidar (shared sensor clock)')
    commands = []
    if mode == 'mapping':
        commands.append(['ros2', 'launch', 'go2_nav2', 'slam_mapping.launch.py', 'odom_source:=utlidar'])
    elif mode == 'navigation':
        from go2_nav2.map_bundle import validate_map
        map_path = validate_map(map_path)
        commands.append(['ros2', 'launch', 'go2_nav2', 'nav2_slam_loc.launch.py',
                         f'map:={map_path}', 'odom_source:=utlidar'])
    elif mode == 'teleop':
        commands.append(['ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard'])
    if mode != 'mapping' and os.environ.get('GO2_NET', 'wifi') == 'wifi':
        host = os.environ.get('GO2_ROBOT_IP')
        if not host:
            raise ValueError('Set GO2_ROBOT_IP before starting a motion session')
        commands.append([sys.executable, str(scripts / 'go2_cmd_vel_tcp.py'), '--role', 'client',
                         '--host', host, '--port', os.environ.get('GO2_CMD_TCP_PORT', '17999')])
    if not commands:
        raise ValueError('transport mode is only available over Wi-Fi')
    return commands


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('start', 'stop', 'status'))
    parser.add_argument('mode', nargs='?', choices=MODES)
    parser.add_argument('--map', default='/ws/maps/my_room.yaml')
    args = parser.parse_args()
    try:
        root = runtime_dir()
        if args.action == 'status':
            print(json.dumps([request(root, mode, 'status') for mode in ([args.mode] if args.mode else MODES)], indent=2))
            return 0
        if not args.mode:
            parser.error('start/stop require a mode')
        if args.action == 'stop':
            print(json.dumps(request(root, args.mode, 'stop'), indent=2))
            return 0
        commands = session_commands(args.mode, args.map)
        preflight = [sys.executable, str(Path(__file__).with_name('session_preflight.py')), args.mode]
        return supervise(root, args.mode, commands, preflight)
    except (OSError, ValueError, RuntimeError) as exc:
        print(f'ERROR: {exc}. No unrelated processes were stopped.', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
