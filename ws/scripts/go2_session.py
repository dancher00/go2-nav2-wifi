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
import tempfile
import time

MODES = ('mapping', 'lidar3d', 'lidar3d-viz', 'navigation', 'teleop', 'transport')
RESOURCES = {'mapping': ('stack',), 'lidar3d': ('stack',), 'lidar3d-viz': ('stack',), 'navigation': ('stack', 'motion'),
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


def supervise(root, mode, commands, preflight=None, owner=None, normal_exit_command=None):
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
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
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
                if child.args == normal_exit_command and info.si_code == os.CLD_EXITED and info.si_status == 0:
                    return 0
                return 1
            readable, _, _ = select.select([server], [], [], .1)
            if readable:
                client, _ = server.accept()
                client.settimeout(.5)
                try:
                    action = client.recv(64).decode()
                    if action == 'stop' or (owner is not None and action == 'stop:' + owner):
                        stop_client = client
                        stopping = True
                    elif action.startswith('stop:'):
                        client.sendall(json.dumps({'mode': mode, 'state': 'owner-mismatch'}).encode())
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


def session_commands(mode, map_path, output_dir=None, bag=None, config=None):
    scripts = Path(__file__).resolve().parent
    odom = os.environ.get('GO2_ODOM_SOURCE', 'utlidar')
    if odom != 'utlidar':
        raise ValueError('Managed sessions require GO2_ODOM_SOURCE=utlidar (shared sensor clock)')
    commands = []
    if mode == 'mapping':
        commands.append(['ros2', 'launch', 'go2_nav2', 'slam_mapping.launch.py', 'odom_source:=utlidar'])
    elif mode == 'lidar3d':
        if output_dir is None:
            raise ValueError('lidar3d requires a new output directory')
        backend_launch = 'legkilo_mapping.launch.py' if os.environ.get('GO2_LIDAR3D_BACKEND') == 'legkilo' else 'lidar3d_mapping.launch.py'
        commands.append(['ros2', 'launch', 'go2_nav2', backend_launch,
                         f'result_dir:={output_dir}'])
        if bag:
            commands[-1].append(f'bag:={bag}')
        if config:
            commands[-1].append(f'config:={config}')
        if not bag and os.environ.get('GO2_LIDAR3D_COMPUTE') == 'jetson':
            commands.append(['env', 'GO2_RELAY_PROFILE=lidar3d-map', 'GO2_RELAY_USE_HUMBLE=1',
                             'GO2_RELAY_CAMERA=0', 'bash', str(scripts / 'robot-relay-wifi.sh'), '--sensors-only'])
    elif mode == 'lidar3d-viz':
        commands.append(['ros2', 'launch', 'go2_nav2', 'lidar3d_robot_viz.launch.py'])
        rviz_file = 'legkilo.rviz' if os.environ.get('GO2_LIDAR3D_BACKEND') == 'legkilo' else 'lidar3d.rviz'
        commands.append(['ros2', 'run', 'rviz2', 'rviz2', '-d', '/ws/src/go2_nav2/rviz/' + rviz_file])
    elif mode == 'navigation':
        from go2_nav2.map_bundle import validate_map
        map_path = validate_map(map_path)
        commands.append(['ros2', 'launch', 'go2_nav2', 'nav2_slam_loc.launch.py',
                         f'map:={map_path}', 'odom_source:=utlidar'])
    elif mode == 'teleop':
        commands.append(['ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard'])
    if mode not in ('mapping', 'lidar3d', 'lidar3d-viz') and os.environ.get('GO2_NET', 'wifi') == 'wifi':
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
    parser.add_argument('--rviz', action='store_true', help='Open RViz in the owned mapping session')
    parser.add_argument('--owner', help='Scope automated stop to this launcher token')
    parser.add_argument('--output', help='New directory for a lidar3d PCD map and TUM trajectory')
    parser.add_argument('--bag', help='Replay recorded lidar3d inputs locally, headless, without robot relay')
    parser.add_argument('--config', help='Backend YAML override for lidar3d')
    args = parser.parse_args()
    try:
        root = runtime_dir()
        if args.action == 'status':
            print(json.dumps([request(root, mode, 'status') for mode in ([args.mode] if args.mode else MODES)], indent=2))
            return 0
        if not args.mode:
            parser.error('start/stop require a mode')
        if args.action == 'stop':
            action = 'stop:' + args.owner if args.owner else 'stop'
            print(json.dumps(request(root, args.mode, action), indent=2))
            return 0
        output_dir = None
        if (args.bag or args.config) and args.mode != 'lidar3d':
            raise ValueError('--bag/--config are only supported for lidar3d')
        if args.bag:
            if args.rviz:
                raise ValueError('Offline replay is headless; inspect its saved PCD and trajectory')
            if not (Path(args.bag) / 'metadata.yaml').is_file():
                raise ValueError('Expected a finalized sensors bag directory with metadata.yaml')
            # Independent local DDS domain, no robot peers. Existing stack ownership remains.
            os.environ['ROS_DOMAIN_ID'] = '65'
            # RMW's localhost-only flag adds lo a second time when XML names it.
            os.environ['ROS_LOCALHOST_ONLY'] = '0'
            os.environ['CYCLONEDDS_URI'] = '<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>true</AllowMulticast></General></Domain></CycloneDDS>'
        if args.output and args.mode != 'lidar3d':
            raise ValueError('--output is only supported for lidar3d')
        if args.mode == 'lidar3d':
            if args.output:
                output_dir = Path(args.output).resolve()
                output_dir.mkdir(parents=True, exist_ok=False)
            else:
                base = Path('/ws/maps/lidar3d')
                base.mkdir(parents=True, exist_ok=True)
                output_dir = Path(tempfile.mkdtemp(prefix=time.strftime('run-%Y%m%d-%H%M%S-'), dir=base))
            (output_dir / 'Log').mkdir()
            (output_dir / 'PCD').mkdir()
            print(f'3D result directory: {output_dir}', flush=True)
        commands = session_commands(args.mode, args.map, output_dir, args.bag, args.config)
        if args.rviz:
            if args.mode not in ('mapping', 'lidar3d'):
                raise ValueError('--rviz is supported for mapping and lidar3d only')
            if args.mode == 'lidar3d':
                commands.append(['ros2', 'launch', 'go2_nav2', 'lidar3d_robot_viz.launch.py'])
            preset = 'lidar3d' if args.mode == 'lidar3d' else 'slam'
            commands.append(['ros2', 'run', 'rviz2', 'rviz2', '-d', f'/ws/src/go2_nav2/rviz/{preset}.rviz'])
        preflight = [sys.executable, str(Path(__file__).with_name('session_preflight.py')), args.mode]
        if args.bag or args.mode == 'lidar3d-viz':
            preflight = None
        result = supervise(root, args.mode, commands, preflight, owner=args.owner,
                           normal_exit_command=commands[-1] if args.rviz or args.bag or args.mode == 'lidar3d-viz' else None)
        if output_dir is not None:
            saved = subprocess.run([sys.executable, str(Path(__file__).with_name('export-lidar3d.py')),
                                    str(output_dir)], check=False)
            result = result or saved.returncode
        return result
    except (OSError, ValueError, RuntimeError) as exc:
        print(f'ERROR: {exc}. No unrelated processes were stopped.', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
