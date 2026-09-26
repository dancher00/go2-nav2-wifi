# Handheld mapping over Wi-Fi

Use this workflow when moving Go2 with its handheld remote. It does not start
laptop teleop, Nav2 goals, a TCP motion server or a sport bridge. Complete the
[one-time setup](RELAY-WIFI.md) first. Use `GO2_ODOM_SOURCE=utlidar`.

The standard Compose container is `go2-humble`. If using the existing dedicated
test container, replace that name with `go2-nav2-live-test` in every command.
IP addresses below are examples: use the current Wi-Fi addresses of your robot
and laptop, also configured in `docker/.env` / the container environment.

## One command (recommended)

From the laptop repository, with the project container already running:

```bash
./mapping.sh
```

This opens RViz and starts the sensor-only relay and SLAM. It uses the IPs in
the container environment and asks for the SSH password if needed (never stored
in the script). Close RViz or press Ctrl+C in this terminal to stop its processes.
Save a map before closing; it is not automatically saved. A lying-down run is
only a sensor test: stand with the remote and restart for a useful walking map.

The launcher selects `go2-nav2-live-test` if running, otherwise `go2-humble`.
Override with `GO2_CONTAINER=... ./mapping.sh`. Logs are in `ws/log/mapping.*`.
An existing relay/session is not taken over or stopped. The remote deployed
`robot-relay-wifi.sh` should include the current HUP cleanup handler.

## Three terminals (manual alternative)

**1. Robot sensor relay — from the laptop:**

```bash
ssh -t unitree@192.168.1.58 \
  'GO2_HOST_IP=192.168.1.90 GO2_RELAY_CAMERA=0 bash ~/robot-relay-wifi.sh --sensors-only'
```

Wait for `SENSOR-ONLY` and `Relay running`. Keep this SSH terminal open. If it
reports `Relay already running`, locate the original terminal instead of
starting or killing additional relay instances.

**2. SLAM — on the laptop, after standing the robot with the remote:**

```bash
docker exec -it go2-humble bash /ws/scripts/go2-session.sh start mapping
```

Keep this terminal open and the robot still during the initial calibration.
Wait for `Shared sensor offset locked` and the map in RViz, then walk slowly.
Start only one mapping or localization launch at a time.

**3. RViz — on the laptop:**

The standard container runs as root. Allow that local X11 user:

```bash
xhost +SI:localuser:root
docker exec -it \
  -e DISPLAY="$DISPLAY" -e QT_QPA_PLATFORM=xcb \
  -e QT_X11_NO_MITSHM=1 -e LIBGL_ALWAYS_SOFTWARE=1 \
  go2-humble bash -c \
  'source /ws/scripts/setup-robot-wifi.sh &&
   ros2 run rviz2 rviz2 -d /ws/src/go2_nav2/rviz/slam.rviz'
```

For a container running with your host UID (the dedicated test container), use
`xhost +SI:localuser:$(id -un)` instead. The mapping preset fixes both the world
and view target to `map`; camera display is off by default. Before SLAM publishes
the first map transform, RViz may temporarily report that `map` is unavailable.
`Stereo is NOT SUPPORTED` does not prevent ordinary 2D viewing. A config-directory
error concerns saving RViz preferences, not SLAM; use the explicit `-d` preset.

## Check, save, restart

In another laptop terminal, the following only observes data:

```bash
docker exec go2-humble bash -c \
  'source /ws/scripts/setup-robot-wifi.sh &&
   python3 /ws/scripts/check-sensor-time.py --duration 10'
```

Expect `passed: true`; see [the diagnostic limits](SENSOR-TIMING.md).

Save before restarting if the current map is worth keeping. Choose a new name
to avoid overwriting an existing map:

```bash
docker exec go2-humble bash -c \
  'source /ws/scripts/setup-robot-wifi.sh &&
   /ws/scripts/save-map.sh room_run_01'
```

The saver refuses an existing name and publishes `.yaml` only after `.pgm`,
`.posegraph` and `.data` are complete. A failed save does not replace an old map.
For later localization, keep all four files together under `ws/maps/`.

Status and scoped shutdown work from another terminal, even if Wi-Fi is down:

```bash
docker exec go2-humble bash /ws/scripts/go2-session.sh status
docker exec go2-humble bash /ws/scripts/go2-session.sh stop mapping
```

See [session ownership and limitations](SESSIONS.md). `not-managed` does not
mean no ROS processes exist: a legacy direct launch still belongs to its own terminal.

| What stopped | What to restart |
| --- | --- |
| Only RViz was closed | Only terminal 3; the SLAM map remains in memory |
| A fresh map is needed | Ctrl+C in terminal 2, wait for launch to finish, run terminal 2 again |
| Robot relay or robot restarted | Start terminal 1 first, then restart terminal 2; RViz can stay open |
| Robot was mapping while lying down | Stand with the remote, then start a fresh SLAM map |
| `SENSOR CLOCK FAULT` appeared | Keep robot stationary, verify sensor/clock health, restart the entire mapping launch |

Keep SLAM in its foreground terminal so Ctrl+C owns the shutdown. Do not merely
close a terminal tab and assume all remote/container processes exited. Do not
use blanket `pkill` commands: other sessions may own matching processes.

At the end, stop robot motion with the remote, save if needed, then Ctrl+C in
the SLAM, RViz and relay terminals. These commands do not disconnect USB cameras
or stop unrelated factory processes on the robot.

### Эксперимент 3D LiDAR

В ветке `experiment/lidar-3d-slam`: `./mapping.sh --3d` запускает отдельный
Point-LIO на Jetson (raw L1 + гироскоп, без камеры, ускорений и loop closure);
на ноутбуке — только RViz через Wi-Fi. `--3d --laptop` сохраняет вычисления на ноутбуке.
Установка, измерения и ограничения: [3D LiDAR SLAM](LIDAR-3D-SLAM.md).
Обычный `./mapping.sh` сохраняет 2D-профиль.
