# Wi‑Fi setup (Go2 Edu)

Jetson relay: **built-in utlidar**, odom, IMU, sport, leg joints, camera, teleop/nav over TCP. **No** external LiDAR. **No** `cyclonedds.xml` patch on the robot.

Laptop connects over **Wi‑Fi** (USB dongle on the laptop is fine) — robot and laptop must be on the **same subnet** (`GO2_HOST_IP`, `GO2_ROBOT_IP` in `.env`).

Mapping and Nav2: [NAVIGATION.md](NAVIGATION.md).

---

## One-time — laptop

```bash
cd go2-nav2-wifi/docker
cp .env.example .env
```

In `.env`:

```bash
GO2_NET=wifi
GO2_RELAY_DOMAIN_ID=64       # same on robot relay and laptop; never 0
GO2_ROBOT_IP=192.168.1.58    # Jetson IP on Wi‑Fi
GO2_HOST_IP=192.168.1.90     # laptop IP on Wi‑Fi
GO2_ODOM_SOURCE=sport        # or utlidar — same for map and nav
# GO2_CMD_VEL_HZ=50          # optional
```

```bash
docker compose build && docker compose up -d
docker compose exec go2 bash -c '/ws/scripts/build-unitree-msgs.sh && /ws/scripts/build.sh'
xhost +local:docker   # RViz on host
```

---

## One-time — robot

**From laptop:**

```bash
cd go2-nav2-wifi
./ws/scripts/deploy-robot-wifi.sh unitree@192.168.1.58
```

**On robot (SSH):**

```bash
bash ~/robot-setup-wifi-robot.sh
```

Installs: `unitree_ros2` msgs → `~/unitree_sdk2` (if missing) → `~/bin/go2_camera_jpeg_cli` → `python3-opencv`.  
~10–20 min.

**Verify relay** (expect **6 topics**, `image_raw` > 0):

```bash
export GO2_HOST_IP=192.168.1.90
bash ~/robot-relay-wifi.sh
```

Log should show `Relay running: onboard domain 0 -> Wi-Fi domain 64`, topic rates
in Hz, and `first frame published` when a camera backend is installed.

---

## Each session

| Where | Command |
|-------|---------|
| **Robot** | `export GO2_HOST_IP=192.168.1.90 && bash ~/robot-relay-wifi.sh` |
| **Laptop** | `cd go2-nav2-wifi/docker && ./shell.sh` |
| **Check** | `bash /ws/scripts/check-wifi-dds.sh` |

Then [NAVIGATION.md](NAVIGATION.md): mapping (`slam_mapping` + `teleop-slam.sh`), `save-map.sh`, `nav-to-point.sh`.

**Do not** run `sport_bridge.launch.py` on the laptop over Wi‑Fi.

### Stationary sensor-only test

The normal launcher starts a sport bridge that can stand the robot up. When the
robot is lying down, use this mode instead (and stop any previously started
motion bridges first):

```bash
# Robot: no TCP command server, sport bridge, or stand command is started.
GO2_HOST_IP=YOUR_LAPTOP_IP GO2_RELAY_CAMERA=0 bash ~/robot-relay-wifi.sh --sensors-only
# Laptop Docker: refresh the environment, then inspect sensors / mapping only.
source /ws/scripts/setup-robot-wifi.sh
python3 /ws/scripts/check-relay-stream.py --duration 10
ros2 launch go2_nav2 slam_mapping.launch.py odom_source:=utlidar
```

Do not start teleop or send goals during this check. A map made while lying down
is only a pipeline smoke test, not a usable navigation map.

### DDS domain separation

The robot's internal subscriber, camera and motion bridges use domain **0**.
Only the Wi-Fi relay publisher and laptop use **64**. Both sides must set the
same `GO2_RELAY_DOMAIN_ID` if changed (supported configuration range: 1–101).
Ethernet setup still uses domain 0. No factory DDS files are changed.

Sharing domain 0 across both relay sides creates a feedback loop: a high reported
message rate can actually be repeated old frames. `check-relay-stream.py` counts
unique original timestamps as well as arrivals. Update both robot and laptop,
restart their ROS processes and re-source the network setup when upgrading.
The supervisor rejects a second instance on the same socket and stops its own
children on exit or child failure.

---

## Relay topics

| Topic | Purpose |
|-------|---------|
| `/utlidar/cloud_deskewed` | SLAM / RViz |
| `/utlidar/robot_odom` | odom (`GO2_ODOM_SOURCE=utlidar`) |
| `/utlidar/imu` | IMU |
| `/sportmodestate` | sport odom |
| `/lf/lowstate` | leg joints in RViz |
| `/go2_front_camera/image_raw` | camera mono8 160×120 |

`/cmd_vel`: laptop → robot over **TCP :17999** (`teleop-slam.sh` / `nav-to-point.sh` start the client).

The laptop sends zero velocity after **0.5 seconds without a new `/cmd_vel`**.
Set `GO2_CMD_SOURCE_TIMEOUT` to a positive, finite number of seconds in `docker/.env`
and recreate the container with `docker compose up -d` to change it. TCP heartbeats
continue while stopped; new source commands resume motion. The client checks command
age again after reconnecting and attempts to send zero on orderly shutdown.

Keyboard teleop publishes on keypresses: hold the movement key for repeated commands;
releasing it lets the source timeout stop motion. If the keyboard's initial repeat
delay exceeds the timeout, motion may briefly pause before repeating. Nav2 publishes
continuously while driving. The robot's `GO2_CMD_TIMEOUT` remains a separate watchdog
for loss of the TCP stream; the launcher now defaults it to **0.5 seconds**, not
8 seconds. Do not increase watchdogs to mask transport or planner failures.

---

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| `unitree_go missing` | `bash ~/robot-build-unitree-msgs.sh`, restart relay |
| No utlidar on laptop | `bash ~/robot-check-topics.sh` on robot; is relay running? |
| Teleop does not move robot | `bash ~/robot-check-cmdvel.sh`; only one TCP client on laptop |
| No camera | `bash ~/robot-build-camera-cli.sh` or `robot-install-camera-sdk.sh` |
| Scan shifted vs walls | same `GO2_ODOM_SOURCE` for map and nav + **2D Pose Estimate** |

Ethernet: [NAVIGATION.md](NAVIGATION.md).
