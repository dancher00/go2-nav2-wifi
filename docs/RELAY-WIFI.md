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

Log should show: `Sport bridge on robot`, `first frame published`, `relay-pub] ready (6 topics)`.

---

## Each session

| Where | Command |
|-------|---------|
| **Robot** | `export GO2_HOST_IP=192.168.1.90 && bash ~/robot-relay-wifi.sh` |
| **Laptop** | `cd go2-nav2-wifi/docker && ./shell.sh` |
| **Check** | `bash /ws/scripts/check-wifi-dds.sh` |

Then [NAVIGATION.md](NAVIGATION.md): mapping (`slam_mapping` + `teleop-slam.sh`), `save-map.sh`, `nav-to-point.sh`.

**Do not** run `sport_bridge.launch.py` on the laptop over Wi‑Fi.

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
for loss of the TCP stream.

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
