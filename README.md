# go2-nav2-wifi — Unitree Go2 Edu · SLAM + Nav2 over Wi‑Fi

[![CI](https://github.com/dancher00/go2-nav2-wifi/actions/workflows/ci.yml/badge.svg)](https://github.com/dancher00/go2-nav2-wifi/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/github/license/dancher00/go2-nav2-wifi)](LICENSE)
[![Release](https://img.shields.io/github/v/release/dancher00/go2-nav2-wifi)](https://github.com/dancher00/go2-nav2-wifi/releases)

**SLAM mapping and Nav2 on a laptop over Wi‑Fi** — built-in Unitree lidar, no external sensors, Docker ROS 2 Humble, no Ethernet cable, no WebRTC, no CycloneDDS patch on the robot.

**3D experiment branch:** `./mapping.sh --3d` computes Point-LIO on the Jetson;
the laptop runs RViz. `./mapping.sh --3d --laptop` keeps the raw-sensor Wi-Fi
experiment. The 2D workflow below is preserved. See [3D setup and results](docs/LIDAR-3D-SLAM.md).

Robot + RViz in the demos below.

### Nav A → B

https://github.com/user-attachments/assets/44ae54a9-09f1-490c-ab3b-6291595e3324

### LiDAR + RViz

https://github.com/user-attachments/assets/2c817478-9fc5-4000-8211-b8b47e07eafb

### SLAM mapping

https://github.com/user-attachments/assets/16ffa9da-6469-4384-a56e-00d0343bb375

---

## What this is

- **Built-in utlidar** on Go2 Edu (`/utlidar/cloud_deskewed`) — no external LiDAR, RealSense, or RPLidar
- **Laptop over Wi‑Fi** (USB dongle is fine) — robot and laptop on the same network; set `GO2_HOST_IP` / `GO2_ROBOT_IP` in `.env`
- **Wi‑Fi relay** on the Jetson — lidar, odom, IMU, legs, camera, `/cmd_vel` over TCP
- **Docker** on the laptop — reproducible Humble + Nav2 + slam_toolbox
- **Workflow:** map the room → save map → **2D Pose Estimate** → **Goal Pose** → robot drives

Tested on **Go2 Edu** (Unitree onboard ROS 2 Foxy + laptop Docker Humble).

---

## Stack

| Layer | Choice |
|-------|--------|
| **LiDAR** | Built-in Unitree **utlidar** → `pointcloud_to_laserscan` → `/scan` |
| **Mapping** | [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) (async mode) |
| **Localization** | slam_toolbox on saved map + RViz **2D Pose Estimate** (not AMCL) |
| **Navigation** | [Nav2](https://navigation.ros.org/) — **SmacPlanner2D** + **DWB** controller |
| **Odometry** | `utlidar`, shared acquisition clock; legacy `sport` is outside the managed workflow |
| **Wi‑Fi** | Jetson **topic relay** + TCP `/cmd_vel` (no WebRTC, no CycloneDDS patch) |
| **Laptop** | **Docker** ROS 2 **Humble** · robot onboard **Foxy** |

---

## Quick start (Wi‑Fi)

**Mapping with the handheld remote:** [start/stop/restart guide](docs/MAPPING-SESSION.md)
uses only the sensor relay, SLAM and RViz; no laptop motion commands.

After the one-time setup, run from the laptop repository:

```bash
./mapping.sh
```

It starts the sensor-only relay, SLAM and RViz. Close RViz or press Ctrl+C to
stop its processes. Robot IPs come from the running container; SSH may ask for
the robot password. Stand with the remote before starting a map for navigation.

[Managed sessions](docs/SESSIONS.md) provide scoped start/stop/status,
duplicate-start refusal and map-bundle checks. Navigation keeps its direct command path.
D435i work is reserved for `experiment/d435i-visual-slam`, not `main`.

Full setup: **[docs/RELAY-WIFI.md](docs/RELAY-WIFI.md)** · Mapping & nav: **[docs/NAVIGATION.md](docs/NAVIGATION.md)**

```bash
git clone https://github.com/dancher00/go2-nav2-wifi.git
cd go2-nav2-wifi/docker
cp .env.example .env          # GO2_ROBOT_IP, GO2_HOST_IP, GO2_ODOM_SOURCE
docker compose build && docker compose up -d
docker compose exec go2 bash -c '/ws/scripts/build-unitree-msgs.sh && /ws/scripts/build.sh'

# from laptop repo root
./ws/scripts/deploy-robot-wifi.sh unitree@ROBOT_IP

# on robot (once)
bash ~/robot-setup-wifi-robot.sh

# each session — on robot
export GO2_HOST_IP=YOUR_LAPTOP_IP && bash ~/robot-relay-wifi.sh
```

Then on the laptop (Docker shell): `slam_mapping` + teleop → `save-map.sh` → `nav-to-point.sh` — see [NAVIGATION.md](docs/NAVIGATION.md).

**Do not** run `sport_bridge.launch.py` on the laptop over Wi‑Fi (relay handles cmd_vel on the robot).

For a robot lying down, use `bash ~/robot-relay-wifi.sh --sensors-only` instead:
normal relay mode starts motion control and may stand it up. Wi-Fi now uses DDS
domain 64 on the laptop and relay publisher; onboard DDS and Ethernet stay on 0.
See [stationary tests and upgrade notes](docs/RELAY-WIFI.md#stationary-sensor-only-test).

---

## Quick start (Ethernet)

```bash
/ws/scripts/build-unitree-msgs.sh
/ws/scripts/build.sh
source /ws/scripts/setup-robot-eth.sh
ros2 launch go2_nav2 sport_bridge.launch.py
```

---

## Docs

| Doc | Content |
|-----|---------|
| [RELAY-WIFI.md](docs/RELAY-WIFI.md) | One-time Wi‑Fi install, relay, deploy |
| [NAVIGATION.md](docs/NAVIGATION.md) | Mapping, save map, nav to goal, troubleshooting |
| [MAPPING-SESSION.md](docs/MAPPING-SESSION.md) | Handheld mapping: terminal roles, restart and checks |
| [SESSIONS.md](docs/SESSIONS.md) | Owned processes, startup checks and map validation |
| [SENSOR-TIMING.md](docs/SENSOR-TIMING.md) | Shared acquisition clock and stationary verification |
| [ROADMAP.md](docs/ROADMAP.md) | Current-stack acceptance criteria, then optional D435i visual SLAM |

## Regression tests

After building the Docker image, run on the laptop (also used by CI):

```bash
bash ws/scripts/test-regressions.sh
```

Tests run with no external network, a read-only repository and temporary ROS
logs. They do not replace supervised walking, navigation or stopping tests.

---

## Requirements

- Unitree **Go2 Edu** (onboard **utlidar** — stock sensor, nothing to mount)
- Laptop: Ubuntu + Docker, **same Wi‑Fi as the robot** (router or hotspot; laptop USB Wi‑Fi dongle works)
- Optional: Ethernet `192.168.123.x` instead of Wi‑Fi

---

## License

MIT · https://github.com/dancher00/go2-nav2-wifi

### Эксперимент 3D LiDAR

В ветке `experiment/lidar-3d-slam`: `./mapping.sh --3d` запускает отдельный
Point-LIO на Jetson (raw L1 + гироскоп, без камеры, ускорений и loop closure);
на ноутбуке — только RViz через Wi-Fi. `--3d --laptop` сохраняет вычисления на ноутбуке.
Установка, измерения и ограничения: [3D LiDAR SLAM](docs/LIDAR-3D-SLAM.md).
Обычный `./mapping.sh` сохраняет 2D-профиль.

Проверенный 3D Point-LIO baseline: [снимок сборки и запуск](docs/POINTLIO-BASELINE.md).
