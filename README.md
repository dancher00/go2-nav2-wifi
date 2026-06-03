# go2-nav2-wifi — Unitree Go2 Edu · SLAM + Nav2 over Wi‑Fi

[![CI](https://github.com/dancher00/go2-nav2-wifi/actions/workflows/ci.yml/badge.svg)](https://github.com/dancher00/go2-nav2-wifi/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/github/license/dancher00/go2-nav2-wifi)](LICENSE)
[![Release](https://img.shields.io/github/v/release/dancher00/go2-nav2-wifi)](https://github.com/dancher00/go2-nav2-wifi/releases)

**SLAM mapping and Nav2 on a laptop over Wi‑Fi** — built-in Unitree lidar, no external sensors, Docker ROS 2 Humble, no Ethernet cable, no WebRTC, no CycloneDDS patch on the robot.

## How this compares

| | **go2-nav2-wifi** (this repo) | [go2_ros2_sdk](https://github.com/abizovnuralem/go2_ros2_sdk) | [go2_ros2_toolbox](https://github.com/andy-zhuo-02/go2_ros2_toolbox) |
|---|-------------------------------|---------------------------------------------------------------|----------------------------------------------------------------------|
| **Robot** | Go2 **Edu** (stock **utlidar**) | Go2 AIR / PRO / **Edu** | Go2 **Edu** + expansion dock |
| **Extra sensors** | None | Often external LiDAR / depth (config-dependent) | Dock LiDAR stack |
| **Wi‑Fi to laptop** | Jetson **TCP topic relay** | **WebRTC** | Usually not the focus (onboard Foxy) |
| **Robot DDS patch** | Not required | CycloneDDS setup for Ethernet; WebRTC for Wi‑Fi | On dock / official stack |
| **Unitree mobile app** | Can stay connected | Close app when using WebRTC | N/A |
| **SLAM / nav** | slam_toolbox + Nav2 (Smac + DWB) | slam_toolbox + Nav2 | slam_toolbox + Nav2 |
| **Where Nav2 runs** | **Laptop** (Docker Humble) | Laptop (typical) | **On robot** (dock) |
| **Scope** | Wi‑Fi/Ethernet Nav2 recipe + relay scripts | Full ROS2 SDK (teleop, Foxglove, detection, multi-robot, …) | Dock-native toolbox |

Robot (left) + RViz (right) in the demos below.

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
| **Odometry** | `utlidar` or **sport** via `GO2_ODOM_SOURCE` — same for map and nav |
| **Wi‑Fi** | Jetson **topic relay** + TCP `/cmd_vel` (no WebRTC, no CycloneDDS patch) |
| **Laptop** | **Docker** ROS 2 **Humble** · robot onboard **Foxy** |

---

## Quick start (Wi‑Fi)

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

---

## Requirements

- Unitree **Go2 Edu** (onboard **utlidar** — stock sensor, nothing to mount)
- Laptop: Ubuntu + Docker, **same Wi‑Fi as the robot** (router or hotspot; laptop USB Wi‑Fi dongle works)
- Optional: Ethernet `192.168.123.x` instead of Wi‑Fi

---

## License

MIT · https://github.com/dancher00/go2-nav2-wifi
