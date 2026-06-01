# go2-nav2-wifi — Unitree Go2 Edu · SLAM + Nav2 over Wi‑Fi

**SLAM mapping and Nav2 point-to-point navigation on a laptop over Wi‑Fi** — Docker ROS 2 Humble, no Ethernet cable, no WebRTC, no CycloneDDS patch on the robot.

Robot (left) + RViz (right) in the demos below.

### Nav A → B

https://github.com/user-attachments/assets/44ae54a9-09f1-490c-ab3b-6291595e3324

### LiDAR + RViz

https://github.com/user-attachments/assets/2c817478-9fc5-4000-8211-b8b47e07eafb

### SLAM mapping

https://github.com/user-attachments/assets/16ffa9da-6469-4384-a56e-00d0343bb375

---

## What this is

- **Wi‑Fi relay** on the Go2 Jetson — lidar, odom, IMU, legs, camera, `/cmd_vel` over TCP
- **Docker** on the laptop — reproducible Humble + Nav2 + slam_toolbox
- **Workflow:** map the room → save map → **2D Pose Estimate** → **Goal Pose** → robot drives

Tested on **Go2 Edu** (Unitree onboard ROS 2 Foxy + laptop Docker Humble).

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

- Unitree **Go2 Edu**
- Laptop: Ubuntu + Docker, same Wi‑Fi as robot
- Optional: Ethernet `192.168.123.x` instead of Wi‑Fi

---

## License

MIT · https://github.com/dancher00/go2-nav2-wifi
