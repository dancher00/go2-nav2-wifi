# Go2 — mapping and navigation

**Wi‑Fi (relay setup, once):** [RELAY-WIFI.md](RELAY-WIFI.md)  
**Map:** `slam_mapping.launch.py` · **Nav:** `/ws/scripts/nav-to-point.sh` + RViz **Goal Pose**

---

## One-time (laptop, Docker)

If you have not done [RELAY-WIFI.md](RELAY-WIFI.md) yet:

```bash
cd go2-nav2-wifi/docker
cp .env.example .env    # GO2_ROBOT_IP, GO2_HOST_IP, GO2_ODOM_SOURCE
docker compose build && docker compose up -d
docker compose exec go2 bash -c '/ws/scripts/build-unitree-msgs.sh && /ws/scripts/build.sh'
```

Ethernet instead of Wi‑Fi: set `GO2_NET=eth` in `.env`, see Ethernet sections below.

**Every Docker terminal** loads env automatically (`go2-env.sh` in `.bashrc`):

```bash
cd go2-nav2-wifi/docker && ./shell.sh
# or: docker compose exec go2 bash
```

Manual reload: `source /ws/scripts/go2-env.sh`.  
Odometry: `GO2_ODOM_SOURCE=utlidar` (default) or `sport` — **use the same value** for mapping and nav (in `docker/.env`).

---

## T0 — host (Ethernet cable)

```bash
cd go2-nav2-wifi/docker
./setup-host-eth.sh
ping -c2 192.168.123.18
docker compose up -d
xhost +local:docker
```

Wi‑Fi: [RELAY-WIFI.md](RELAY-WIFI.md) — deploy + `robot-relay-wifi.sh` on the robot.

---

## RViz preview (model + rainbow lidar cloud)

Like [go2_robot](https://github.com/Unitree-Go2-Robot/go2_robot): RobotModel + PointCloud2 colored by intensity.

**Wi‑Fi:** relay on robot ([RELAY-WIFI.md](RELAY-WIFI.md)). **Ethernet:** cable + `sport_bridge` on laptop.

**Host:** `xhost +local:docker` · **Docker:** `./shell.sh`

**Wi‑Fi**

| T | Command |
|---|---------|
| **R** | on robot: `bash ~/robot-relay-wifi.sh` |
| **1** | `ros2 launch go2_nav2 bringup_viz.launch.py` |

**Ethernet**

| T | Command |
|---|---------|
| **1** | `ros2 launch go2_nav2 sport_bridge.launch.py` |
| **2** | `ros2 launch go2_nav2 bringup_viz.launch.py` |

Fixed Frame: **odom**. Relay + `/lf/lowstate` → legs in RViz. Built-in camera (mono8, low res): [RELAY-WIFI.md](RELAY-WIFI.md).

Orbit with the mouse to adjust the view.

Check in container: `ros2 topic hz /utlidar/cloud_deskewed` (should be > 0).

---

## Build a map

In each terminal: `cd go2-nav2-wifi/docker && ./shell.sh`

**Wi‑Fi** (relay on robot — teleop goes through it, **not** `sport_bridge` on laptop):

| T | Command |
|---|---------|
| **R** | on robot: `export GO2_HOST_IP=192.168.1.90 && bash ~/robot-relay-wifi.sh` |
| **1** | `ros2 launch go2_nav2 slam_mapping.launch.py` |
| **2** | `ros2 run rviz2 rviz2 -d $(ros2 pkg prefix go2_nav2)/share/go2_nav2/rviz/slam.rviz` |
| **3** | `/ws/scripts/teleop-slam.sh` (speed: **q/z w/x e/c** in teleop) |

**Ethernet** (cable `192.168.123.x`):

| T | Command |
|---|---------|
| **1** | `ros2 launch go2_nav2 sport_bridge.launch.py` |
| **2** | `ros2 launch go2_nav2 slam_mapping.launch.py` |
| **3** | `ros2 run rviz2 rviz2 -d $(ros2 pkg prefix go2_nav2)/share/go2_nav2/rviz/slam.rviz` |
| **4** | `/ws/scripts/teleop-slam.sh` |

Drive around the room. Check: `/ws/scripts/check-slam.sh` — `/scan` and `/map` must be present.

**Camera while mapping:** relay starts the bridge (`GO2_RELAY_CAMERA=1` by default) → `/go2_front_camera/image_raw`. Enable **Go2FrontCamera** in RViz. Check: `ros2 topic hz /go2_front_camera/image_raw`. If empty — on robot `bash ~/robot-build-camera-cli.sh`, restart relay; see [RELAY-WIFI.md](RELAY-WIFI.md).

**Odom from sport state:** in `docker/.env` set `GO2_ODOM_SOURCE=sport`, restart `./shell.sh`. Mapping:

```bash
ros2 launch go2_nav2 slam_mapping.launch.py odom_source:=${GO2_ODOM_SOURCE}
```

Default `utlidar` uses `/utlidar/robot_odom`. `sport` uses `/sportmodestate` → `/sport_state/odom` (relay forwards `/sportmodestate`).

**Teleop does not move robot over Wi‑Fi:** update `robot_relay_wifi.py`, `robot-relay-wifi.sh`, `robot_sport_bridge.py` on the robot (via `deploy-robot-wifi.sh`), restart relay; run `bash ~/robot-build-unitree-msgs.sh` once if `unitree_api` is missing.

**Jittering in place, `robot-check-cmdvel.sh` → NO /cmd_vel:** use `/ws/scripts/teleop-slam.sh` on the laptop (TCP client inside). Put the handheld remote aside.

**No map, log shows `pointcloud_to_laserscan` / `transform cache`:** over Wi‑Fi cloud timestamps may not match TF (`go2_cloud_stamp_sync` in `slam_mapping` re-stamps the cloud; rebuild `go2_nav2`).

Save map (while T1 `slam_mapping` is running):

```bash
/ws/scripts/save-map.sh my_room
ls /ws/maps/my_room.yaml /ws/maps/my_room.pgm /ws/maps/my_room.posegraph
```

Ctrl+C in mapping terminals when done.

---

## Navigate to a point

Load the saved map, set **where you are** once, then **where to go** — single goal, no waypoint file.

| RViz tool | Purpose |
|-----------|---------|
| **2D Pose Estimate** | where the robot **is now** (once per session) |
| **2D Goal Pose** | **where to drive** (click on map + heading arrow) |

**Wi‑Fi** (same as mapping — relay on robot, **no** `sport_bridge` on laptop):

| T | Command |
|---|---------|
| **R** | on robot: `export GO2_HOST_IP=192.168.1.90 && bash ~/robot-relay-wifi.sh` |
| **1** | `/ws/scripts/nav-to-point.sh /ws/maps/my_room.yaml` (reads `GO2_ODOM_SOURCE` from `.env`) |
| **2** | `ros2 run rviz2 rviz2 -d $(ros2 pkg prefix go2_nav2)/share/go2_nav2/rviz/nav.rviz` |

If the map was built with `sport` → set `GO2_ODOM_SOURCE=sport` in `docker/.env`, or scan will not align with walls.

**Ethernet:** T1 `sport_bridge.launch.py`, T2 `/ws/scripts/nav-to-point.sh`, T3 RViz.

**RViz order (T2):**
1. Wait **~12 s** after T1 starts. Check: `ros2 lifecycle get /map_server` → `active [3]`.
2. **2D Pose Estimate — once per session** (robot position + body heading). Log: `SLAM pose received`. Scan should match walls. Repeat only if scan drifts or log shows `/pose stale`.
3. **2D Goal Pose** — target. Blue line `/nav_plan`. Robot does **not** spin at the goal (final yaw ignored).

**Why it “does not move” without Pose Estimate:** slam_toolbox does not know your pose on the saved map until you set it. This is normal localization behavior (not GPS).

**Second goal does not run after the first:** wait ~2 s between goals (queue in `go2_goal_pose_nav`).

**Empty RViz / `unconnected trees`:** need `map→odom→base_link`. Without `/utlidar/robot_odom` from the robot there is no `odom→base_link` — start `bash ~/robot-relay-wifi.sh`. Check: `/ws/scripts/check-tf-nav.sh`.

**`/pose stale`:** until SLAM publishes `/pose`, `map→odom` is **extrapolated from odometry** and does not freeze.

**Spins at goal:** final yaw rotation is disabled in config; restart `nav-to-point.sh` after updating the repo.

**Goal behind — no turn-in-place:** **pre-rotate** (~60°+ toward goal) runs before planning. Log: `Pre-rotate toward goal`. Or set an intermediate goal to the side.

**Goal from terminal** (T1 running, Pose Estimate done; `yaw` in radians):

```bash
ros2 run go2_nav2 go2_simple_navigator --ros-args -p pose:="1.5 0.5 0.0"
```

---

## Optional: patrol loop

Multi-point route: `record-waypoints.sh` → `patrol.launch.py`. Example: `go2_nav2/config/patrol_example.yaml`. For a single A→B goal, the section above is enough.

---

## Navigation speed

Main knobs in `go2_nav2/config/go2_nav2_minimal.yaml`:

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `FollowPath.max_vel_x` | 0.45 | forward speed (m/s) |
| `general_goal_checker.xy_goal_tolerance` | 0.40 | “arrived” radius (m) |
| `general_goal_checker.yaw_goal_tolerance` | 3.14 | do not rotate at goal |

Teleop **q/z** sets speed on the laptop; speed clamp on the robot is **off** by default. Optional: `export GO2_MAX_LINEAR=0.5` before `robot-relay-wifi.sh`. Unitree firmware may limit speed itself.

---

## Checks

```bash
ros2 topic hz /utlidar/cloud_deskewed
ros2 topic hz /scan
/ws/scripts/check-tf-odom.sh
ros2 node list | grep sport_bridge   # exactly one /sport_bridge
```

Duplicates (motor grinding, cmd_vel conflict):

```bash
pkill -f sport_bridge
pkill -f nav2_slam_loc
pkill -f go2_odom_tf
ros2 daemon stop && sleep 1 && ros2 daemon start
# then nav-to-point.sh again (+ sport_bridge on Ethernet)
```

---

## Common issues

| Symptom | Fix |
|---------|-----|
| No topics | Wi‑Fi: relay on robot + `check-wifi-dds.sh`; cable: `setup-robot-eth.sh`; `ROS_DOMAIN_ID=0` |
| No `.posegraph` | run `save-map.sh` while `slam_mapping` is active |
| `failed to create plan` | goal in free space, closer; scan aligned with walls |
| Robot in RViz wrong vs floor | **2D Pose Estimate** on `/initialpose` |
| Goal Pose silent / Nav2 not ready | log `planner_server FATAL` / `NavfnPlanner does not exist` — `sudo apt install ros-humble-nav2-navfn-planner ros-humble-nav2-dwb-controller` |
| `Planner rejected goal` / `controller_server Failed` | wait 15 s after start; `ros2 lifecycle get /planner_server` → `active [3]`; restart `nav-to-point.sh` |
| Plan arcs only / `invalid motion model` | planner **SmacPlanner2D**. `sudo apt install ros-humble-nav2-smac-planner`, restart `nav-to-point.sh` |
| Two models in RViz, jumps | duplicate `map→odom` — rebuild `go2_nav2`, one **Pose Estimate** |
| Plan visible, robot still, `cmd tcp reconnecting` | update `go2_cmd_vel_tcp.py` on robot, restart relay; on laptop `/ws/scripts/check-nav-cmdvel.sh` — `/cmd_vel` ~15–20 Hz |
| Stops mid-route, `Failed to make progress` | wait for leg to finish (~60 s); do not spam Goal Pose; on robot `GO2_CMD_TIMEOUT=8` |
| Second Goal Pose silent | restart T1 after `colcon build`; log should show `Goal received` |
| `Transform data too old` odom→map | restart T1; do not spam Pose Estimate while driving |
| Motors grind, jerking | **only one** `sport_bridge`; do not set Goal while moving |
| `unitree_api` missing | `build-unitree-msgs.sh` on laptop; `robot-build-unitree-msgs.sh` on robot |
