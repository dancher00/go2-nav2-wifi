# Go2 — mapping and navigation

**Wi‑Fi (relay setup, once):** [RELAY-WIFI.md](RELAY-WIFI.md)  
**Map:** `/ws/scripts/go2-session.sh start mapping` · **Nav:** `/ws/scripts/nav-to-point.sh` + RViz **Goal Pose**

Use [managed sessions](SESSIONS.md) for duplicate prevention and scoped stop/status.
These commands are not a replacement for the handheld stop.

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
Odometry: use `GO2_ODOM_SOURCE=utlidar` for the timestamp-aligned pipeline, in both
mapping and navigation. `sport` is a legacy arrival-stamped alternative and is
not covered by the alignment fix.

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

**Using the handheld remote?** Start with [Mapping session](MAPPING-SESSION.md):
sensor-only relay, SLAM and RViz. No laptop motion bridge or teleop is needed.
The tables below describe the separate laptop-teleop workflow.

In each terminal: `cd go2-nav2-wifi/docker && ./shell.sh`

**Wi‑Fi** (relay on robot — teleop goes through it, **not** `sport_bridge` on laptop):

| T | Command |
|---|---------|
| **R** | on robot: `export GO2_HOST_IP=192.168.1.90 && bash ~/robot-relay-wifi.sh` |
| **1** | `/ws/scripts/go2-session.sh start mapping` |
| **2** | `ros2 run rviz2 rviz2 -d $(ros2 pkg prefix go2_nav2)/share/go2_nav2/rviz/slam.rviz` |
| **3** | `/ws/scripts/teleop-slam.sh` (speed: **q/z w/x e/c** in teleop) |

**Ethernet** (cable `192.168.123.x`):

| T | Command |
|---|---------|
| **1** | `ros2 launch go2_nav2 sport_bridge.launch.py` |
| **2** | `/ws/scripts/go2-session.sh start mapping` |
| **3** | `ros2 run rviz2 rviz2 -d $(ros2 pkg prefix go2_nav2)/share/go2_nav2/rviz/slam.rviz` |
| **4** | `/ws/scripts/teleop-slam.sh` |

Drive around the room. Check: `/ws/scripts/check-slam.sh` — `/scan` and `/map` must be present.

**Camera while mapping:** relay starts the bridge (`GO2_RELAY_CAMERA=1` by default) → `/go2_front_camera/image_raw`. Enable **Go2FrontCamera** in RViz. Check: `ros2 topic hz /go2_front_camera/image_raw`. If empty — on robot `bash ~/robot-build-camera-cli.sh`, restart relay; see [RELAY-WIFI.md](RELAY-WIFI.md).

**Explicit odometry selection:** keep `GO2_ODOM_SOURCE=utlidar` in `docker/.env`. Mapping:

```bash
/ws/scripts/go2-session.sh start mapping
```

Default `utlidar` translates `/utlidar/robot_odom` through the shared sensor clock
before publishing TF. Legacy `sport` uses `/sportmodestate` → `/sport_state/odom`
with arrival-time stamps; it is not the recommended mapping path.

**Teleop does not move robot over Wi‑Fi:** update `robot_relay_wifi.py`, `robot-relay-wifi.sh`, `robot_sport_bridge.py` on the robot (via `deploy-robot-wifi.sh`), restart relay; run `bash ~/robot-build-unitree-msgs.sh` once if `unitree_api` is missing.

**Jittering in place, `robot-check-cmdvel.sh` → NO /cmd_vel:** use `/ws/scripts/teleop-slam.sh` on the laptop (TCP client inside). Put the handheld remote aside.

**No map, log shows `pointcloud_to_laserscan` / `transform cache`:** rebuild `go2_nav2` and restart the entire mapping launch. `go2_cloud_stamp_sync` now translates cloud AND native odometry with one fixed clock offset; allow one second for calibration. Do not independently re-stamp either stream with arrival time. See [sensor timing](SENSOR-TIMING.md).

Save map (while T1 `slam_mapping` is running):

```bash
/ws/scripts/save-map.sh my_room
python3 -m go2_nav2.map_bundle check /ws/maps/my_room.yaml
```

All four files are required (`.yaml`, `.pgm`, `.posegraph`, `.data`). Existing
names are refused: choose a new name for each run. Ctrl+C in mapping terminals
or `/ws/scripts/go2-session.sh stop mapping` when done.

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

Managed navigation requires `GO2_ODOM_SOURCE=utlidar`; rebuild legacy `sport`
maps with the shared-clock pipeline before using this workflow.

**Ethernet:** T1 `sport_bridge.launch.py`, T2 `/ws/scripts/nav-to-point.sh`, T3 RViz.

**RViz order (T2):**

1. Wait **~12 s** after T1 starts. Check: `ros2 lifecycle get /map_server` → `active [3]`.
2. **2D Pose Estimate** (robot position + body heading). Scan should match walls.
3. **2D Goal Pose** — target. Blue line `/nav_plan`. Robot does **not** spin at the goal (final yaw ignored).

**Why it “does not move” without Pose Estimate:** slam_toolbox does not know your pose on the saved map until you set it. This is normal localization behavior (not GPS).

**Second goal does not run after the first:** wait ~2 s between goals (queue in `go2_goal_pose_nav`).

**Empty RViz / `unconnected trees`:** need `map→odom→base_link`. Without `/utlidar/robot_odom` from the robot there is no `odom→base_link` — start `bash ~/robot-relay-wifi.sh`. Check: `/ws/scripts/check-tf-nav.sh`.

**`/pose stale`:** the last timestamp-matched `map→odom` correction is held;
the displayed pose propagates through fresh `odom→base_link`. Before the first matched
pose the correction is identity. Stale SLAM poses do not mean localization is
healthy: stop with the remote if localization is wrong, and check scans and
alignment before sending another goal. There is no additional navigation gate.

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

Stationary measurements on this robot: native LiDAR about **15.5 Hz**, isolated
Wi-Fi relay about **15.4 Hz**, with no duplicate cloud timestamps. The configured
controller runs at 15 Hz and local costmap at 8 Hz. At 0.45 m/s, a 15 Hz scan
interval is about 67 ms / 3 cm of travel. This is not a guarantee of safe stopping:
Wi-Fi delay, processing, braking and physical obstacle tests still matter.
Increasing the reported relay rate by duplicating frames adds no information.

---

## Checks

```bash
ros2 topic hz /utlidar/cloud_deskewed
ros2 topic hz /scan
/ws/scripts/check-tf-odom.sh
ros2 node list | grep sport_bridge   # exactly one /sport_bridge
```

Duplicates or conflicting controllers: stop motion with the remote first, then
inspect ownership and stop the relevant managed session:

```bash
/ws/scripts/go2-session.sh status
/ws/scripts/go2-session.sh stop navigation
```

---

For an unmanaged launch (including an Ethernet sport bridge), use Ctrl+C in its
original terminal. No command above stops unrelated or factory robot processes.

## Common issues

| Symptom | Fix |
|---------|-----|
| No topics | Wi‑Fi: relay + laptop must share `GO2_RELAY_DOMAIN_ID=64`; internal robot and cable stay on domain 0 |
| No `.posegraph` | run `save-map.sh` while `slam_mapping` is active |
| `failed to create plan` | goal in free space, closer; scan aligned with walls |
| Robot in RViz wrong vs floor | **2D Pose Estimate** on `/initialpose` |
| Goal Pose silent / Nav2 not ready | log `planner_server FATAL` / `NavfnPlanner does not exist` — `sudo apt install ros-humble-nav2-navfn-planner ros-humble-nav2-dwb-controller` |
| `Planner rejected goal` / `controller_server Failed` | wait 15 s after start; `ros2 lifecycle get /planner_server` → `active [3]`; restart `nav-to-point.sh` |
| Plan arcs only / `invalid motion model` | planner **SmacPlanner2D**. `sudo apt install ros-humble-nav2-smac-planner`, restart `nav-to-point.sh` |
| Two models in RViz, jumps | duplicate `map→odom` — rebuild `go2_nav2`, one **Pose Estimate** |
| Plan visible, robot still, `cmd tcp reconnecting` | update `go2_cmd_vel_tcp.py` on robot, restart relay; on laptop `/ws/scripts/check-nav-cmdvel.sh` — `/cmd_vel` ~15–20 Hz |
| Stops mid-route, `Failed to make progress` | inspect planner/controller, fresh scans and TCP connection; do not extend command watchdogs to mask the failure |
| Second Goal Pose silent | restart T1 after `colcon build`; log should show `Goal received` |
| `Transform data too old` odom→map | restart T1; do not spam Pose Estimate while driving |
| Motors grind, jerking | **only one** `sport_bridge`; do not set Goal while moving |
| `unitree_api` missing | `build-unitree-msgs.sh` on laptop; `robot-build-unitree-msgs.sh` on robot |
