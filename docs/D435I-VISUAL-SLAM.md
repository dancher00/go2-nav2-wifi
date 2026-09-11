# Go2 + D435i indoor RGB-D mapping, onboard Jetson

Branch: `experiment/d435i-visual-slam`, worktree `/home/danya/go2-d435i-visual-slam`.
The default now follows the Go2-specific demo architecture: **native Go2 odometry
+ D435i RGB/depth + RTAB-Map visual loop closure and map optimization**.
This is camera mapping assisted by the robot's odometry, not camera-only VIO.
All processing runs on the Jetson; the laptop runs RViz only.

**Current status: failed ramp traversal.** The native-odometry configuration
did not reconstruct a descent and return to the same wall correctly: its
optimized endpoint remained approximately 4 m below the initial point. Local
turn tests do not validate this configuration for room or multi-floor mapping.
The saved map is paused for investigation; no successful replacement is deployed.

## Reference implementations

- [L-winder2002 / Go2 + D435i + RTAB-Map](https://github.com/L-winder2002/Unitree-Go2-Mapping-and-Navigation-Using-Intel-RealSense-D435i-and-RTAB-Map):
  specifically uses `/utlidar/robot_odom` as movement input with a camera mounting
  transform. We implement an independent isolated receiver and immutable clock
  translation instead of restamping the latest pose on every image.
- [Hossein Naderi's Go2 SLAM demo](https://h-naderi.github.io/projects/1-slam) and
  [source](https://github.com/h-naderi/unitree-go2-slam-nav2): the showcased system
  combines D435i and a RoboSense 3D LiDAR. It is not a camera-only demonstration.
- [Official RTAB-Map D435i RGB-D example](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_examples/launch/realsense_d435i_color.launch.py)
  and [stereo example](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_examples/launch/realsense_d435i_stereo.launch.py).
- [RealSense ROS driver](https://github.com/realsenseai/realsense-ros).

The first camera-only RGB-D profile passed static checks but failed during the
user's walk: visual registration lost its feature correspondences, then stopped
producing useful movement estimates. Fixing message synchronization alone did
not fix visual tracking. Do not use the initial static results as evidence of
walking performance. The previous session databases and logs are preserved.

## Hardware and isolation

Inspected 2026-09-09: Orin Nano 8 GB, Ubuntu 20.04, L4T R35.3.1 / JetPack 5.1.1,
D435i USB 3, firmware 5.15.1.55. ROS 2 Humble runs in Docker without host upgrades.
Installed packages: RealSense ROS 4.58.3, librealsense 2.58.3, RTAB-Map ROS 0.23.7.
The Docker build uses the live ROS apt repository; repeat checks after rebuilding.

- Jetson container/image: `go2-d435i-onboard` / `go2-d435i:local`.
- Jetson directory: `~/go2-d435i-visual-slam`.
- Results/visualization DDS domain: **65**, dedicated CycloneDDS Wi-Fi peers.
- Only the native odometry reader joins domain **0**, on `eth0`. It subscribes to
  `/utlidar/robot_odom`; it publishes no motion commands or TF into that domain.
- Laptop: `go2-d435i-rviz`. No camera/IMU subscriptions are enabled in RViz.
- LiDAR experiment containers, files, services and host network/power settings
  are not managed by these scripts. CPU/RAM/Wi-Fi are shared resources.

The native RealSense V4L/IIO backend needs writable IIO trigger attributes. Only
RealSense USB sysfs subtrees are bound writable; the container is not privileged.
Video, USB and IIO devices are passed through. Restart this experiment after
unplugging/replugging the camera, because device IDs may change.

## Pose, timing and calibration

```
d435i_map -> d435i_odom -> d435i_base -> camera_link -> camera optical frames
```

Native Go2 odometry is about 150 Hz. Its source clock was approximately 763.83 s
behind the Jetson wall clock. A one-second calibration locks the minimum
arrival-minus-source offset once, then translates every acquisition timestamp
by that same amount. This is an epoch estimate with unknown residual transport
latency, not a hardware camera/robot synchronization calibration.

No stale pose is periodically restamped. Duplicate/stale measurements are
rejected; clock discontinuities and large coordinate jumps latch a fault until
a new session. A local origin removes the initial world translation and yaw,
while preserving measured roll/pitch and the gravity-aligned vertical axis.
RTAB-Map requests historical TF at each image's timestamp. RGB, aligned depth
and camera info use exact synchronization and reliable subscriptions.

The user confirmed the camera is rigidly mounted facing forward. Rotation is
therefore initialized to zero. **Translation is provisional:** default
`x=0.20, y=0, z=0.15 m` comes from the reference demo, not a measurement of this
robot. Measure from the Go2 body frame to the D435i camera frame and override it:

```bash
./d435i.sh start camera_x:=0.20 camera_y:=0.0 camera_z:=0.15
```

An incorrect lever arm causes geometric errors during turns. Camera mapping
is not yet calibrated robot localization for autonomous navigation.

## Run

```bash
cd /home/danya/go2-d435i-visual-slam
./d435i.sh setup     # install/update only the Jetson experiment
./d435i.sh start     # native Go2 odometry + RGB-D mapping (default)
./d435i.sh check     # 20-second read-only check on Jetson
./d435i.sh rviz      # laptop visualization
./d435i.sh status
./d435i.sh stop      # graceful database save, preserve log, remove own container
```

Defaults: robot `unitree@192.168.8.245`, laptop IPv4 inferred from its route.
Override `GO2_ROBOT_IP`, `GO2_ROBOT_USER`, `GO2_HOST_IP`; rerun setup after
address changes. SSH keys/password prompts are standard; no password is saved.
`GO2_SSH_CONTROL_PATH` can reuse an existing connection.

The local RViz image is `go2-humble:local` (override `GO2_RVIZ_IMAGE`). DISPLAY
and XAUTHORITY come from the desktop. Closing RViz does not stop mapping.

Every start creates `data/<UTC timestamp>-<pid>/rtabmap.db` on the Jetson;
`data/latest` points there. Stop gives RTAB-Map up to 45 seconds to save the
SQLite database, writes `launch.log`, then removes only this container. Existing
sessions are retained. `start odom_source:=rgbd` retains the earlier experimental
camera-only profile, with no Go2 odometry reader; it is not the recommended mode.

## What RViz shows

- Dense color map: RTAB-Map's assembled, graph-corrected `/d435i/cloud_map`.
  Cell size 4 cm, depth decimation 2, depth range 0.4–4 m.
- Yellow trajectory: `/d435i/mapPath`, with the SLAM graph poses.
- Small axes: current body pose, `/d435i/odom`.
- Live status: green only while odometry, camera info and RTAB-Map updates are
  all recent. A stale input or clock/pose fault turns it red. Green confirms a
  live pipeline, not correct room geometry or a successful loop closure.
- Occupancy grid: optional, disabled initially.

Camera images are configured at 640x480, 30 Hz; graph processing is configured at 2 Hz.
Built-in D435i depth computation and CPU mapping on Jetson are used; no GPU
acceleration is claimed. IMU is published and filtered, but Go2 mode gets motion
from native odometry, not from tightly coupled visual-inertial estimation.

## Validation

The checker reports sample/rate counts, valid odometry, required TF edges,
nonempty map, mapping updates, observed position span and detected loop IDs.
A 15-second Go2-mode check received 2245/2245 valid poses (~149.55 Hz),
27 mapping updates (~1.83 Hz), and a 13905-point map. This sample was stationary.

The extended report is [90-second check](measurements/d435i-go2-90s-2026-09-09.json).
Check its position span and loop IDs before claiming a walking or return-loop
test. A complete room map requires observations from a room traversal; static
samples do not validate drift, alignment on turns, or loop-closure quality.

Initial camera-only measurements remain under `docs/measurements/` as historical
evidence. A previous graceful save passed SQLite integrity checking (172 stored
nodes, 65,839,104 bytes); stored intermediate nodes are not independent places.
Regression checks cover clock handling and relative-pose anchoring/reset rejection.

### Walking geometry failure and offline review

The user completed a traversal with a 4.38 x 5.53 m observed position span.
The 90-second observer recorded 13,309 valid native poses and one reported loop
closure; the cloud reached 812,267 points. Its TF failure was an observer queue
issue: `/tf_static` has multiple publishers, and depth 1 discarded a historical
message before it was processed. The checker now uses depth 100 for `/tf_static`;
the [follow-up check](measurements/d435i-go2-final-2026-09-09.json) passed.

**The resulting geometry was nevertheless visibly smeared and rejected by the
user. This mode is not validated for accurate room reconstruction.** Mapping was
paused and `/d435i/rtabmap/backup` saved `rtabmap.db.back` before a battery swap.
Session: `20260909T161102-52111`. No new session should silently reuse its origin
following a robot reboot.

Offline work runs on Jetson in `go2-d435i-review` (2 CPU cores, 3 GiB memory,
network disabled), with outputs under that session's `review/` directory.
Native and optimized exports show a maximum 2.28 m graph position change.
Removing camera-IMU gravity constraints alone still gives 2.27 m; it does not
resolve the visible error. The unoptimized cloud also has misaligned surfaces.
RGB-D pair registration is being used to separate pose/registration errors;
planar PnP ambiguities must not be treated as reliable ground truth.

`analyze-d435i-pairs.py` and `compare-d435i-maps.py` are offline diagnostic scripts.
A depth-ICP replay with native odometry guesses is an experiment on a copied
input database, not yet the live default or a claimed successful reconstruction.

### Battery restart and exposure correction

After the battery replacement, the original database remained intact. Offline
ICP odometry lost tracking; native odometry with ICP neighbor-link refinement
completed (three closures) but its exported surfaces were still smeared. Neither
experiment was promoted to the live launch. Removing gravity constraints also
failed to resolve the map. The cause of the full geometry error remains open.

The recorded RGB frames show pronounced motion blur on turns (for example nodes
253 and 280). The live camera now requests 640x480 at 30 Hz with manual RGB
exposure 78 (D435 UVC units, 7.8 ms), gain 64, and depth exposure 8000 us/gain 64.
`config/d435i_camera.yaml` is a flat dictionary for the RealSense launch file.
Exposure must be adjusted for a different room's lighting; this is not a universal
calibration. Live metadata confirmed actual_exposure=78, gain_level=64,
auto_exposure=0 and actual_fps=29978 (29.978 Hz). The captured stationary image
had median intensity 86/255. Reduced motion blur is **not yet verified in motion**.

New session: `20260909T163123-16054`. A 20-second stationary check passed all TF
and liveness checks: 2984 valid odometry samples, 39 mapping updates, and RGB/depth
observer rates of 22.0/28.7 Hz. This is a transport check, not room-map validation.
The map range is now limited to 0.4–4 m with 4 cm cells. RViz uses points at 5 FPS;
the liveness marker expires after one second without updates so reboot cannot
leave a stale green status. The laptop runs only RViz; replay processing stays
in the isolated Jetson review container.

### Return-test diagnosis: stereo saturation (corrected)

The user returned to the start in session `20260909T163123-16054`. The saved
database passed SQLite integrity checking. It contains 63 map keyframes, two
visual loop closures and six proximity closures. The maximum optimized pose
correction was 0.194 m, but surfaces remained poor. Start/end camera positions
differ by 0.248 m raw and 0.237 m optimized, largely in height (the first frame
was recorded while the robot was lying down). These are not measured closure
errors or ground-truth accuracy estimates.

Inspection of raw IR images found the manual stereo settings introduced above
were unsuitable: gain 64 and exposure 8000 us saturated 76.3%/75.2% of the IR
images. After enabling stereo auto exposure, saturation fell to zero, raw depth
coverage at 0.4–4 m increased from 21.6% to 82.2%, and aligned depth coverage was
89.1%, without moving the robot. Metadata reported gain 16 and exposure 7759 us.
See [measured comparison](measurements/d435i-exposure-comparison-2026-09-09.json).

The current config keeps manual short RGB exposure but **stereo auto exposure**
with initial gain 16. The earlier manual-depth configuration is superseded.
Bad depths in the saved traversal cannot be repaired by graph optimization.
The checker now reports usable depth coverage and optional IR saturation,
with warnings for sparse depth or saturated IR, explicitly separating these
checks from map accuracy. Room reconstruction with this correction still needs
validation in motion.

The first turn with corrected stereo exposure was saved in session
`20260909T164043-21904`: 256 recorded nodes, nine map keyframes, approximately
98.74 degrees of yaw in one direction, no return turn or loop closure yet.
The database passed integrity checking. Native and optimized top-view exports
show a substantially narrower main wall than the saturated-depth traversal,
with remaining outliers. Maximum graph correction was 0.0173 m; this does not
measure map accuracy. See [turn summary](measurements/d435i-turn-2026-09-09.json).
Mapping resumed into the same database to allow a return-view comparison.

The return turn was subsequently saved in the same session (1392 recorded
nodes, 18 map keyframes). Five return-view loop links connect nodes 1344–1351
to earlier views. The unoptimized top view shows doubled main-wall contours;
the optimized export brings those contours together, with remaining outliers.
This supports the local turn/overlap test, not room-scale mapping accuracy.
Maximum graph position correction is 0.105 m, not an accuracy measurement.
The final heading is approximately 31 degrees from the initial heading, so
start/end pose separation must not be reported as measured drift.
See [return summary](measurements/d435i-return-turn-2026-09-09.json) and
[raw/optimized top views](measurements/d435i-return-turn-comparison.jpg).
Mapping resumed into this same map after backup; all exports ran on Jetson.

### Ramp traversal and return: failed

The user subsequently descended a ramp and returned to the starting wall.
Session `20260909T164043-21904` now contains 1952 recorded nodes and 187 map
keyframes; the backup passes SQLite integrity checking. The full native pose
history spans only 0.04088 m in Z. The optimized graph spans 3.9989 m in Z and
ends at Z=-3.82539 m versus initial Z=0.151045 m. This difference is inconsistent
with the reported return; it is not an estimate of actual floor height.
The latest loop link targets node 1738, before the final return, so the final
return did not reconnect the map to the original wall.

The native motion source does not provide the necessary vertical trajectory
for this traversal. Optimizing RGB-D loop links against this motion produced a
bad map; the flat-ground turn test was insufficient validation of the design.
Replacing this input with a verified 6-DoF motion estimate remains necessary.

Two offline replays of nodes 1640–1900 recomputed odometry with native guesses:
depth ICP tracked 9/261 frames and lost 252; visual RGB-D tracked 8/261 and lost
253. Neither result is usable or deployed. The recorded mapping images are
roughly 2 Hz, so these failures also do not establish performance of a new
full-camera-rate odometry implementation. Results and exports are on Jetson
under `ramp-review/`; see [failure report](measurements/d435i-ramp-failure-2026-09-09.json).
Mapping remains paused with the failed map retained in RViz for inspection.
