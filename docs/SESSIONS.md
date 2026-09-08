# Managed laptop sessions

Run inside the built laptop Docker container. Set `GO2_NET=wifi`, the current
robot/laptop IPs and `GO2_ODOM_SOURCE=utlidar` in the container environment.
The launcher loads ROS itself. Keep start commands in foreground terminals.

```bash
/ws/scripts/go2-session.sh start mapping
/ws/scripts/go2-session.sh status
/ws/scripts/go2-session.sh stop mapping
```

For handheld mapping use the robot's `--sensors-only` relay. Mapping never starts
a TCP motion client, sport bridge or stand command. Navigation/teleop require
the separately started robot motion relay and a supervised clear area; do not
use these modes for a lying-down sensor check.

| Mode | Owns | Compatibility wrapper |
| --- | --- | --- |
| `mapping` | SLAM + sensors | — |
| `navigation --map /ws/maps/room.yaml` | Localization, Nav2, Wi-Fi TCP client | `nav-to-point.sh /ws/maps/room.yaml` |
| `teleop` | Keyboard teleop + Wi-Fi TCP client | `teleop-slam.sh` |
| `transport` | Wi-Fi TCP client only | `start-cmdvel-tcp-client.sh` |

Use `start MODE`, `stop MODE` and `status [MODE]`. Mapping and navigation share
an exclusive stack lock; navigation, teleop and transport share a motion lock.
Mapping can coexist with teleop. Preflight observes advancing/recent native
cloud and odometry for four seconds and rejects known conflicting ROS nodes.
It does not take over or kill those nodes. Missing data or an invalid map fails
before the stack/TCP client starts.

## Ownership limits

Stop uses a private Unix control socket; the supervisor signals only process
groups it created. Child failure tears down the owned session. `status`/`stop`
do not depend on robot connectivity or ROS discovery. Locks are scoped to the
same container/filesystem, UID and configured domain; use the same environment
for all commands. Do not override `GO2_SESSION_DIR` between start and stop.

Locks do not coordinate other containers or laptops. Graph checks are best
effort, not global arbitration. Direct `ros2 launch` commands and factory robot
processes remain unmanaged: stop them in their original terminal. `not-managed`
is not proof that no ROS process or motion bridge exists. The old
`kill-stale-odom-tf.sh` now only reports how to stop an owned session.

## Navigation

Navigation keeps its existing direct `/cmd_vel` path and transport watchdogs.
There is no extra command-filtering node or sensor-readiness state machine.
Startup checks are one-shot diagnostics, not continuous motion supervision.
Set RViz **2D Pose Estimate**, check scan alignment and then send **Goal Pose**;
keep the handheld stop available. See [navigation](NAVIGATION.md).

## Map bundles

With mapping running and the ROS environment sourced:

```bash
/ws/scripts/save-map.sh room_run_02
python3 -m go2_nav2.map_bundle check /ws/maps/room_run_02.yaml
```

Saving refuses any existing component of the chosen name. All four components
are staged on the same filesystem, checked and published with exclusive links;
YAML is published last, with a relative image path. Existing maps are never
overwritten. Keep the `.yaml`, `.pgm`, `.posegraph` and `.data` together.
Validation checks metadata and nonempty files, not serialized graph integrity
or geometric mapping quality. Save/reload navigation still needs a walking test.

## Verification scope

`bash ws/scripts/test-regressions.sh` runs supervisor ownership, map-save,
sensor timing and existing transport watchdog tests without external network.
On 2026-09-08 the lying-down robot supplied mapping data at about 15.4 Hz:
managed startup/status, duplicate refusal, four-file saving, no-overwrite and
owned shutdown were checked. No navigation/motion test was performed. The saved
`session_smoke_20260908_guard` map is a pipeline artifact, not a navigation map.
Physical stopping time, walking drift and end-to-end goal execution remain
unverified for these changes; see [the roadmap](ROADMAP.md).
