# URDF self-filter for 3D navigation

The navigation input `/lidar3d/obstacle_points` now comes from
`go2_urdf_self_filter`. The previous low-return heuristic is removed.
Point-LIO and `/lidar3d/registered` retain their original input/output.

The filter subscribes to the same `/robot_description` used by RViz and
uses current `/joint_states`. Each URDF visual mesh is represented by its
own convex hull; box, cylinder and sphere visuals use primitive tests.
Visual origins, mesh units/scales, joint axes and fixed transforms are
respected. Convex hulls are built once at model load. Per-link bounding
boxes reject distant points before detailed plane tests. This does not
fill the spaces between legs with one enclosing hull. Visual geometry
was chosen because this model's collision trunk is much smaller than
its visual body. Concavities within one mesh are approximated by its hull.

The default padding is 0.015 m. It is not expanded to cover unexplained
nearby returns. The filter removes only points inside those articulated
volumes, including their small padding. Ground and external objects are
not removed merely because of height or closeness to the body.
`/lidar3d/self_points` shows the removed points in pink in the 3D RViz
planning/navigation configurations.

Cloud-to-body TF is requested at the cloud timestamp. LowState has no
sensor timestamp. In 3D mode the joint bridge associates joint measurements
received within 0.2 s with each Point-LIO pose and publishes the identical
pose stamp, keeping the visual TF tree coherent. The filter additionally
matches joint samples by nearest reception time (max 0.1 s). This is not
hardware synchronization; fast leg motion still needs walking validation.
If URDF, joints or TF are unavailable, the queue waits up to 0.3 s, then
passes the original cloud without deleting points and reports a warning.
The robot/laptop clock discrepancy itself is not corrected by this filter.

Measured on the laptop, 2026-09-11:

| Measurement | Result |
|---|---:|
| Geometry only, recorded 600-point scan, median / p95 | 2.32 / 2.51 ms |
| Geometry only, 12,257 recorded points, median / p95 | 14.60 / 16.88 ms |
| Live complete callback including ROS cloud conversion, p95 | 10.7–18.6 ms |
| Live reception-to-publication including queue/TF wait, p95 | 25.1–36.0 ms |

Live session: `ws/log/mapping.6KDQzW`, sensor-only `--3d --plan`.
Model: 29 visuals, 12 moving joints, SHA256
`54c084152d8941e546381895e3d064cb9b23c1a7e18c07983e29595e2d38911d`.
Tests cover articulated movement, retention of gaps and nearby external
points, missing-joint rejection, and loading the actual expanded Go2 model.
The live filter loads and processes scans; these tests do not establish
that all previously observed false obstacles are self returns or that
navigation during walking is now reliable.

A live TF check found body and leg timestamps 1.26 s apart before joint/body
pose pairing, causing about 1 cm of visible separation even at rest.
After pairing, all four hip world positions match body-pose × URDF-link
transform to numerical precision, with identical timestamps. Session:
`mapping.FMRAgz` (sensor-only). Six ROS tests passed, including stale-joint
rejection and existing odometry geometry checks.

## Shadow returns

Containment alone did not remove the reported obstacles: all four recorded
near-body obstacle points were outside the visual envelopes but their rays
from the calibrated sensor origin intersected the front hip/thigh meshes.
The filter now additionally tests ray occlusion against the articulated URDF
within 1 m of the sensor. This follows the containment/shadow distinction in
[robot_body_filter](https://github.com/ctu-vras/robot_body_filter).

Sensor-containing shapes are excluded from shadow tests because the trunk
visual convex hull encloses the embedded sensor. Their point-containment
checks remain enabled. Meshes use convex-plane ray clipping; box, sphere and
cylinder visuals use corresponding solid intersections. The 1.5 cm padding
is unchanged. No height band or enlarged rectangular body mask is used.

On the recorded scene, all 4 of 4 near-body obstacle returns were classified
as shadowed; see `measurements/urdf-shadow-check-2026-09-11.json`.
`/lidar3d/shadow_points` displays these rejected returns in orange.
Tests cover points before, behind and beside a body part, range limiting,
sensor-inside handling, primitives, and a moving articulated link.
Earlier latency measurements above describe containment only; shadow-test
latency must be measured separately on the live stream.

Live shadow validation (`mapping.mdsxuf`, sensor-only): processing p95
27.2–31.2 ms; reception-to-publication p95 42.3–49.5 ms; no scans bypassed
for missing transforms or joints in the observed intervals. After more than
20 s of scans, the global costmap had zero lethal cells inside the padded
body footprint. Unknown cells remain under the robot because occluded
returns are no longer used to mark or clear space. This is not walking or
low-obstacle avoidance validation.

## Latency follow-up

The filter previously crashed when a missing-data interval changed a shared
logging call site from INFO to WARN: Humble raises `ValueError: Logger severity
cannot be changed between calls`. Separate logging call sites fix this; a ROS
regression test covers INFO → WARN → INFO. The launch also respawns the filter
after unexpected process exits.

Planning/navigation RViz configs retain 10 seconds of recent scans and render
at 10 FPS (previous planning retention: 120 seconds). The robot relay cloud
interval is now 0.1 seconds instead of 0.2 seconds. Mapping data recording is
unchanged; this interval controls the relayed navigation/visualization stream.

Live sensor-only session `mapping.pCBnDB`, after battery replacement/reboot:
- Relay cloud rate: 7.6–7.8 Hz, previously approximately 4 Hz.
- Two complete 10-second filter reporting windows: 77 scans each, no bypasses;
  processing p95 22.5–23.8 ms, reception-to-output p95 37.2–37.4 ms.
- Independent 8-second subscriber received 62 raw and 61 filtered scans
  (the observation window can end before the final filtered publication).
- RViz CPU averaged 156% over 10 seconds with the display window filled;
  earlier observed CPU was approximately 259%. This is an observational
  comparison across sessions, not a controlled rendering benchmark.
- Relay regression suite: 16 passed; ROS logging regression: 1 passed.

These measurements exclude sensor acquisition, onboard SLAM, network transit
before reception, and rendering latency. Walking/navigation was not tested
in this sensor-only session.
