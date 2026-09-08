# Project roadmap

## Phase 1 — polish the existing LiDAR + Wi-Fi solution (current)

Keep the native Unitree LiDAR, `utlidar` odometry, laptop Humble/Docker and
sensor-only handheld mapping as the baseline. No camera is required.

Acceptance checklist (an unchecked item is not yet verified):

- [x] Shared acquisition-time translation for cloud, odometry and TF, with
  duplicate/stale rejection and clock-discontinuity tests.
- [x] Stationary live validation and historical scan-time TF lookups, documented
  in [sensor timing](SENSOR-TIMING.md).
- [x] One isolated regression-test command for local runs and CI.
- [x] Clear handheld mapping workflow and fixed-map RViz defaults.
- [ ] Supervised repeated walking loops: record route, distance, speed and
  wall/loop-closure error; publish repeatable before/after evidence.
- [ ] End-to-end save/reload/localization and repeated A-to-B goals with the
  new timing pipeline, including goal cancellation and stopping.
- [ ] Scoped session ownership and duplicate-start prevention for all laptop
  launchers; eliminate broad process-kill patterns.
- [ ] Sensor and network fault-injection tests: missing odometry/scans, Wi-Fi
  interruption, clock reset, reconnect and measured stopping response.
- [ ] Latency p50/p95/p99, dropout and bandwidth reports under representative
  load; do not infer acquisition latency solely from restamped headers.
- [ ] Reproduce installation on a clean supported environment, pass hosted CI,
  and publish a release with known limitations and a tested configuration.

Automated tests and stationary measurements are not safety certification or
proof of navigation accuracy. Robot motion tests require a supervised clear
area and the handheld stop available. Changes to a running robot session are
not part of offline cleanup.

## Phase 2 — Wi-Fi + D435i visual SLAM (next, not started)

Add a separate optional profile; retain the validated LiDAR baseline.

1. Verify USB/backend/compute support on the actual robot. Establish camera
   intrinsics, mounting TF and camera/IMU/robot clock relationships.
2. Select and integrate a visual or visual-inertial SLAM backend; decide where
   odometry runs and which image/depth/pose data crosses Wi-Fi.
3. Measure tracking failures, trajectory/map error, relocalization, CPU/GPU
   load, bandwidth and end-to-end latency against the LiDAR baseline.
4. Test camera loss, weak lighting/texture and network degradation with explicit
   failure handling; do not silently switch odometry origins in a running map.

Adding D435i alone is not claimed as research novelty. The intended contribution
is a reproducible, measured implementation with clear operating limits.
