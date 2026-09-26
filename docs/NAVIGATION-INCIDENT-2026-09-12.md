# Navigation incident, 2026-09-12

Session: `ws/log/mapping.btRpeA`; recording: `ws/maps/lidar3d/run-20260912-003417-q1vdj9j4`.

Navigation was stopped and the remote motion bridge and TCP server were verified absent. Recording was copied with checksum verification (12 files, 334951734 bytes). No navigation restart was performed during diagnosis.

## Observed

The first goal produced a 55-pose, approximately 2.42 m path and FollowPath was accepted. The second goal was queued behind the still-active first goal. The robot turned substantially and then barely translated. Both raw gyro and factory odometry confirm real rotation. Five-second motion log snapshots show a -0.80 rad/s turn command followed by approximately 0.06 m/s or 0.03 rad/s commands. TCP reception continued at approximately 20 Hz. Snapshots do not contain the full command history.

A subsequent four-timestamp SSH clock measurement found Jetson ahead of the laptop by 5.057987957 seconds, with best-sample uncertainty 0.000977287 seconds. Evidence: `ws/log/mapping.btRpeA/clock-offset.json`. This is a post-incident measurement, not an exact offset history during the run. Laptop timedatectl reported unsynchronized; recent systemd-timesyncd logs showed repeated NTP timeouts. Jetson reported synchronized.

SLAM translates sensor stamps to its local Jetson clock; body odometry preserves these stamps. Navigation runs on the laptop. The upstream Humble RotationShimController explicitly stamps its sampled path point with its own current time before transforming to the body frame. With future-dated robot transforms this can select an older robot orientation from the TF buffer, explaining delayed turn feedback and making clock skew a strong causal candidate. An isolated replay or corrected-clock driving test is still needed to confirm the complete failure mechanism.

Source: https://github.com/ros-navigation/navigation2/blob/humble/nav2_rotation_shim_controller/src/nav2_rotation_shim_controller.cpp (getSampledPathPt, transformPoseToBaseFrame). Installed binary behavior was not independently instrumented.

## Limits and next action

Do not compare laptop and robot log timestamps directly. The recording lacks full commands, planned path, costmaps, and TF lookup results, so it cannot conclusively distinguish all controller effects. No speculative tuning or further speed increase was applied during diagnosis. Before the next driving test, establish a common clock and verify timestamp age across the complete pipeline; capture commands, path and feedback. The incident is diagnosed in part, not validated as fixed.
