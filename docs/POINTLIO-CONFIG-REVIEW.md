# Point-LIO configuration review — 11 September 2026

Follow-up: the controlled recording on 12 September exposed a stationary startup
failure. See [the denser initial-map fix](POINTLIO-STARTUP-FIX.md); the comparisons
below describe the earlier 2,000-point baseline.

The user asked whether configuration changes could solve the problem; this
investigation prioritizes that question before further algorithm changes. No working Jetson settings or binary were changed by this investigation.
The stance-foot implementation remains an isolated, unpromoted experiment.

## Primary sources checked

- [CMU Go2 real-robot setup and IMU calibration](https://github.com/jizhang-cmu/autonomy_stack_go2/blob/foxy-humble/README.md#imu-calibration): requires per-robot L1 IMU calibration. Its utility commands stepping, standing and turning; it was not run here.
- [CMU launch](https://github.com/jizhang-cmu/autonomy_stack_go2/blob/foxy-humble/src/slam/point_lio_unilidar/launch/mapping_utlidar.launch): overrides YAML with `use_imu_as_input=false`, `prop_at_freq_of_imu=true`, and 0.1 m scan/map filters. Our baseline already uses those values.
- [CMU YAML](https://github.com/jizhang-cmu/autonomy_stack_go2/blob/foxy-humble/src/slam/point_lio_unilidar/config/utlidar.yaml): LiDAR measurement covariance 0.01, output-model covariance values 500/1000, plane threshold 0.1 and zero gravity match our baseline. Its IMU interval is 0.01; ours is 0.004 for the observed 250 Hz L1 stream. Initialization map size differs: launch 10, ours 2000; this was not varied in the present comparison.
- [Unitree L1 configuration](https://github.com/unitreerobotics/point_lio_unilidar/blob/main/config/unilidar_l1.yaml): also uses 500/1000, 0.004 s, 0.01 LiDAR covariance and 0.1 plane threshold. Large covariance numbers alone are not evidence of a typo. Its full-IMU/gravity settings must not be mixed blindly into CMU's gyro-only adaptation.
- [CMU sensor transformation](https://github.com/jizhang-cmu/autonomy_stack_go2/blob/foxy-humble/src/utilities/transform_sensors/transform_sensors/transform_everything.py): loads gyro calibration, compensates cross-axis effects, then zeros acceleration for the SLAM input. Our raw-axis route does not use this transformer. Its rotations and calibration constants must not be copied without matching the sensor frames.
- [Point-LIO authors' notes](https://github.com/hku-mars/Point-LIO#important-notes): emphasize sensor synchronization, per-point timestamps, IMU units/saturation and correct rigid transforms. The extrinsic is the LiDAR pose **in the IMU frame**.
- [CMU issue 22](https://github.com/jizhang-cmu/autonomy_stack_go2/issues/22): reports the same stationary drifting symptom. The issue is evidence that the symptom is known, not evidence of a verified remedy. GitHub's rendered page did not supply discussion comments; API requests for issue 22/27 timed out.

## Actual configuration-only repetitions

The unmodified installed `/opt/go2-lidar3d` executable replayed the same first
90 seconds of `run-20260911-181720-bj8yn462`, at 1x on the laptop. Each run saved
1,361 poses over 88.41 seconds and 725,179 points. Only one parameter was changed
per run; acceleration remained disabled and no foot observations were used.
A separate control run of the stock executable produced a trajectory bitwise
identical to the experimental executable with additions disabled (all 1,361 poses).
The comparison therefore includes a stock-executable baseline.

| Configuration | Median local plane spread | 90th percentile |
|---|---:|---:|
| Baseline | 26.79 mm | 49.81 mm |
| `plane_thr=0.05` | 27.12 mm | 49.99 mm |
| `lidar_meas_cov=0.001` | 26.97 mm | 50.42 mm |
| `imu_meas_omg_cov=0.01` | 27.48 mm | 50.11 mm |

The first two candidate values also appear as alternatives in upstream config
comments. The third is a diagnostic change in gyro weight, not an upstream
recommendation. These tests did not establish a map-quality improvement.
Local plane spread uses covariance eigenvalues in populated 0.5 m cells; changing
cell coverage can affect the metric. It is not surveyed accuracy. Endpoint motion
on this walking prefix is not an error measurement or a confirmed loop return.

[Detailed measurements](measurements/pointlio-config-replay-2026-09-11.json).

## IMU check and next step

Raw L1 acceleration is present: its norm's 5th/50th/95th percentiles were
8.92/10.21/15.91 m/s² on this recording. The earlier conversation's explanation
about zero acceleration applied to the gyro-only estimator input, not to missing
hardware measurements. Body-IMU acceleration was 8.00/9.50/10.58 m/s².

A comparison against the body gyro found a static median difference of approximately
[0.0148, -0.0191, -0.0020] rad/s after the existing rigid rotation. This is a
**difference between sensors**, not an independently measured L1 bias. A rotation
fit changed held-out dynamic RMS only from 0.203 to 0.200 rad/s, so no calibration
was accepted or deployed. A controlled stationary/turning recording and a check
of acquisition time alignment are better-founded next steps than blindly changing
covariances, gravity, or reusing another Go2's coefficients.

[IMU diagnostics](measurements/pointlio-imu-diagnosis-2026-09-11.json).
