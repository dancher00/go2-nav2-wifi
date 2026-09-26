# CMU comparison after live drift diagnosis

Reviewed `foxy-humble` at `43d5f54b389b251713f0097893c30fa76c870d54` on 2026-09-09.
Sources were inspected as code, not inferred solely from the README.

- [Sensor transformer](https://github.com/jizhang-cmu/autonomy_stack_go2/blob/43d5f54b389b251713f0097893c30fa76c870d54/src/utilities/transform_sensors/transform_sensors/transform_everything.py): consumes raw `/utlidar/cloud` and LiDAR `/utlidar/imu`. Rotates axes, subtracts gyro bias, applies calibrated Z-to-X/Y gyro cross-axis compensation. Publishes a corrected raw IMU separately, but **sets acceleration to zero and orientation to identity in the IMU sent to Point-LIO**. Applies one cloud-derived clock offset to both streams. This is not per-message receipt restamping.
- [Point-LIO configuration](https://github.com/jizhang-cmu/autonomy_stack_go2/blob/43d5f54b389b251713f0097893c30fa76c870d54/src/slam/point_lio_unilidar/config/utlidar.yaml): transformed cloud/IMU, gravity zero, fixed extrinsics, `use_imu_as_input=true`. Do not equate the flag with actual acceleration use: upstream transformer has already zeroed it. No leg encoder or factory world-position input in this SLAM pipeline. Source inspection found no loop-closure pose graph in this Point-LIO backend; navigation/FAR visibility graph is not a SLAM pose graph.
- [Calibration](https://github.com/jizhang-cmu/autonomy_stack_go2/blob/43d5f54b389b251713f0097893c30fa76c870d54/src/utilities/calibrate_imu/src/calibrate_imu.cpp): estimates static biases and gyro Z leakage into X/Y during yaw rotation. README requires robot-specific calibration. This is not a full spatial/temporal LiDAR/IMU calibration. The executable drives robot motion; it was **not run**.
- [Deployment and delay notes](https://github.com/jizhang-cmu/autonomy_stack_go2/blob/43d5f54b389b251713f0097893c30fa76c870d54/README.md): onboard Foxy or external Ethernet; reports >1 s delays with external Humble and transient startup delay also with Foxy. Wireless HDMI in onboard setup carries the display; it is not evidence for raw-sensor Wi-Fi performance. Their reported delays do not prove our live drift is a Wi-Fi delay.

## Implication for this project

The existing `--3d` Point-LIO profile already derives from CMU. Current `--3d --legkilo` instead uses body IMU and encoder kinematics, so CMU's LiDAR gyro coefficients must not be copied onto the body IMU. CMU avoids our LowState/Sport pairing dependency. It does not supply a demonstrated solution to global loop drift.

Useful next controlled baseline is the existing Point-LIO route with verified LiDAR gyro calibration and accepted inputs. Preserve the relay, sessions and sensor-only mode. Do not import the entire navigation stack or its command nodes. Compare using the same recordings before a new live walk; zeroed/transformed IMU recordings cannot recover raw sensor calibration information that was not recorded.

No CMU code was installed or launched during this review.
