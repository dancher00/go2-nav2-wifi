# Improving 3D estimation: native state and leg kinematics

Review and read-only robot measurement, 2026-09-09. No fusion backend was enabled
by this investigation. Current Point-LIO remains gyro-assisted without acceleration
or loop closure; it has not met a walking-accuracy acceptance criterion.

## What Unitree already exposes

The [official SDK/ROS2 interface](https://github.com/unitreerobotics/unitree_ros2#state-acquisition)
exposes position, velocity, body IMU orientation, foot positions and forces through
SportModeState. These are state estimates, not independent raw leg odometry.
The public interface does not reveal the production estimator's sensor fusion.

On this Go2, `/sportmodestate` delivered ~299 Hz, `/utlidar/robot_odom` ~151 Hz,
and `/lowstate` ~498 Hz. Pairing native odometry and SportModeState by their source
stamps gave a p95 time gap of 1.95 ms, p95 position difference 2.47 micrometres,
and p95 attitude difference 0.00177 degrees; ~75% positions were bitwise equal.
This stationary/mode=0 observation supports treating these as correlated outputs,
not independent inputs to another filter. It is not proof of their internal algorithm.

During the overlapping ~12 seconds, Point-LIO's position extent reached 4.73 cm
on one axis; native pose extent was below 0.1 mm. Both were observed directly
on DDS domain 0, before Wi-Fi and RViz. This establishes variation in the estimator
output; neither source is ground truth. Factory states can also be held or constrained
in idle mode. Our session stopped normally during the observation; native streams
continued. `/uslam/frontend/odom`, `/uslam/localization/odom` and
`/lio_sam_ros2/mapping/odometry` delivered no samples, so their advertised names do
not provide an active ready-made factory SLAM baseline.

[Measurements](measurements/lidar3d-odometry-sources-2026-09-09.json).

## Candidates checked against source

| Candidate | What it offers | Fit and limitations |
|---|---|---|
| Native SportModeState/robot_odom | Already available body state | First practical comparison baseline. Clouds accumulated with it must be labelled factory-odometry mapping, not independent SLAM. Production fusion internals unknown. |
| [Leg-KILO, current master](https://github.com/ouguangjun/Leg-KILO/tree/4b29f2f7d175dcf661f23f01f77918bed2bf0502) | ROS2 implementation, KILO mode with kinematic/IMU observations, LiDAR updates, loop-closure graph | Preferred candidate for actual joint LiDAR/leg estimation. Requires L1 PointCloud2/time adapter, Go2 SportModeState kinematic/IMU adapter, body-IMU-to-LiDAR extrinsics, clocks, Go2 kinematics and contact thresholds. Not a ready L1/Humble/Jetson integration. |
| [Inria go2_odometry](https://github.com/inria-paris-robotics-lab/go2_odometry) | ROS2 InEKF using LowState IMU, joints and foot sensors | Useful independent proprioceptive baseline with covariance. Requires Pinocchio and their InEKF/description dependencies. Does not by itself correct LiDAR mapping or supply loop closure. |
| [YibinWu leg-odometry](https://github.com/YibinWu/leg-odometry) | IMU/encoder EKF with Go2 rosbag examples | Offline Python baseline, not a ready real-time ROS2 replacement. |

Leg-KILO's old indexed README described ROS1. A direct checkout of current master
at the commit above confirms ROS2 and `sensor_type: KILO`, with a Go1/Velodyne
example, and a loop-closure backend. `usesImu()` applies to LIO; in KILO the body
IMU is consumed with the kinematics message, not blindly fused with another IMU.
The current interface expects a custom HighState. Its existing Go2 dataset config
uses Ouster and LIO, which does not demonstrate stock L1 + Go2 legs support.
The Go1 example contact thresholds (200/220) must not be copied: our current
LowState foot force values were 13–16, without a labelled contact calibration trial.

## Recommended progression

1. Use native state as a separately labelled comparison on a controlled walking
   recording. Capture raw cloud plus full-rate LowState, SportModeState and existing
   common-clock traces. Present bags lack the full-rate kinematic input for replaying
   a leg-assisted estimator on the earlier walk.
2. Adapt the existing Leg-KILO backend in this experiment, retaining Point-LIO as
   the comparison. Prefer its joint measurement model over combining already fused
   output poses. Evaluate short-term body jitter, loop endpoint consistency, map
   overlap, processing time and resource use separately; repeat on the same inputs.
3. Promote only after measured improvement. Loop closure can reduce accumulated
   drift but cannot repair arbitrary incorrect timing, contact inference or extrinsics.

An EKF added only after Point-LIO cannot correct Point-LIO's already built internal
map. Copying a smoother output to RViz would address appearance, not map accuracy.
Current LIO covariance is all zero; it is not a trustworthy uncertainty model for
fusion. [robot_localization's guidance](https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/doc/configuring_robot_localization.rst)
also warns against counting duplicated information from the same measurement source.
No new guards, motion control, camera fusion or factory DDS changes are needed.

## Leg-KILO integration (experimental)

The Go2's current mode=0 SportModeState has zero foot force, position and speed
fields in both high- and low-frequency topics. The adapter therefore computes
encoder forward kinematics from LowState `q/dq` and uses its measured foot forces.
It pairs LowState with source-stamped SportModeState by **identical six-axis IMU
samples**, inside the existing C++ backend. A 15-second native capture matched
4491 of 4492 Sport packets with LowState; the inferred stamp-minus-tick offset
had a 5–95% spread of 109 microseconds (isolated outliers about 8 ms). This is
packet association evidence, not proof of absolute physical acquisition latency.
No arrival-time restamping or independent tick-clock offset is introduced.

Factory world pose, velocity and orientation are not estimator inputs. Body IMU
and raw L1 share the existing fixed clock translation. Raw point-relative times are
preserved in the bag. The backend retains upstream 2 ms time buckets for point
updates; `/utlidar/cloud` is deskewed inside the backend. Optional LowState
recording writes native CDR bytes directly; decoding 500 Hz motor messages in
the Python clock callback was too slow during recording. Pairing/FK therefore
runs in C++, without an extra ROS node. The robot URDF defines FR/FL/RR/RL order,
hip offsets 0.1934/0.0465 m, lateral thigh offset 0.0955 m and 0.213 m leg segments.

Install with `GO2_ROBOT_IP=192.168.8.245 ./setup-lidar3d.sh --legkilo`, then launch
with `./mapping.sh --3d --legkilo`. The default `--3d` remains Point-LIO for comparison.
Both compute on Jetson; the existing relay sends results and robot joints to RViz
on the laptop. No new control or readiness nodes are introduced. Saving, verified
transfer to laptop and removal of transferred Jetson results reuse the existing
session lifecycle. Input recording remains opt-in with `GO2_RECORD=1`.

Upstream is pinned to `4b29f2f7d175dcf661f23f01f77918bed2bf0502`; the patch builds a
headless ROS2 backend. Live ROS outputs now publish the complete optimized backend
map and corrected trajectory at about 1 Hz; the current body pose uses the latest
backend correction. RViz replaces each map instead of accumulating stale scans.
The native saved PCD and backend TUM include accepted loop optimization. The absolute
submap loading path was fixed and short-loop settings were tested on recorded walks.
[Backend-map validation](measurements/legkilo-backend-map-2026-09-09.json).
Enabling loop closure does not establish map accuracy.
Body-to-L1 extrinsics use the earlier measured rigid registration; contact force
thresholds 35/25 are provisional until a labelled stand/walk trial. Contact thresholds require calibration; encoder-derived foot speed still assumes
rigid geometry and a non-slipping stance foot.

Initial replay did not meet acceptance: with no detected foot contacts, exact
per-point update times gave a 1.93 m maximum displacement over 33.1 seconds,
versus Point-LIO's 0.0136 m over its 32.4-second output window on the same bag.
Point-to-plane-only matching did not fix this (2.66 m). The IMU-up/observed-floor
normal discrepancy was 0.33 degrees. These are stability observations, not ground
truth accuracy. The failed initial configuration is not promoted to a working
walking baseline. [Results](measurements/legkilo-initial-replay-2026-09-09.json).

A subsequent contact-bearing static record completed on Jetson: 955 poses over
62.06 s, maximum displacement 0.0221 m. Point-LIO replay gave 0.0277 m; Leg-KILO
replay using the zero native Sport foot fields instead of encoder kinematics gave
0.0546 m. Live and replay used different CPUs, so this is preliminary evidence of
static contact benefit, not a controlled statistical or walking-accuracy claim.
[Integration measurements](measurements/legkilo-baseline-2026-09-09.json).


### Поиск аналогичного дрейфа, 9 сентября 2026

- [CMU #22](https://github.com/jizhang-cmu/autonomy_stack_go2/issues/22):
  тот же внешний симптом — неподвижный робот перемещается в RViz. В прочитанных
  комментариях советуют проверить саму одометрию; подтверждённого исправления нет.
- [CMU #27](https://github.com/jizhang-cmu/autonomy_stack_go2/issues/27):
  пользователи сообщают о противоположном знаке Z акселерометра L1 у разных
  прошивок. Один участник устранил наклон изменением Euler-углов преобразования;
  это частный результат, не проверенная калибровка для нашего робота.
- [Go2 robot #49](https://github.com/Unitree-Go2-Robot/go2_robot/issues/49):
  огромные некорректные ускорения IMU L1; автор сообщил аппаратную причину.
  Наш Leg-KILO использует body IMU, а на записи лёжа ускорения конечные и
  стабильные (примерно [0.85, 0.15, 9.47] м/с²). Это не тот же установленный сбой.
- [MYBOTSHOP](https://forum.mybotshop.de/t/unitree-go2-suppression-of-lidar-data-deviation/1068):
  описаны наклон комнаты и дублирование объектов; конкретного исправления
  этой ошибки backend в обсуждении нет.

Поиск не дал готового подтверждённого патча для нашего запуска Leg-KILO лёжа.
Чужие углы преобразования и аппаратные диагнозы не перенесены в конфигурацию.
Локальный A/B повтор отделяет второй этап обновления по облаку от остальной
цепочки: без него стартовый уход заметно меньше. Это обход проблемного режима,
а не доказанное устранение всех причин дрейфа или новый алгоритм.
