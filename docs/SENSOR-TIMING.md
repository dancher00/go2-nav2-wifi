# Native LiDAR acquisition-time alignment

Use `odom_source:=utlidar` (the default). Native `/utlidar/cloud_deskewed`
and `/utlidar/robot_odom` must share the same acquisition clock. On the tested
robot their timestamps lagged laptop ROS time by about 757 seconds, although
the robot and laptop OS clocks were close. This was not a 757-second Wi-Fi delay.

Previously the cloud and odom TF were stamped independently at callback arrival,
and a TF timer stamped the last pose as fresh even when no new odometry arrived.
This lost the relationship between a cloud measurement and the robot pose.

`go2_cloud_stamp_sync` now calibrates one epoch offset from at least one second
and ten increasing odometry samples. The offset is the minimum arrival-minus-
source time, minus a 50 ms history margin. Both streams receive exactly this
same, immutable translation, preserving their original time differences:

| Output | Stamp |
| --- | --- |
| `/utlidar/cloud_deskewed_sync` | Original cloud stamp + shared offset |
| `/utlidar/robot_odom_sync` | Original odometry stamp + shared offset |
| `/odom`, `odom -> base_link` TF | Synced odometry stamp, unchanged |
| `/scan` | Synced cloud stamp, unchanged |

The offset is published latched on `/go2/sensor_time_offset_ns`. Calibration
estimates the epoch offset, not exact one-way network latency. It does not
change system clocks or correct different sensor clock rates. IMU timestamps
are unchanged: this pipeline does not fuse the external IMU.

No odometry TF is emitted before the first accepted measurement or periodically
from an old pose. Repeated, reordered and over-500-ms-old measurements are
dropped. Clouds stop if accepted odometry stops for 500 ms. Clock discontinuities
latch an error and stop forwarding: restart sensors AND SLAM, not only the time
node, to create a new session. Never set `odom_tf_use_current_stamp:=true`.
The optional `sport` odometry path still uses arrival-time stamps and is NOT
covered by this native-LiDAR alignment; do not use it to validate this fix.

The localization `map_odom_relay` also matches each SLAM `/pose` to historical
odom TF at that pose's stamp. It waits for that transform instead of substituting
the latest pose, and holds the resulting map-to-odom correction between updates.
Robot motion continues through odom-to-base TF.

## Read-only check

With the sensor relay and mapping running:

```bash
docker exec go2-nav2-live-test bash -c \
  'source /ws/scripts/setup-robot-wifi.sh &&
   python3 /ws/scripts/check-sensor-time.py --duration 10'
```

The report checks exact timestamp matches at all five pipeline boundaries,
duplicate stamps and rates. A BEST_EFFORT observer can lose packets, so it
reports unmatched stamps and permits up to 5% observer loss. Nonzero unmatched
counts merit a repeat check. `passed: true` validates timestamp propagation,
not map accuracy while walking.

Existing map distortion cannot be undone by fixing stamps. Start a fresh map
with the robot upright for the next walking test. Validate that walls overlap
after slow turns and after returning to the starting point. Floor-level scans,
body roll/pitch, reflective surfaces, poor geometry and native odometry drift
remain separate sources of mapping error.

## Stationary verification, 2026-09-08

The robot was lying down; its sensor-only relay stayed running. No motion bridge
was started. The previous map was preserved as
`ws/maps/before_time_fix_20260908_1510.{pgm,yaml,posegraph,data}` before restarting
mapping. These local map artifacts and test logs are intentionally Git-ignored.

- Humble/Python 3.10 and native robot Foxy/Python 3.8: all 52 tests passed.
  The real-node smoke test runs in a separate ROS domain, with loopback-only DDS.
- `colcon build --packages-select go2_description go2_nav2 --symlink-install`
  succeeded; launch arguments parsed successfully.
- Final 10.023-second live sample: cloud/scan 15.37 Hz, synced odometry and TF
  149.56 Hz. Zero duplicate or unmatched interior timestamps at all five
  boundaries. Locked offset remained `757123326776` ns across repeat checks.
- Median arrival age: scan 98.4 ms, odom TF 69.0 ms; these include the deliberate
  50 ms history margin and are not pure network-latency measurements.
- A separate TF-buffer check resolved all 89 historical scan-time poses without
  error; SLAM published a 202 by 362 cell map at 5 cm resolution. The same check
  observed zero `/cmd_vel` messages.

Local evidence: `ws/log/time_fix_20260908/sensor-time-final.json`,
`mapping-verified.log`, and `foxy-tests-final.log` in that directory. Walking
mapping accuracy and the full localization launch have not been live-tested in
this stationary run; the localization timestamp logic has regression tests.

### Эксперимент 3D LiDAR

В ветке `experiment/lidar-3d-slam`: `./mapping.sh --3d` запускает отдельный
Point-LIO на Jetson (raw L1 + гироскоп, без камеры, ускорений и loop closure);
на ноутбуке — только RViz через Wi-Fi. `--3d --laptop` сохраняет вычисления на ноутбуке.
Установка, измерения и ограничения: [3D LiDAR SLAM](LIDAR-3D-SLAM.md).
Обычный `./mapping.sh` сохраняет 2D-профиль.

### Leg-KILO: пары IMU и энкодеров, 2026-09-09

SportState и LowState связываются по совпадающим сырым float32 полям IMU.
Firmware повторяет ключи IMU; даже одинаковый LowState.tick может сопровождаться
разными энкодерами. Кеш сохраняет первый вариант LowState и допускает повторное
использование для SportState. Это воспроизводимая привязка, но не доказательство
одновременного аппаратного измерения суставов и IMU.

В записи обнаружено отставание времени приёма LowState от SportState до
389 мс в проверенном примере. Это хронология callback/записи, **не измеренная
Wi-Fi end-to-end latency**. Существующая очередь увеличена с 8 до 512 пакетов;
появление более поздней пары позволяет передать предшествующие IMU без
недостающих наблюдений ног. Для них не меняется гистерезис контактов.
Общая временная шкала и watchdog не изменены.

Опциональный `GO2_LEGKILO_INPUT_TRACE` в окружении backend записывает хеши
полей облака, IMU и кинематики плюс `.pairing` журнал приёма. По умолчанию
отключён. Завершённые CSV сравнивает `ws/scripts/compare-legkilo-inputs.py`.
На коротком круге все 1913 кадров и сохранённые позы совпали при 1×/2×.
Остаётся ограничение обработки хвоста записи: последние ожидающие пары
без последующего LowState не сбрасываются в frontend при остановке.
[Подробные результаты](measurements/legkilo-input-pairing-2026-09-09.json).
