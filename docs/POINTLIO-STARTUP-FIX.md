# Point-LIO stationary startup correction — 12 September 2026

The controlled recording exposed more than a metre of false motion during the
initial stationary interval. A denser initial map fixes that failure on replay
without changing IMU axes, measurement weights, adding leg fusion or smoothing
published poses.

## Change and use

The default `go2_nav2/config/pointlio_go2.yaml` now sets `init_map_size: 20000`
instead of 2000. The stock Point-LIO binary already supports this parameter.
`mapping.sh --3d` transfers this YAML to the isolated Jetson runtime on its next
launch; no binary rebuild is needed. The launcher reminds the operator to keep
the robot stationary until the first map appears. Initialization took about
2.4 seconds from the first recorded cloud in this test, versus 0.52 seconds before.
The number of seconds depends on the available points.

```bash
GO2_RECORD=1 ./mapping.sh --3d
```

Stay still until the first 3D map appears, then begin the route. The seed is
accumulated without motion compensation, so starting to walk during collection
can still spoil initialization. No robot movement is commanded by this change.

## Diagnosis

The stock estimator's internal state log starts with near-zero velocity, then
LiDAR updates introduce false velocity and rotation. Raw stationary clouds
separated by 2.5 seconds align within about 1 cm and 0.53 degrees (98% overlap
within 25 cm), while Point-LIO estimates almost 1 m and 18 degrees. Both identity
and the erroneous LIO pose as initial ICP guesses converge near the same result.
This supports an estimator startup failure rather than real robot translation.

Point timestamps in the 20-second prefix are finite, in seconds, and cloud
acquisition intervals do not overlap: gaps are 1.5–7.1 ms. This check does not
prove absolute hardware synchronization, but reveals no gross unit or ordering
fault that could explain the static metre-scale displacement.

Point-LIO builds its initial kd-tree from accumulated downsampled scans before
starting point-wise updates. Increasing that seed produces a consistent reduction
in false startup motion with all other parameters held fixed:

| Seed threshold | First output | Maximum displacement during initial stop |
|---|---:|---:|
| 2,000 points | 0.52 s | 106.1 cm |
| 5,000 points | 0.84 s | 4.44 cm |
| 10,000 points | 1.36 s | 2.77 cm |
| 20,000 points | 2.40 s | 1.62 cm |

The stop is evaluated through eight seconds after the first cloud. Each row uses
its own first output pose, so output start times differ. This measures stationary
stability, not surveyed mapping accuracy. A 20,000-point seed ended that window
5.2 mm from its first pose. The sweep supports insufficient seed geometry as the
practical cause of this failure; it does not establish a universal minimum point
count for every environment.

The alternative `init_map_size=10` still drifts almost a metre. Tightening only
`plane_thr` to 0.02 leaves up to 48 cm of drift. Neither alternative is promoted.
Raw gyro bias remains a separate calibration lead; no guessed bias was applied.

[Measurements](measurements/pointlio-startup-fix-2026-09-12.json) ·
[Startup comparison](measurements/pointlio-startup-fix-2026-09-12.png)

## Full replay and regression checks

Both configurations replayed the entire controlled bag at 1x with the stock
binary, in isolated ROS domains. The full logs contain 4,031 baseline poses and
4,002 corrected poses; the difference is the intentional longer initialization.
The 20-second startup trajectories are bitwise equal to the corresponding full
replay prefixes, including when full runs execute concurrently.

| Full controlled route | Baseline 2,000 | Corrected 20,000 |
|---|---:|---:|
| Median local plane spread | 26.01 mm | 22.17 mm |
| 90th percentile plane spread | 43.73 mm | 38.85 mm |
| Populated plane cells | 802 | 848 |

Plane spread is the square root of the smallest covariance eigenvalue in 0.5 m
cells with at least 30 points, second eigenvalue >0.0025 and first/second ratio
<0.25. This is a geometry diagnostic, not surveyed accuracy; cell coverage changes.

On the earlier 90-second walking recording, median spread changes from 26.79 to
27.08 mm, and p90 from 49.81 to 51.16 mm. That recording does not show a general
geometry improvement. This change specifically addresses the demonstrated
stationary startup failure.

A raw-cloud endpoint check, compensated for the output-frame transform and
nominal LiDAR/IMU lever arm, differs from the baseline's final pose by 98 cm and
19.4 degrees, versus 4.3 cm and 0.59 degrees for the correction. This sparse initial
cloud ICP has 91.5% overlap and 9.6 cm residual RMS and is only a consistency check;
it must not be interpreted as an independent 4.3 cm accuracy measurement.

The corrected PCD and trajectory are saved under
`ws/maps/lidar3d/replay-20260912-startup20000/`; the original recording and map remain
available. Nine mapping-launcher regression tests, shell syntax and diff checks
pass. A fresh live Jetson run has not yet been performed.

Live follow-up: [the next Jetson run confirmed stable startup](POINTLIO-LIVE-VALIDATION.md).
