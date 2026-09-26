# Point-LIO with stance-foot observations — experiment, 11 September 2026

The experiment adds foot-velocity observations inside Point-LIO, before LiDAR
updates and insertion into the internal map. It is not an output pose smoother.
The ordinary `./mapping.sh --3d` and its installed executable remain unchanged.
No experimental binary has been deployed to Jetson. Loop closure has not been
implemented. Further algorithm work was paused while investigating the user's suggestion
to check calibration and configuration first.

## Implemented modes

`pointlio_leg_gyro_go2.yaml` retains the original L1 gyro-only output model and
adds foot observations in the L1 IMU frame. The measured body/L1 transform and
the original L1/IMU transform account for the IMU origin's rotational lever arm.
This is the default configuration for the experimental `--pointlio-leg` flag.

`pointlio_leg_go2.yaml` is the alternative full body-IMU input model, with body
acceleration and gyro and the same foot observations. This alternative was not
selected for the default experiment: its map spread was worse in the current
recording. The two IMUs are never simultaneously fed as independent inertial
measurements. Factory pose, world velocity and orientation are never fused.

SportModeState supplies the acquisition timestamp. Its six-axis IMU sample is
matched exactly to LowState, whose joint angles, joint rates and foot forces
supply forward kinematics and contact hysteresis. Unmatched Sport samples keep
body IMU data, but do not create synthetic leg observations. Contact thresholds
35/25 and the rigid transform inherit the earlier Go2 measurements and remain
provisional. Repeated or backward Sport stamps are not reapplied.

The observation enforces zero world velocity of a stance foot. It is applied at
most every 20 ms, at an estimator time no more than 10 ms after the sample.
Foot disagreement inflates covariance; covariance is not divided by contact
count, because feet share IMU and calibration errors. A Mahalanobis gate rejects
large innovations. Updates use Joseph covariance form and SO(3) error reset.
The assumption of a non-slipping contact is still approximate.

## Build and replay

Inside the ROS container, build a **separate** workspace:

```bash
bash /ws/scripts/build-pointlio-leg.sh /ws/pointlio-leg
```

The script applies `pointlio-wifi.patch` to pinned CMU source, then
`pointlio-leg.patch` and the headers in `ws/scripts/pointlio-leg/`. The new
package adds the existing `unitree_go` dependency. Source the resulting install
for offline ROS-domain-isolated replay. Do not source it over a live baseline
session. The leg install includes a SHA256 manifest of the patch and headers.

A future Jetson installation can use the same build script and workspace; only
then does `GO2_RECORD=1 ./mapping.sh --3d --pointlio-leg` become available there.
The launcher checks the separate installation before starting remote mapping.
The flag does not enable driving. Recording includes L1 IMU, LowState and
source-stamped SportModeState for repeatable comparisons.

## Validation and limitations

- Separate C++ build succeeded. Numerical finite differences check FK velocities
  and the rotational, velocity, gyro-bias and output angular-velocity Jacobians.
  A rigid-frame test checks the body/L1 lever arm.
- The configured ROS container passed the full Python regression suite (118 tests
  at the initial implementation). The stock test images lack SciPy/Unitree runtime
  setup; those environment failures must not be described as algorithm failures.
- A 32.4 s no-contact recording produced **bitwise identical** body-model
  trajectories with legs enabled/disabled; no foot corrections were accepted.
- On an older 58.3 s walking prefix, all 897 output timestamps and all 411,060
  saved points were retained. The added observations did not demonstrate an
  improvement in local plane spread.
- On the first 90 s of `run-20260911-181720-bj8yn462`, the L1/leg version retained
  the same 1,361 poses and 725,179 points as the baseline. It accepted 3,997
  observations with complete matching of the observed Sport samples. Median
  local plane spread changed from 26.79 to 26.51 mm: this small change is **not**
  evidence of an accuracy improvement. The body-IMU/leg version was 32.98 mm.
- Local plane spread measures the smallest covariance eigenvalue in populated
  0.5 m cells; cell selection/coverage differs, so it is a diagnostic, not surveyed
  wall accuracy. The prefixes lack a user-confirmed same-position return. Their
  endpoint displacement includes real motion and is not localization error.

[First replay data](measurements/pointlio-leg-first-replay-2026-09-11.json),
[current recording comparison](measurements/pointlio-leg-current-replay-2026-09-11.json),
[map comparison](measurements/pointlio-leg-current-replay-2026-09-11.png).
