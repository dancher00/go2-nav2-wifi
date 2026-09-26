# Controlled test, 2026-09-12

Follow-up diagnosis and correction: [Point-LIO startup fix](POINTLIO-STARTUP-FIX.md).

Run: `run-20260911-233529-h3i_gtpx`. Stopped on user authorization. Archived to laptop through the SHA256-verified fetch workflow. Sensor bag: 262.24 s, 65,749 IMU, 39,126 factory clock-reference odometry and 4,038 cloud messages. Map: 2,435,507 points.

The live estimator moves 0.972 m in the first three-second window, while factory odometry changes 0.12 mm and raw gyro norm p95 is 0.0292 rad/s. This strongly indicates startup estimator drift. The complete endpoint separation of 1.117 m must not be reported as loop error.

Comparing stationary raw clouds at 52–55 s and 250–253 s yields 99.35% overlap within 0.25 m, 43.3 mm point residual RMS, and a relative pose differing from Point-LIO by 8.6 mm and 0.243 degrees. Both identity and LIO initial guesses converge to the same alignment. This supports relatively stable localization after the route; ICP is not independent ground truth.

An isolated 20-second replay tested CMU launch initialization size 10 versus current 2000. First-three-second displacement remained 0.986 m versus 0.947 m with stock parameters. This change does not fix the startup fault and was not promoted. Replay and live trajectories differ, so these are paired replay comparisons rather than exact reproduction claims.

Stationary L1 gyro X medians are about 0.017 rad/s in multiple windows. This is a calibration lead, not an accepted bias correction: initial state, timestamp handling and observability still need diagnosis. No estimator parameters or robot deployment changed. The stock YAML comment was corrected to state that acceleration is disabled by the backend, not absent from raw measurements.

Raw numeric results: `measurements/pointlio-controlled-2026-09-12.json`.
