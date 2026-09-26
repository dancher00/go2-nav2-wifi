# Live validation — 12 September 2026

Run `run-20260911-235721-kv1ccv14` used the stock Point-LIO binary on Jetson with
`init_map_size=20000`, confirmed in the saved YAML. It contains 131.42 seconds of
trajectory, 2,023 poses and 1,268,914 saved points. The completed run remained on
Jetson after shutdown; it was recovered to the laptop using the normal SHA256
archive workflow (12 files, 350,506,236 bytes verified).

During the first 55 seconds, maximum estimated displacement from the first pose
is 2.92 cm. The 95th percentile distance from the median stationary position is
7.05 mm. The previous metre-scale startup failure did not recur.

The stationary median pose before and after the route differs by 6.74 cm and
4.08 degrees. This must not be labelled SLAM error: raw clouds also indicate that
the robot returned to a slightly different pose. Three-second cloud windows at
5–8 and 125–128 seconds align with a roughly 6.8 cm translation and 3.78-degree
rotation. The Point-LIO relative pose differs from that alignment by 1.38 cm and
0.207 degrees after nominal lidar/IMU lever-arm compensation. Both identity and
Point-LIO initial guesses converge to the same alignment, with 98.16% overlap
within 25 cm and 5.63 cm point residual RMS. This supports consistent localization
but is not independent ground-truth accuracy.

The map's local plane-spread median is 22.28 mm and p90 is 40.52 mm in 789 accepted
0.5 m cells. This does not prove absence of every doubled surface, and comparisons
across different routes are not a controlled map-quality test.

No further parameter changes were made. The default startup correction is retained.

[Detailed measurements](measurements/pointlio-live-validation-2026-09-12.json).
