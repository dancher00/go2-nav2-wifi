# go2_description

Unitree Go2 URDF and meshes for RViz / TF.

Used by **go2_nav2** via `robot_description.launch.py` → `robot_state_publisher`.

Frames: `base_link` → `trunk` → `base_footprint`, `utlidar_lidar` (lidar joint in xacro).

Build: `/ws/scripts/build.sh` (together with `go2_nav2`).
