# Navigation acceptance preflight — 12 September 2026

Requested scenario: make a map, save it, restart, localize on that same map, and
navigate to a goal. No robot motion was performed in this preflight.

| Stage | Existing 2D path | Current Point-LIO 3D path |
|---|---|---|
| Saved artifact | Existing YAML/PGM/posegraph/data bundle validates | Latest live PCD/trajectory/config archived with SHA256 checks |
| Reload after process restart | Posegraph and dataset deserialize; map_server becomes active | No saved-map input or localization mode in the current launcher/backend patch |
| Localize on fresh live sensors | Not tested here | Blocked by missing saved-map localization |
| Goal, cancel, repeated A–B | Not tested here | Full saved-map scenario cannot proceed |

The 2D test used the installed launch with `goal_nav:=false`, the existing
`live_floor_20260908` map, and isolated ROS domain 90, without robot transport.
It proves loading, not localization against the current room or navigation.
The localization launch/config before the fix match local main `4bab4ab`.

A launch bug was reproduced: upstream slam_toolbox `localization_launch.py`
defaults `use_sim_time` to true and overrides the false value in the supplied
YAML. The live navigation include now explicitly passes `use_sim_time=false`.
On restart, the node reports false; saved posegraph/dataset load and map_server
is active. Seven existing map-bundle tests, Python compilation and diff checks
pass. Live localization and driving still need a subsequent supervised test.

Logs: `ws/log/nav-acceptance/reload.log` and `reload-fixed.log`. An initial
joint-state import failure came from the diagnostic shell not sourcing Unitree
messages; sourcing the installed overlay removed it on repetition. Missing
map/base TF during this isolated run is expected because there are no live sensor
inputs; it is not counted as a demonstrated product failure.

Result: the 3D candidate does not yet satisfy the project's saved-map navigation
workflow. Its verified startup improvement remains useful, but it cannot yet
replace the existing workflow. The next required implementation is persistent
map loading, initial-pose handling and relocalization feeding a consistent
map/odometry transform to Nav2. Rebuilding a fresh map after restart would not
pass the requested test.
