# Repository audit — 27 September 2026

Scope: release readiness of the existing 2D workflow and the accumulated 3D
work, demo editing, branch inventory, test discovery and offline build checks.
This is not a new live robot acceptance test or an exhaustive security audit.

The initial worktree contained 23 modified tracked files and 34 untracked files.
They were preserved in commit `4d371d5` before release edits. Published `main`
(`dd67d83`) was merged into the release candidate; documentation conflicts were
resolved while retaining the 3D quick start and recording details.

## Findings and corrections

| Finding | Correction / disposition |
|---|---|
| The demo used a filename-derived 54-second offset and covered RViz startup while the robot walked. | Aligned camera 66 s with screen 22 s by motion; removed the cover and checked the 37.2-second export. Alignment remains approximate. |
| `unittest discover` silently omitted eight pytest cases. | Runner now uses pytest for both styles; all 130 cases pass. |
| A real-robot URDF geometry test depended on ignored local session files. | It now expands the tracked xacro with deterministic joint positions and runs on a clean checkout. |
| CI did not compile or test `go2_rviz_controls`. | Added an isolated workspace build and explicit CTest invocation that fails when no tests are found. |
| Unitree CI clone followed upstream HEAD. | Pinned the same commit used by the 3D image: `5204e6e098ee53f4bd929bd77eb1d387cd0fa842`. |
| Python package version was 0.3.0 but ROS metadata was 0.2.0. | Both are 0.2.0; removed duplicate `unitree_go` dependency. |
| Closing RViz in `--view` left its sensor launch running. | RViz exit emits launch shutdown, allowing managed-session cleanup. |
| The installed `go2-lidar3d:local` image lacked SciPy. | Validation used `go2-lidar3d:validation`, which has the required dependencies; the Dockerfile installs SciPy and now explicitly installs pytest. Upgrade notes require rebuilding. |
| 3D saved-map localization remains incomplete. | Preserved helper code and experimental work; release notes explicitly exclude integrated relocalization and autonomous navigation acceptance. |

## Branch cleanup

| Branch | Disposition |
|---|---|
| `docs/3d-demo` | PR #1 is squash-merged into `main`. Local and remote branch removed after preserving tag `archive/2026-09-27/3d-demo` (`0eb7574`). |
| `experiment/pointlio-camera-fusion` | Unique calibration/fusion commits retained under pushed tag `archive/2026-09-27/pointlio-camera-fusion` (`bdd8f4e`); local branch removed. No remote branch existed. |
| `experiment/lidar-3d-slam` | Retained until the release is merged: published `main` still links to it. Its local work is included in the release candidate. |
| `main` | Local pointer fast-forwarded to fetched `origin/main`. |
| `release/v0.2.0` | Candidate branch for review and the draft release. |

Both experimental worktree directories are retained at detached archive tags;
their contents were not deleted. Existing baseline/archive tags are unchanged.

## Checks

- `bash ws/scripts/test-regressions.sh go2-lidar3d:validation`: 130 passed;
  one upstream xacro/Python deprecation warning.
- Fresh isolated colcon build: `go2_nav2`, `go2_description`,
  `go2_rviz_controls`; CTest: 1/1 speed-panel integration test passed.
- Shell syntax checks for tracked shell scripts and Python undefined-name checks.
- Demo: H.264, 1920 × 720, 30 fps, 37.2 s, no audio; opening and subsequent
  motion visually checked, final map view retained.

These local checks use the prepared validation image. Remote CI independently
rebuilds the images from the committed Dockerfiles; its result is reported on
the release pull request. No robot commands were sent during this audit.
