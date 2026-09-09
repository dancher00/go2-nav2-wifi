# Stock Go2 camera: optional input for the 3D mapping branch

Branch `experiment/go2-stock-camera`, based on main `4bab4ab`.
This addition does not alter Point-LIO, Leg-KILO, factory services, robot motion,
TF or the existing low-resolution camera relay. It is a camera input adapter,
not yet a fused visual/LiDAR estimator or a colored-map implementation.

## Observed hardware interface

On this robot the existing Unitree C++ `VideoClient.GetImageSample` reader
returned color JPEGs at **1920x1080**. A short probe received 12 replies at
18.26 replies/s; some consecutive JPEGs were identical. This is a reply rate,
not a measured unique exposure rate. The SDK API returns JPEG bytes without
an exposure timestamp or calibration. The existing generic bridge's 160x120
mono output is a preview setting, not the camera's native resolution.

Official interface: [Unitree SDK2 VideoClient](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/robot/go2/video/video_client.hpp).

## Run on Jetson

Requires the project's existing `go2-humble:local` image and the Unitree C++
camera reader/SDK libraries on Jetson. The launcher creates its own image with
OpenCV, container `go2-stock-camera`, and `~/go2-stock-camera` directory.
It never installs packages into or restarts the other agent's LiDAR container.

```bash
GO2_ROBOT_IP=192.168.8.245 bash stock-camera.sh start
bash stock-camera.sh status
bash stock-camera.sh stop
```

Output DDS domain defaults to **65**, isolated from existing domain-64 LiDAR
visualization. `GO2_CAMERA_DOMAIN=64` selects the same domain as that visualization
when integrating. SDK acquisition stays on domain 0, interface `eth0`, in a
separate child process. JPEG decode and publication run on Jetson; no USB camera
access is required. The existing D435i session is not stopped or changed.

The first 30 seconds of each session are recorded under
`~/go2-stock-camera/data/latest/capture/`, then recording ends while streaming
continues. `frames.jsonl` records receipt time and explicitly null exposure time.
The duration is bounded; disk usage per frame depends on the scene. Normal stop
writes `summary.json`. Existing capture directories are never overwritten.

## Interface for the other agent

| Topic | Type | Semantics |
|---|---|---|
| `/go2_stock_camera/image/compressed` | sensor_msgs/CompressedImage | Original color JPEG, receipt timestamp |
| `/go2_stock_camera/image` | sensor_msgs/Image | Native BGR8, only published when subscribed |
| `/go2_stock_camera/status` | std_msgs/String | JSON: freshness, repeats, dimensions, calibration/fusion flags |

Image QoS is best-effort with a short queue. Prefer compressed transport for
remote visualization; native 1080p raw images can consume excessive Wi-Fi
bandwidth. No CameraInfo or mounting TF is invented. An RViz Image display may
show the image without a calibrated camera pose; a calibrated 3D Camera display
requires the missing calibration.

Identical consecutive JPEGs are conservatively suppressed. Each received frame
is consumed at most once, and frames older than 300 ms are discarded. This fixes
the failure mode where an old buffered image is repeatedly given fresh stamps.
Receipt time remains distinct from physical exposure time: these measures do
not establish synchronization with LiDAR acquisition.

## What remains before actual fusion

1. Calibrate the wide-angle camera intrinsics/distortion at the native resolution.
2. Measure the rigid camera-to-LiDAR/body transform; factory mounting alone does
   not provide a verified numeric transform for this software stack.
3. Obtain or characterize image acquisition timing relative to LiDAR/IMU. A
   request/response timestamp is not enough to claim accurate synchronization.
4. Associate camera keyframes with the backend's pose history. Leg-KILO currently
   publishes globally corrected poses and complete optimized map snapshots;
   preserve node/submap associations so later loop corrections move color data
   with the geometry. Do not accumulate images against a frozen earlier pose.
5. First compare colored geometry against measured LiDAR surfaces on saved
   inputs. Then evaluate visual place recognition or measurement fusion with
   one explicitly chosen owner of global loop corrections.

Until those steps are validated, `fusion_enabled=false` is intentional.
The other branch can cherry-pick the additive camera files without changing its
launchers; start/stop can subsequently be attached to its existing supervisor.

## Atlas / World Labs assessment (2026-09-09)

[Atlas](https://www.worldlabs.ai/blog/atlas) demonstrates reconstruction from
images/video, explicit point clouds and Gaussian splats, and robotics simulation.
The authors explicitly describe filling unobserved areas with generated content.
Proposed use here: an optional post-run visual scene or simulation dataset,
checked against LiDAR geometry. Generated free space is not a measurement for
the robot's planner. No Atlas upload, API call or account request is implemented.

The announcement offers early access for selected partners. The public
[World API model list](https://docs.worldlabs.ai/api/models) lists Marble models,
not Atlas. The reviewed sources do not provide an Atlas deployment procedure
for this Jetson. On-device rendering of a generated splat scene must not be
confused with running the reconstruction model locally. Such a service would
be an optional separate workflow; the requested onboard SLAM remains independent.

## Verified camera input (2026-09-09)

The Jetson SDK returned native 1920x1080 color JPEGs. A 10-second ROS probe
received 127 unique published frames (12.47 Hz between first and last), strictly
increasing receipt timestamps, and zero decode errors. The measurement is saved
in `measurements/stock-camera-live-2026-09-09.json`. Five queue/framing tests pass.
These checks establish camera transport, not calibrated fusion or map accuracy.

## Depth Anything 3 assessment (2026-09-09)

[DA3](https://github.com/ByteDance-Seed/Depth-Anything-3) is a practical candidate
for experimental dense geometry from the stock camera. Small (80M parameters)
and Base support multiple views and known camera poses, but are relative-depth
models. Metric-Large is a separate monocular model. Do not interpret arbitrary
Small output as measured metres. Start with Small on a few recorded keyframes,
with calibrated images and validated LiDAR poses supplying the scale reference.
Measure peak shared memory, latency and held-out LiDAR surface disagreement on
Jetson before enabling continuous operation. DA3 inference has not been run here.

The [GerdsenAI ROS 2 wrapper](https://github.com/GerdsenAI/GerdsenAI-Depth-Anything-3-ROS2-Wrapper)
reports measured TensorRT performance on Orin NX 16GB; its Nano rates are marked
expected. Its Jetson route uses JetPack 6.x / TensorRT 10.3, unlike this robot's
JetPack 5.1.1. Do not run the automatic host installer on the shared robot.
The [RWTH TensorRT node](https://github.com/ika-rwth-aachen/ros2-depth-anything-v3-trt)
publishes metric depth and colored PointCloud2, but documents CUDA 12.8/13,
TensorRT 10.9 and Jazzy test environments. Neither is a verified drop-in here.

[DA3-Streaming](https://github.com/ByteDance-Seed/Depth-Anything-3/blob/main/da3_streaming/README.md)
adds chunk alignment and loop-closure experiments for long sequences. Its
published less-than-12GB GPU budget does not establish suitability for 8GB shared
with LiDAR SLAM. [MapAnything](https://github.com/facebookresearch/map-anything)
is another candidate for pose/calibration-conditioned metric reconstruction;
onboard resource requirements still need measurement.

Proposed sequence: calibrate and color measured LiDAR geometry first; benchmark
DA3-Small dense reconstruction on saved inputs second; integrate only geometry
that passes consistency checks. Keep keyframe associations so backend loop
corrections can update reconstructed surfaces. All reconstruction stays on
Jetson; the laptop receives RViz output. No model installation, host upgrade,
cloud upload or changes to the other agent's LiDAR backend were performed.
