# DA3 experiment on Go2 / Jetson

This branch runs DA3 on the robot's Orin Nano 8GB / JetPack 5.1.1, with the
laptop running RViz only. It does not change the other agent's LiDAR backend.

## What works

- Original stock-camera JPEGs feed a separate DA3 preview via ROS 2 domain 65.
- DA3-Small runs on the existing TensorRT 8.5.2.2 with FP16 acceleration.
- Input: RGB, ImageNet normalization, aspect-preserving center crop to 308x182.
- The TensorRT graph is compared against the original ONNX graph on three saved
  images. Numerical agreement gates deployment; an engine hash detects changes.
- The preview publishes relative depth, a colored point cloud, image comparison
  and freshness status. Its TF tree is separate from the robot/map TF tree.
- A bounded camera recording supports offline multi-view reconstruction on
  Jetson, with estimated poses and intrinsics, PLY export and RViz publication.

**This is not calibrated metric mapping or LiDAR-camera fusion.** The scene has
relative scale. Predicted camera intrinsics do not constitute camera calibration.
Single-frame inference timing is not the speed of multi-view reconstruction.

## Commands from the laptop

First run `stock-camera.sh start` if the separate stock-camera service is absent.
The already installed model and environment on this robot can be used directly:

```bash
bash da3-preview.sh start
bash da3-preview.sh rviz
bash da3-preview.sh record      # up to 120 seconds / 256 MiB, about 1 JPEG/s
bash da3-preview.sh status
bash da3-preview.sh stop        # only our worker and preview container
```

`GO2_SSH_CONTROL_PATH` optionally selects an existing authenticated SSH master.
`GO2_ROBOT_IP`, `GO2_ROBOT_USER`, and `GO2_CAMERA_DOMAIN` are configurable; the
camera service must use that same domain. No password is stored in the scripts.
The model worker exits after 30 minutes; status becomes stale if it stops.
Raw captures stay on Jetson under `~/go2-stock-camera/da3/records/`.

After recording, run on **Jetson**, selecting the actual recording directory and
an unused output directory:

```bash
cd ~/go2-stock-camera/da3
OPENBLAS_NUM_THREADS=1 nice -n 15 venv/bin/python reconstruct.py \
  --model models/model.onnx --images records/RECORDING \
  --output results/RESULT --views 24 --motion-keyframes
```

Then from the laptop:

```bash
bash da3-preview.sh scene RESULT
bash da3-preview.sh rviz-scene
bash da3-preview.sh stop-scene
```

Reconstruction publishes `/go2_da3/reconstruction` and an estimated camera path.
Stop an existing scene publisher before loading another. Closing RViz does not
stop the robot-side services. Preview and scene windows are independent.

## Reproducing the Small model setup on Jetson

Copy `ws/scripts/da3/*.py` into `~/go2-stock-camera/da3/`, then run there on Jetson.
The venv uses host OpenCV and TensorRT bindings; pip changes stay inside the venv.

```bash
python3 -m venv --system-site-packages venv
venv/bin/pip install numpy==1.24.4 onnx==1.14.1 onnxruntime==1.16.3
mkdir -p models
curl -fL --retry 3 -o models/model.onnx https://huggingface.co/onnx-community/depth-anything-v3-small/resolve/0b6a7f3bf5595f9950b91389e0da3a0de130324c/onnx/model.onnx
curl -fL --retry 3 -o models/model.onnx_data https://huggingface.co/onnx-community/depth-anything-v3-small/resolve/0b6a7f3bf5595f9950b91389e0da3a0de130324c/onnx/model.onnx_data
sha256sum models/model.onnx models/model.onnx_data
venv/bin/python prepare_trt8.py models/model.onnx models/model-trt8.onnx
OPENBLAS_NUM_THREADS=1 venv/bin/python freeze_onnx.py models/model-trt8.onnx models/model-fixed.onnx
nice -n 15 /usr/src/tensorrt/bin/trtexec \
  --onnx=models/model-fixed.onnx --saveEngine=models/da3-small-308-fp16.engine \
  --fp16 --workspace=512 --buildOnly
```

Expected SHA256:

- `model.onnx`: `396008798244a074297fd88e450433b1357fc687f534939375c804ded86e7b2a`
- `model.onnx_data`: `802bb24741e67f5bb2b369fc64d40afe11439cc895d676d658d65cfb75c9860f`

The pinned [community ONNX export](https://huggingface.co/onnx-community/depth-anything-v3-small)
provides depth, confidence, extrinsics and intrinsics. Numerical validation here
compares with that original export, not an independently executed official
PyTorch checkpoint. [Official DA3](https://github.com/ByteDance-Seed/Depth-Anything-3)
provides the model architecture and model-family descriptions.

TensorRT 8.5 cannot directly import this graph's LayerNormalization operation.
`prepare_trt8.py` lowers last-axis LayerNorm into equivalent arithmetic. Freezing
the input shape and folding constants removes data-dependent shape paths that
otherwise prevent TensorRT's Einsum optimization. The intermediate graph uses
standard ONNX operations. No JetPack upgrade or TensorRT host replacement occurs.

Validate before live GPU use (each output directory must be new):

```bash
OPENBLAS_NUM_THREADS=1 venv/bin/python benchmark.py --backend ort \
  --model models/model.onnx --images ../data/latest/capture --output results/reference
OPENBLAS_NUM_THREADS=1 venv/bin/python benchmark.py --backend trt \
  --model models/da3-small-308-fp16.engine --images ../data/latest/capture \
  --output results/trt --compare results/reference
venv/bin/python validate_engine.py models/da3-small-308-fp16.engine \
  results/trt/benchmark.json models/trt-validated.json
```

The gate requires less than 1% mean relative depth disagreement and less than 2%
relative L2 disagreement for every exported output, on at least three frames.
This tests conversion, not scene accuracy. The original ONNX path remains the
fallback when a validated GPU engine is absent. Both run on Jetson.

## Calibration and map integration

The SDK JPEG API exposes neither exposure timestamps nor verified calibration.
Third-party SDK files exist on the robot, but `front_camera_1080.yaml` has a
noncanonical camera matrix (bottom row `[1, 0, 1]`, also a nonzero entry below fx).
The 720p file belongs to another resolution and is not a verified calibration of
this camera. Neither is enabled. The lens visibly curves straight image lines.

Before metric integration: obtain valid intrinsics/distortion, validate the
camera-to-LiDAR transform and timing, and use the backend pose/keyframe history.
[Koide's calibration toolbox](https://github.com/koide3/direct_visual_lidar_calibration)
is a candidate for camera-to-LiDAR extrinsics after camera intrinsics are known;
it supports non-repetitive LiDAR and fisheye models. It has not been installed.
Global LiDAR loop corrections must update associated visual keyframes/surfaces.

The current DA3 inference consumes JPEGs only. It does not subscribe to odometry,
write any backend map, or assume that the robot's nominal odometry captures ramps.
