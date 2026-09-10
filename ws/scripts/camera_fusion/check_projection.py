"""Stationary capture diagnostic. Run on Jetson; never changes calibration."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from core import load_calibration, project_visible, rectify


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    args = parser.parse_args()
    folder = args.session
    cfg, k, distortion, _ = load_calibration(folder / 'calibration.json')
    paths = sorted(p for p in folder.glob('*.npz') if p.stem.isdigit())
    if len(paths) < 10:
        raise SystemExit('Need at least ten captured camera/cloud pairs')
    captures = []
    for path in paths:
        with np.load(path, allow_pickle=False) as data:
            captures.append({key: data[key].copy() for key in data.files})
    anchor = len(captures) // 2
    image = cv2.imread(str(paths[anchor].with_suffix('.jpg')))
    rectified, small_k, valid = rectify(image, k, distortion, 960, 540)
    poses = np.array([c['t_world_sensor'] for c in captures])
    translation_diameter = np.linalg.norm(
        poses[:, None, :3, 3] - poses[None, :, :3, 3], axis=-1).max()
    rotations = poses[:, :3, :3]
    rotation_diameter = 0.
    for first in rotations:
        for second in rotations:
            angle = np.arccos(np.clip((np.trace(first.T @ second) - 1) / 2, -1, 1))
            rotation_diameter = max(rotation_diameter, float(angle))
    # Reject accumulation across physically inconsistent poses. These limits
    # measure estimated motion, not calibration quality or exposure timing.
    coherent = translation_diameter < .03 and rotation_diameter < np.deg2rad(1)
    selected = captures if coherent else [captures[anchor]]
    xyz = np.concatenate([c['xyz'] for c in selected])
    _, uv, depth = project_visible(xyz, captures[anchor]['t_world_camera'],
                                    small_k, 960, 540, valid)
    overlay = rectified.copy()
    depth_view = np.zeros_like(rectified)
    # Fixed metric scale: blue is near, red is far; saturation at 5 m.
    colors = cv2.applyColorMap(np.uint8(np.clip(depth / 5, 0, 1) * 255), cv2.COLORMAP_JET).reshape(-1, 3)
    for pixel, color in zip(uv, colors):
        color = tuple(int(c) for c in color)
        cv2.circle(overlay, tuple(pixel), 2, color, -1)
        cv2.circle(depth_view, tuple(pixel), 2, color, -1)
    panels = []
    for picture, label in [(rectified, 'Rectified camera'), (overlay, 'Nominal LiDAR projection - NOT calibrated'),
                           (depth_view, 'LiDAR camera depth: blue 0 m -> red 5 m')]:
        panel = cv2.copyMakeBorder(picture, 35, 0, 0, 0, cv2.BORDER_CONSTANT)
        cv2.putText(panel, label, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 255, 255), 1)
        panels.append(panel)
    cv2.imwrite(str(folder / 'projection-check.jpg'), np.vstack(panels))
    report = {
        'profile': cfg['profile'], 'calibration_verified': False,
        'pairs': len(captures), 'combined_pairs': len(selected),
        'pose_translation_diameter_m': float(translation_diameter),
        'pose_rotation_diameter_deg': float(np.rad2deg(rotation_diameter)),
        'stationary_accumulation_allowed': bool(coherent),
        'projected_pixels': len(uv),
        'image_coverage_fraction': len(uv) / (960 * 540),
        'depth_percentiles_m': np.percentile(depth, [10, 50, 90]).tolist() if len(depth) else [],
        'median_receipt_cloud_difference_ms': float(np.median([
            abs(int(c['cloud_stamp_ns']) - int(c['image_stamp_ns'])) / 1e6 for c in captures])),
        'notes': ['Receipt/cloud stamp difference is NOT camera exposure latency.',
                  'A stationary single viewpoint cannot validate latency or full calibration.',
                  'No calibration parameters have been fitted or changed.'],
    }
    (folder / 'projection-check.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
