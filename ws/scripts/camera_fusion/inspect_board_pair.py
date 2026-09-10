"""Inspect a stationary board capture; plane candidates are not calibration."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from core import load_calibration


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    parser.add_argument('--calibration', required=True, type=Path)
    args = parser.parse_args()
    cv2.setNumThreads(1)
    folder = args.session
    capture = json.loads((folder / 'capture.json').read_text())
    flow = capture.get('image_displacement_px_at_640x360_p50_p90')
    if flow is None or flow[0] > .5 or flow[1] > 2:
        raise SystemExit('Scene moved: stationary board inspection rejected')
    files = sorted(f for f in folder.glob('*.npz') if f.stem.isdigit())
    if len(files) < 20:
        raise SystemExit('Incomplete capture')
    _, k, d, sensor_camera = load_calibration(args.calibration)
    frame = files[len(files) // 2]
    image = cv2.imread(str(frame.with_suffix('.jpg')))
    found, corners = cv2.findChessboardCornersSB(
        cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), (8, 5),
        flags=cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE)
    if not found:
        raise SystemExit('Complete 8x5 board not detected')
    objects = np.zeros((40, 3), np.float32)
    objects[:, :2] = np.mgrid[0:8, 0:5].T.reshape(-1, 2) * .03
    ok, rvec, translation = cv2.solvePnP(objects, corners, k, d)
    if not ok:
        raise SystemExit('Board pose failed')
    rotation = cv2.Rodrigues(rvec)[0]
    projected, _ = cv2.projectPoints(objects, rvec, translation, k, d)
    rms = float(np.sqrt(np.mean(np.sum((projected-corners)**2, axis=2))))
    points = np.concatenate([np.load(f, allow_pickle=False)['xyz'] for f in files])
    _, unique = np.unique(np.floor(points/.01).astype(np.int64), axis=0, return_index=True)
    points = points[unique]
    camera = (points-sensor_camera[:3, 3]) @ sensor_camera[:3, :3]
    board = (camera-translation.ravel()) @ rotation
    # Broad nominal ROI tolerates mounting error. It can include the wall;
    # never interpret this selection alone as a measured board plane.
    selection = ((board[:, 0] > -.18) & (board[:, 0] < .39)
                 & (board[:, 1] > -.18) & (board[:, 1] < .30)
                 & (np.abs(board[:, 2]) < .30))
    nearby = points[selection]
    report = {'frame': frame.name, 'corners': len(corners), 'board_rms_px': rms,
              'camera_board_rotation': rotation.tolist(),
              'camera_board_translation': translation.ravel().tolist(),
              'nearby_unique_lidar_points': len(nearby),
              'calibration_verified': False,
              'warning': 'Nearby plane may be wall. Multiple separated tilted board poses required.'}
    if len(nearby) >= 30:
        rng = np.random.RandomState(42)
        best = np.zeros(len(nearby), bool)
        for _ in range(300):
            a, b, c = nearby[rng.choice(len(nearby), 3, replace=False)]
            normal = np.cross(b-a, c-a)
            length = np.linalg.norm(normal)
            if length < 1e-8:
                continue
            normal /= length
            inliers = np.abs((nearby-a) @ normal) < .012
            if inliers.sum() > best.sum():
                best = inliers
        if best.sum() >= 30:
            plane = nearby[best]
            center = plane.mean(0)
            _, singular, vt = np.linalg.svd(plane-center, full_matrices=False)
            normal = vt[-1]
            report.update(candidate_plane_normal=normal.tolist(),
                          candidate_plane_offset=float(normal @ center),
                          candidate_plane_inliers=int(best.sum()),
                          candidate_plane_rms_m=float(np.sqrt(np.mean(((plane-center) @ normal)**2))))
    cv2.drawChessboardCorners(image, (8, 5), corners, True)
    cv2.imwrite(str(folder/'detected-board.jpg'), image)
    (folder/'board-inspection.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2), flush=True)


if __name__ == '__main__':
    main()
