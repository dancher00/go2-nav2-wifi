"""Rigid local registration and validity rules, independent of ROS."""
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from .lidar3d_map import voxelize


def pose_matrix(values):
    a = np.asarray(values, dtype=float)
    if a.shape != (7,) or not np.isfinite(a).all() or np.linalg.norm(a[3:]) < 1e-6:
        raise ValueError('Expected finite x y z qx qy qz qw')
    t = np.eye(4)
    t[:3,:3] = Rotation.from_quat(a[3:]).as_matrix()
    t[:3,3] = a[:3]
    return t


def transform(points, matrix):
    return points @ matrix[:3,:3].T + matrix[:3,3]


def delta(a, b):
    return (float(np.linalg.norm(a[:3,3]-b[:3,3])),
            float(Rotation.from_matrix(a[:3,:3].T @ b[:3,:3]).magnitude()))


class Registration:
    def __init__(self, target):
        self.target = voxelize(target, .10)
        self.tree = cKDTree(self.target)

    def align(self, source, seed, acquiring=False):
        source = voxelize(source, .12)
        if len(source) > 5000:
            source = source[np.linspace(0,len(source)-1,5000).astype(int)]
        estimate = np.array(seed, dtype=float, copy=True)
        if len(source) < 300:
            return estimate, {'accepted':False, 'reason':'insufficient points'}
        for radius in ((.8,.4,.2) if acquiring else (.35,.2)):
            for _ in range(12):
                moved = transform(source, estimate)
                distances, ids = self.tree.query(moved)
                selected = distances < radius
                if selected.sum() < 100:
                    return estimate, {'accepted':False,'reason':'insufficient overlap'}
                cutoff = np.percentile(distances[selected], 85)
                selected &= distances <= cutoff
                a, b = moved[selected], self.target[ids[selected]]
                ac, bc = a.mean(0), b.mean(0)
                u, _, vh = np.linalg.svd((a-ac).T @ (b-bc))
                rot = vh.T @ u.T
                if np.linalg.det(rot) < 0:
                    vh[-1] *= -1
                    rot = vh.T @ u.T
                step = np.eye(4)
                step[:3,:3], step[:3,3] = rot, bc-rot@ac
                estimate = step @ estimate
                if delta(step, np.eye(4))[0] < .0002 and delta(step,np.eye(4))[1] < .0002:
                    break
        distances, ids = self.tree.query(transform(source, estimate))
        selected = distances < .25
        overlap = float(selected.mean())
        rmse = float(np.sqrt(np.mean(distances[selected]**2))) if selected.any() else float('inf')
        eig = np.linalg.eigvalsh(np.cov(self.target[ids[selected]].T)) if selected.sum()>3 else np.zeros(3)
        shift, angle = delta(seed, estimate)
        # A single floor plane cannot establish a useful 3D position.
        geometry = eig[0] > .005 and eig[1] > .04
        accepted = bool(overlap >= .70 and rmse <= .12 and geometry and
                        shift <= (1.5 if acquiring else .30) and
                        angle <= np.deg2rad(35 if acquiring else 8))
        return estimate, {'accepted':accepted,'overlap':overlap,'rmse_m':rmse,
                          'shift_m':shift,'angle_rad':angle,'geometry_eigenvalues':eig.tolist(),
                          'reason':'matched' if accepted else 'quality gate rejected'}


class MotionPermit:
    """A recovered localization never resumes a previous movement command."""
    def __init__(self, timeout=.6):
        self.timeout, self.updated, self.valid, self.armed = timeout, -float('inf'), False, False

    def status(self, valid, now):
        if not valid or now-self.updated > self.timeout:
            self.armed = False
        self.valid, self.updated = bool(valid), now

    def ready(self, now):
        if now-self.updated > self.timeout:
            self.armed = False
        return self.valid and now-self.updated <= self.timeout

    def goal(self, now):
        self.armed = self.ready(now)
        return self.armed

    def allowed(self, now):
        return self.ready(now) and self.armed
