"""Per-link URDF visual envelopes and articulated forward kinematics."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation

NS = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}

def origin(element):
    t = np.eye(4)
    if element is not None:
        t[:3, :3] = Rotation.from_euler('xyz', np.fromstring(element.get('rpy', '0 0 0'), sep=' ')).as_matrix()
        t[:3, 3] = np.fromstring(element.get('xyz', '0 0 0'), sep=' ')
    return t

def mesh_points(path):
    root = ET.parse(path).getroot()
    geometries = {}
    for geo in root.findall('.//c:geometry', NS):
        v = geo.find('c:mesh/c:vertices/c:input[@semantic="POSITION"]', NS)
        source = geo.find('c:mesh/c:source[@id="' + v.get('source')[1:] + '"]', NS)
        geometries['#' + geo.get('id')] = np.fromstring(source.find('c:float_array', NS).text, sep=' ').reshape(-1, 3)
    results = []
    def walk(node, parent):
        transform = parent.copy()
        for child in node:
            tag = child.tag.split('}')[-1]
            if tag == 'matrix':
                transform = transform @ np.fromstring(child.text, sep=' ').reshape(4, 4)
            elif tag in ('translate', 'rotate', 'scale'):
                raise ValueError('Unsupported COLLADA transform: ' + tag)
        for inst in node.findall('c:instance_geometry', NS):
            pts = geometries[inst.get('url')]
            results.append(pts @ transform[:3, :3].T + transform[:3, 3])
        for child in node.findall('c:node', NS):
            walk(child, transform)
    for node in root.findall('.//c:visual_scene/c:node', NS):
        walk(node, np.eye(4))
    unit = root.find('c:asset/c:unit', NS)
    return np.concatenate(results) * float(unit.get('meter', '1'))

class RobotGeometry:
    """Mesh convex hull per visual (not one hull spanning gaps between legs).

    Primitive visuals use their exact box/sphere/cylinder tests. Mesh origins,
    units, scales and joint axes come from the same URDF shown in RViz.
    """
    def __init__(self, xml, resolve_package, padding=0.015):
        if not 0 <= padding <= 0.03:
            raise ValueError('Self-filter padding must be between 0 and 0.03 m')
        self.padding = padding
        root = ET.fromstring(xml)
        self.parts = []
        self.joints = []
        self.required_joints = set()
        for joint in root.findall('joint'):
            kind = joint.get('type')
            if kind not in ('fixed', 'revolute', 'continuous', 'prismatic'):
                raise ValueError('Unsupported joint: ' + kind)
            name = joint.get('name')
            if kind != 'fixed':
                self.required_joints.add(name)
            axis = joint.find('axis')
            self.joints.append((name, kind, joint.find('parent').get('link'),
                                joint.find('child').get('link'), origin(joint.find('origin')),
                                np.fromstring(axis.get('xyz', '1 0 0'), sep=' ') if axis is not None else np.array([1., 0., 0.])))
        for link in root.findall('link'):
            for visual in link.findall('visual'):
                geom = visual.find('geometry')
                shape = list(geom)[0]
                kind = shape.tag
                transform = origin(visual.find('origin'))
                if kind == 'mesh':
                    uri = shape.get('filename')
                    if not uri.startswith('package://'):
                        raise ValueError('Expected package mesh URI: ' + uri)
                    package, relative = uri[len('package://'):].split('/', 1)
                    vertices = mesh_points(Path(resolve_package(package)) / relative)
                    vertices *= np.fromstring(shape.get('scale', '1 1 1'), sep=' ')
                    hull = ConvexHull(vertices)
                    data = (hull.equations, vertices.min(axis=0), vertices.max(axis=0))
                elif kind == 'box':
                    data = np.fromstring(shape.get('size'), sep=' ') / 2
                    if np.max(data) < .001:
                        continue  # Frame markers are not physical robot surfaces.
                elif kind == 'sphere':
                    data = float(shape.get('radius'))
                elif kind == 'cylinder':
                    data = (float(shape.get('radius')), float(shape.get('length')) / 2)
                else:
                    raise ValueError('Unsupported visual: ' + kind)
                self.parts.append((link.get('name'), transform, kind, data))
        if not self.parts:
            raise ValueError('URDF has no filterable geometry')

    def transforms(self, positions):
        missing = self.required_joints - positions.keys()
        if missing:
            raise ValueError('Missing joint positions: ' + ', '.join(sorted(missing)))
        result = {'base_link': np.eye(4)}
        remaining = self.joints.copy()
        while remaining:
            progressed = False
            for joint in remaining[:]:
                name, kind, parent, child, fixed, axis = joint
                if parent not in result:
                    continue
                moving = np.eye(4)
                if kind in ('revolute', 'continuous'):
                    moving[:3, :3] = Rotation.from_rotvec(axis / np.linalg.norm(axis) * positions[name]).as_matrix()
                elif kind == 'prismatic':
                    moving[:3, 3] = axis / np.linalg.norm(axis) * positions[name]
                result[child] = result[parent] @ fixed @ moving
                remaining.remove(joint)
                progressed = True
            if not progressed:
                raise ValueError('URDF tree is not rooted at base_link')
        return result

    def contains(self, points_in_base, positions):
        transforms = self.transforms(positions)
        inside = np.zeros(len(points_in_base), dtype=bool)
        for link, visual, kind, data in self.parts:
            t = transforms[link] @ visual
            points = (points_in_base - t[:3, 3]) @ t[:3, :3]
            if kind == 'mesh':
                # Normalized hull plane equations; padding is a metric offset.
                planes, low, high = data
                candidates = np.flatnonzero(np.all((points >= low - self.padding) &
                                                    (points <= high + self.padding), axis=1) & ~inside)
                test = np.zeros(len(points), dtype=bool)
                if len(candidates):
                    test[candidates] = np.all(points[candidates] @ planes[:, :3].T +
                                              planes[:, 3] <= self.padding, axis=1)
            elif kind == 'box':
                test = np.all(np.abs(points) <= data + self.padding, axis=1)
            elif kind == 'sphere':
                test = np.sum(points * points, axis=1) <= (data + self.padding) ** 2
            else:
                test = ((np.sum(points[:, :2] ** 2, axis=1) <= (data[0] + self.padding) ** 2)
                        & (np.abs(points[:, 2]) <= data[1] + self.padding))
            inside |= test
        return inside

    def shadows(self, points_in_base, positions, sensor_in_base, max_distance=1.0):
        """Reject nearby returns whose sight line intersects articulated body.

        Sensor-containing shapes are excluded from ray tests (e.g. the trunk
        visual hull encloses an embedded lidar); their containment test remains.
        """
        result = np.zeros(len(points_in_base), dtype=bool)
        nearby = np.sum((points_in_base - sensor_in_base) ** 2, axis=1) <= max_distance ** 2
        transforms = self.transforms(positions)
        for link, visual, kind, data in self.parts:
            indices = np.flatnonzero(nearby & ~result)
            if not len(indices):
                break
            t = transforms[link] @ visual
            o = (sensor_in_base - t[:3, 3]) @ t[:3, :3]
            end = (points_in_base[indices] - t[:3, 3]) @ t[:3, :3]
            directions = end - o
            pad = self.padding
            if kind in ('mesh', 'box'):
                if kind == 'mesh':
                    planes, low, high = data
                    # Cheap ray/box test before convex-hull plane clipping.
                    box = np.column_stack([np.vstack([np.eye(3), -np.eye(3)]),
                                            np.r_[-high - pad, low - pad]])
                    candidate = ray_polytope(o, directions, box)
                    if not np.any(candidate):
                        continue
                    planes = planes.copy()
                    planes[:, 3] -= pad
                else:
                    planes = np.column_stack([np.vstack([np.eye(3), -np.eye(3)]),
                                              np.r_[-data - pad, -data - pad]])
                    candidate = np.ones(len(indices), dtype=bool)
                if np.all(planes[:, :3] @ o + planes[:, 3] <= 0):
                    continue
                result[indices[candidate]] |= ray_polytope(o, directions[candidate], planes)
            elif kind == 'sphere':
                radius = data + pad
                if o @ o <= radius ** 2:
                    continue
                lo, hi, valid = ray_quadratic(o, directions, radius)
                result[indices] |= valid & (lo < 1 - 1e-6) & (hi > 0) & (lo <= hi)
            else:
                radius, half = data[0] + pad, data[1] + pad
                if o[:2] @ o[:2] <= radius ** 2 and abs(o[2]) <= half:
                    continue
                lo, hi, valid = ray_quadratic(o[:2], directions[:, :2], radius)
                dz = directions[:, 2]
                moving = np.abs(dz) > 1e-12
                z1 = np.full(len(indices), -np.inf)
                z2 = np.full(len(indices), np.inf)
                a = np.zeros(len(indices)); b = np.zeros(len(indices))
                np.divide(-half - o[2], dz, out=a, where=moving)
                np.divide(half - o[2], dz, out=b, where=moving)
                z1[moving] = np.minimum(a[moving], b[moving])
                z2[moving] = np.maximum(a[moving], b[moving])
                valid &= moving | (abs(o[2]) <= half)
                lo = np.maximum(lo, z1); hi = np.minimum(hi, z2)
                result[indices] |= valid & (lo < 1 - 1e-6) & (hi > 0) & (lo <= hi)
        return result


def ray_polytope(origin, directions, planes):
    """Intersect open sight segments (0,1) with a convex solid."""
    start = planes[:, :3] @ origin + planes[:, 3]
    slopes = directions @ planes[:, :3].T
    parallel = np.abs(slopes) < 1e-12
    valid = ~np.any(parallel & (start > 0), axis=1)
    limits = np.zeros_like(slopes)
    np.divide(-start, slopes, out=limits, where=~parallel)
    lower = np.max(np.where(slopes < -1e-12, limits, -np.inf), axis=1)
    upper = np.min(np.where(slopes > 1e-12, limits, np.inf), axis=1)
    return valid & (lower <= upper) & (lower < 1 - 1e-6) & (upper > 0)


def ray_quadratic(origin, directions, radius):
    aa = np.sum(directions ** 2, axis=1)
    bb = 2 * (directions @ origin)
    cc = origin @ origin - radius ** 2
    disc = bb ** 2 - 4 * aa * cc
    moving = aa > 1e-12
    lo = np.full(len(directions), -np.inf)
    hi = np.full(len(directions), np.inf)
    root = np.sqrt(np.maximum(disc, 0))
    np.divide(-bb - root, 2 * aa, out=lo, where=moving)
    np.divide(-bb + root, 2 * aa, out=hi, where=moving)
    return lo, hi, (moving & (disc >= 0)) | (~moving & (cc <= 0))
