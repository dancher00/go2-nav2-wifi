#!/usr/bin/env python3
"""Project live-pose URDF visual meshes into base_link; preserve model provenance.

Input: expanded URDF, JSON array of named joint samples, package directory.
Uses the mesh scene transforms, not the deliberately simplified collision boxes.
"""
import argparse
import hashlib
import json
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

def project(urdf, joints, package):
    links = {}
    for link in urdf.findall('link'):
        parts = []
        for visual in link.findall('visual'):
            mesh = visual.find('geometry/mesh')
            if mesh is None:
                continue
            name = mesh.get('filename').removeprefix('package://go2_description/')
            pts = mesh_points(package / name)
            pts *= np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
            t = origin(visual.find('origin'))
            parts.append(pts @ t[:3, :3].T + t[:3, 3])
        if parts:
            # Reducing each mesh to its 3D convex hull preserves the final XY hull.
            points = np.concatenate(parts)
            links[link.get('name')] = points[ConvexHull(points).vertices]
    all_points = []
    for sample in joints:
        transforms = {'base_link': np.eye(4)}
        remaining = list(urdf.findall('joint'))
        while remaining:
            progressed = False
            for joint in remaining[:]:
                parent = joint.find('parent').get('link')
                if parent not in transforms:
                    continue
                t = origin(joint.find('origin'))
                if joint.get('type') in ('revolute', 'continuous'):
                    axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                    rot = np.eye(4)
                    rot[:3, :3] = Rotation.from_rotvec(axis * sample[joint.get('name')]).as_matrix()
                    t = t @ rot
                elif joint.get('type') != 'fixed':
                    raise ValueError('Unsupported joint')
                transforms[joint.find('child').get('link')] = transforms[parent] @ t
                remaining.remove(joint)
                progressed = True
            if not progressed:
                raise ValueError('Disconnected URDF')
        for name, points in links.items():
            t = transforms[name]
            all_points.append((points @ t[:3, :3].T + t[:3, 3])[:, :2])
    points = np.concatenate(all_points)
    return points[ConvexHull(points).vertices]

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('urdf', type=Path); p.add_argument('joints', type=Path)
    p.add_argument('package', type=Path); p.add_argument('output', type=Path)
    a = p.parse_args(); samples = json.loads(a.joints.read_text())
    hull = project(ET.parse(a.urdf).getroot(), samples, a.package)
    result = {'source_urdf_sha256': hashlib.sha256(a.urdf.read_bytes()).hexdigest(),
              'joint_sample_count': len(samples), 'frame': 'base_link',
              'bounds_min_xy': hull.min(axis=0).tolist(), 'bounds_max_xy': hull.max(axis=0).tolist(),
              'dimensions_xy': np.ptp(hull, axis=0).tolist(), 'hull_xy': hull.tolist(),
              'limitation': 'Envelope of supplied poses only; not a certified walking swept volume.'}
    a.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k != 'hull_xy'}, indent=2))

if __name__ == '__main__':
    main()
