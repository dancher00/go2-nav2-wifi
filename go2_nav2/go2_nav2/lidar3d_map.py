"""Portable, immutable saved-map bundles for level-floor Point-LIO localization."""
from pathlib import Path
import argparse
import hashlib
import json
import os
import re
import tempfile
import numpy as np
import yaml


def voxelize(points, size=0.10):
    points = np.asarray(points, dtype=np.float64)
    points = points[np.isfinite(points).all(axis=1)]
    _, ids = np.unique(np.floor(points / size).astype(np.int64), axis=0, return_index=True)
    return points[ids]


def read_pcd(path):
    """Read the uncompressed binary PCD written by our native Point-LIO exporter."""
    with Path(path).open('rb') as stream:
        header = {}
        for _ in range(100):
            line = stream.readline()
            if not line:
                raise ValueError('Truncated PCD header')
            parts = line.decode('ascii').split()
            if parts and not parts[0].startswith('#'):
                header[parts[0]] = parts[1:]
            if parts and parts[0] == 'DATA':
                break
        if header.get('DATA') != ['binary']:
            raise ValueError('Expected native Point-LIO binary PCD')
        fields = header['FIELDS']
        sizes = list(map(int, header['SIZE']))
        counts = list(map(int, header.get('COUNT', ['1'] * len(fields))))
        offsets = np.cumsum([0] + [s*c for s,c in zip(sizes, counts)])
        for name in ('x', 'y', 'z'):
            i = fields.index(name)
            if (sizes[i], counts[i], header['TYPE'][i]) != (4, 1, 'F'):
                raise ValueError('Expected float32 XYZ')
        dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4']*3,
                          'offsets': [int(offsets[fields.index(n)]) for n in ('x','y','z')],
                          'itemsize': int(offsets[-1])})
        raw = stream.read()
        if len(raw) != int(header['POINTS'][0]) * dtype.itemsize:
            raise ValueError('PCD byte count mismatch')
        a = np.frombuffer(raw, dtype=dtype)
        return np.column_stack([a[n] for n in ('x','y','z')])


def occupancy(points, resolution=0.05):
    # Only observed near-floor cells become free. Unknown space stays unknown;
    # obstacle cells win over floor observations. No inferred free-space filling.
    points = points[(points[:,2] > -0.08) & (points[:,2] < 1.6)]
    if len(points) < 100:
        raise ValueError('Insufficient level-floor map geometry')
    low = np.floor(points[:,:2].min(0) / resolution) * resolution - resolution
    ij = np.floor((points[:,:2] - low) / resolution).astype(int)
    wh = ij.max(0) + 2
    if np.prod(wh) > 16000000:
        raise ValueError('Map extent is too large')
    grid = np.full((wh[1], wh[0]), -1, dtype=np.int8)
    floor = np.abs(points[:,2]) < 0.08
    grid[ij[floor,1], ij[floor,0]] = 0
    obstacles = points[:,2] >= 0.15
    grid[ij[obstacles,1], ij[obstacles,0]] = 100
    return grid, low, resolution


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def load_map(directory):
    root = Path(directory).resolve()
    meta = json.loads((root/'manifest.json').read_text())
    if meta.get('version') != 1 or meta.get('frame') != 'lidar3d_saved_map':
        raise ValueError('Unsupported saved 3D map format')
    for name in ('points.npz', 'occupancy.npz'):
        if digest(root/name) != meta['sha256'][name]:
            raise ValueError(f'Saved map checksum mismatch: {name}')
    with np.load(root/'points.npz', allow_pickle=False) as f:
        points = f['points']
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 1000 or not np.isfinite(points).all():
        raise ValueError('Invalid localization geometry')
    with np.load(root/'occupancy.npz', allow_pickle=False) as f:
        grid, origin, resolution = f['grid'], f['origin'], float(f['resolution'])
    if grid.ndim != 2 or not np.isin(grid, [-1,0,100]).all() or resolution <= 0:
        raise ValueError('Invalid occupancy grid')
    return points, grid, origin, resolution, meta


def save_map(run, destination, mount):
    run, destination = Path(run).resolve(), Path(destination).resolve()
    if destination.exists():
        raise ValueError('Map already exists; use a new name')
    frame = json.loads((run/'output-frame.json').read_text())
    if not frame.get('floor_alignment_applied'):
        raise ValueError('A floor-aligned Point-LIO run is required')
    config = yaml.safe_load((run/'pointlio.yaml').read_text())['/**']['ros__parameters']
    if config['mapping'].get('output_frame') != 'lidar3d_map':
        raise ValueError('Unexpected source frame')
    points = read_pcd(run/'PCD/scans.pcd')
    grid, origin, resolution = occupancy(points)
    sparse = voxelize(points, 0.10).astype(np.float32)
    if len(sparse) < 1000:
        raise ValueError('Too few points for localization')
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.saving-3d-', dir=destination.parent) as stage:
        stage = Path(stage)
        np.savez_compressed(stage/'points.npz', points=sparse)
        np.savez_compressed(stage/'occupancy.npz', grid=grid, origin=origin, resolution=resolution)
        meta = {'version':1, 'frame':'lidar3d_saved_map', 'source_run':run.name,
                'source_pcd_sha256':digest(run/'PCD/scans.pcd'), 'source_output_frame':frame,
                'sensor_from_base':list(mount), 'points':len(sparse),
                'sha256':{n:digest(stage/n) for n in ('points.npz','occupancy.npz')}}
        (stage/'manifest.json').write_text(json.dumps(meta, indent=2)+'\n')
        load_map(stage)
        # mkdir is the no-overwrite reservation; publish manifest last.
        destination.mkdir()
        for name in ('points.npz','occupancy.npz','manifest.json'):
            os.replace(stage/name, destination/name)
    return destination


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run')
    parser.add_argument('destination')
    parser.add_argument('--mount', required=True)
    args = parser.parse_args()
    mount = yaml.safe_load(Path(args.mount).read_text())['go2_odom_tf']['ros__parameters']['sensor_from_base']
    print(save_map(args.run, args.destination, mount))

if __name__ == '__main__':
    main()
