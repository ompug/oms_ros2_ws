#!/usr/bin/env python3
import argparse
import json
import numpy as np
from cloud_io import read_pcd, xyz_array


def main():
    parser = argparse.ArgumentParser(description='Inspect a PCD without altering it')
    parser.add_argument('pcd')
    args = parser.parse_args()
    header, pts = read_pcd(args.pcd)
    xyz = xyz_array(pts)
    finite = np.isfinite(xyz).all(axis=1)
    good = xyz[finite]
    result = {
        'path': args.pcd, 'point_count': len(pts), 'fields': pts.dtype.names,
        'width': int(header['WIDTH'][0]), 'height': int(header['HEIGHT'][0]),
        'nan_count': int(np.isnan(xyz).any(axis=1).sum()),
        'infinite_count': int(np.isinf(xyz).any(axis=1).sum()),
        'finite_count': int(finite.sum()),
        'min_xyz_m': good.min(axis=0).tolist() if len(good) else None,
        'max_xyz_m': good.max(axis=0).tolist() if len(good) else None,
        'extent_xyz_m': np.ptp(good, axis=0).tolist() if len(good) else None,
        'file_bytes': __import__('os').path.getsize(args.pcd),
    }
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
