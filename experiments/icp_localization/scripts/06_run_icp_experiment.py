#!/usr/bin/env python3
"""Run both minimizers, controlled initialization offsets, and measurable errors."""
import argparse
import csv
import json
import math
import pathlib
import subprocess
import time
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from cloud_io import read_pcd, xyz_array


ROOT = pathlib.Path(__file__).resolve().parents[1]
AXIS = np.array([[0., 1., 0.], [0., 0., 1.], [1., 0., 0.]])  # camera <- rslidar
MAP_ROT = Rotation.from_euler('ZYX', [math.pi / 2, 0, math.pi / 2]).as_matrix()


def wrap(deg):
    return (deg + 180) % 360 - 180


def pose_from_csv(row):
    xyz = np.array([float(row[f'reference_{key}']) for key in 'xyz'])
    quat = np.array([float(row[f'reference_{key}']) for key in ('qx', 'qy', 'qz', 'qw')])
    return xyz, Rotation.from_quat(quat)


def run_one(binary, map_path, scan, config, init_xyz, init_rot, out, aligned=None):
    q = init_rot.as_quat()
    cmd = [str(binary), str(map_path), str(scan), str(config),
           *[str(v) for v in init_xyz], *[str(v) for v in q], str(out)]
    if aligned:
        cmd.append(str(aligned))
    start = time.perf_counter()
    completed = subprocess.run(cmd, text=True, capture_output=True)
    out.parent.mkdir(parents=True, exist_ok=True)
    (out.with_suffix('.log')).write_text(' '.join(cmd) + '\n' + completed.stdout + completed.stderr)
    if completed.returncode:
        return {'failed': True, 'converged': False, 'failure_reason': completed.stderr.strip()[-800:],
                'wall_runtime_s': time.perf_counter() - start}
    stats = json.loads(out.read_text())
    stats['failed'] = False
    stats['converged'] = not stats['iteration_limit_reached']
    stats['wall_runtime_s'] = time.perf_counter() - start
    return stats


def metrics(stats, reference_xyz, reference_rot):
    result_xyz = np.array([stats[key] for key in 'xyz'])
    result_rot = Rotation.from_quat([stats[key] for key in ('qx', 'qy', 'qz', 'qw')])
    delta = result_xyz - reference_xyz
    map_delta = MAP_ROT @ delta
    ref_rpy = Rotation.from_matrix(MAP_ROT @ reference_rot.as_matrix()).as_euler('xyz', degrees=True)
    icp_rpy = Rotation.from_matrix(MAP_ROT @ result_rot.as_matrix()).as_euler('xyz', degrees=True)
    stats.update({
        'translation_error_3d_m': float(np.linalg.norm(delta)),
        'translation_error_xy_m': float(np.linalg.norm(map_delta[:2])),
        'error_x_m': float(map_delta[0]), 'error_y_m': float(map_delta[1]),
        'error_z_m': float(map_delta[2]),
        'roll_error_deg': wrap(icp_rpy[0] - ref_rpy[0]),
        'pitch_error_deg': wrap(icp_rpy[1] - ref_rpy[1]),
        'yaw_error_deg': wrap(icp_rpy[2] - ref_rpy[2]),
        'icp_map_x_m': float((MAP_ROT @ result_xyz)[0]),
        'icp_map_y_m': float((MAP_ROT @ result_xyz)[1]),
        'reference_map_x_m': float((MAP_ROT @ reference_xyz)[0]),
        'reference_map_y_m': float((MAP_ROT @ reference_xyz)[1]),
    })
    stats['abs_yaw_error_deg'] = abs(stats['yaw_error_deg'])
    return result_xyz, result_rot


def independent_fitness(stats, scan_path, tree):
    _, points = read_pcd(scan_path)
    raw = xyz_array(points)
    raw = raw[np.isfinite(raw).all(axis=1)]
    rng = np.linalg.norm(raw, axis=1)
    raw = raw[(rng >= 1.5) & (rng <= 70.0)]
    if len(raw) > 25000:
        raw = raw[::math.ceil(len(raw) / 25000)]
    rot = Rotation.from_quat([stats[key] for key in ('qx', 'qy', 'qz', 'qw')])
    aligned = rot.apply(raw @ AXIS.T) + np.array([stats[key] for key in 'xyz'])
    distances, _ = tree.query(aligned, k=1, workers=2)
    good = distances < 2.5
    stats['fitness_inlier_fraction'] = float(np.mean(good))
    stats['fitness_rmse_m'] = float(np.sqrt(np.mean(distances[good] ** 2))) if good.any() else None


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--metadata', default=ROOT / 'metadata/scans_with_reference_pose.csv', type=pathlib.Path)
    p.add_argument('--map', default=ROOT / 'map/reference_map.pcd', type=pathlib.Path)
    p.add_argument('--binary', default=ROOT / 'build/icp_localize', type=pathlib.Path)
    p.add_argument('--results-dir', default=ROOT / 'results', type=pathlib.Path)
    p.add_argument('--no-perturbations', action='store_true')
    p.add_argument('--snapshot', help='Run only one snapshot for smoke testing')
    args = p.parse_args()
    rows = list(csv.DictReader(args.metadata.open(newline='')))
    if args.snapshot:
        rows = [row for row in rows if row['snapshot_id'] == args.snapshot]
    if not rows:
        raise SystemExit('No matched scans')
    _, map_points = read_pcd(args.map)
    tree = cKDTree(xyz_array(map_points))
    args.results_dir.mkdir(parents=True, exist_ok=True)
    records = []
    for row in rows:
        snapshot = row['snapshot_id']
        if row['match_valid'].lower() != 'true':
            print(snapshot, 'skipped: timestamp mismatch')
            continue
        ref_xyz, ref_rot = pose_from_csv(row)
        scan = pathlib.Path(row['pcd_file'])
        modes = [('baseline', np.zeros(3), 0.0)]
        if not args.no_perturbations:
            modes += [(f'dx_{shift:g}m', np.array([shift, 0., 0.]), 0.)
                      for shift in (0.25, 0.5, 1.0)]
            modes += [(f'heading_{angle:g}deg', np.zeros(3), angle)
                      for angle in (5., 10., 20.)]
        for method in ('point_to_point', 'point_to_plane', 'point_to_point_coarse'):
            if method != 'point_to_point' and not args.no_perturbations:
                method_modes = modes[:1]  # compare minimizers at the same unperturbed pose
            else:
                method_modes = modes
            for mode, shift, heading in method_modes:
                initial_xyz = ref_xyz + shift
                initial_rot = Rotation.from_euler('y', math.radians(heading)) * ref_rot
                path = args.results_dir / 'runs' / method / f'{snapshot}_{mode}.json'
                path.parent.mkdir(parents=True, exist_ok=True)
                aligned = None
                if mode == 'baseline' and snapshot in ('scan_001', 'scan_005', 'scan_010'):
                    folder = args.results_dir / 'aligned_clouds' / method
                    folder.mkdir(parents=True, exist_ok=True)
                    aligned = folder / snapshot
                stats = run_one(args.binary, args.map, scan, ROOT / 'config' / f'{method}.yaml',
                                initial_xyz, initial_rot, path, aligned)
                stats.update(snapshot_id=snapshot, method=method, mode=mode,
                             scan_timestamp_ns=row['timestamp_ns'],
                             pose_timestamp_ns=row['pose_timestamp_ns'],
                             timestamp_delta_ms=row['abs_timestamp_delta_ms'],
                             initial_x=float(initial_xyz[0]), initial_y=float(initial_xyz[1]),
                             initial_z=float(initial_xyz[2]), initial_heading_offset_deg=heading,
                             initial_translation_offset_m=float(np.linalg.norm(shift)),
                             reference_x=float(ref_xyz[0]), reference_y=float(ref_xyz[1]),
                             reference_z=float(ref_xyz[2]),
                             reference_qx=float(ref_rot.as_quat()[0]),
                             reference_qy=float(ref_rot.as_quat()[1]),
                             reference_qz=float(ref_rot.as_quat()[2]),
                             reference_qw=float(ref_rot.as_quat()[3]))
                if not stats['failed']:
                    result_xyz, result_rot = metrics(stats, ref_xyz, ref_rot)
                    independent_fitness(stats, scan, tree)
                    stats['translation_correction_m'] = float(np.linalg.norm(result_xyz-initial_xyz))
                    stats['rotation_correction_deg'] = float(np.degrees((result_rot*initial_rot.inv()).magnitude()))
                records.append(stats)
                print(snapshot, method, mode, 'failed' if stats['failed'] else
                      f"{stats['translation_error_xy_m']:.3f} m, {stats['yaw_error_deg']:.2f} deg, {stats['runtime_s']:.2f} s", flush=True)
    baseline = {(r['snapshot_id'], r['method']): r for r in records if r['mode'] == 'baseline' and not r['failed']}
    for record in records:
        if record['failed']:
            record['same_solution_as_baseline'] = False
            continue
        base = baseline.get((record['snapshot_id'], record['method']))
        if base is None:
            record['same_solution_as_baseline'] = False
            continue
        distance = np.linalg.norm(np.array([record[k] for k in 'xyz'])-np.array([base[k] for k in 'xyz']))
        yaw_difference = abs(wrap(record['yaw_error_deg']-base['yaw_error_deg']))
        record['same_solution_as_baseline'] = bool(distance < 0.5 and yaw_difference < 5.0)
        record['distance_from_baseline_solution_m'] = float(distance)
    fields = sorted({key for record in records for key in record})
    with (args.results_dir / 'icp_results.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(records)
    summary = {}
    for method in ('point_to_point', 'point_to_plane', 'point_to_point_coarse'):
        group = [r for r in records if r['method'] == method and r['mode'] == 'baseline' and not r['failed']]
        item = {'completed_scans': len(group), 'converged_scans': sum(r['converged'] for r in group),
                'total_scans': len(rows)}
        for key in ('translation_error_3d_m', 'translation_error_xy_m', 'yaw_error_deg', 'abs_yaw_error_deg',
                    'runtime_s', 'fitness_rmse_m', 'fitness_inlier_fraction'):
            vals = np.array([r[key] for r in group if r.get(key) is not None], dtype=float)
            if len(vals):
                item[key] = dict(mean=float(vals.mean()), median=float(np.median(vals)),
                                 rmse=float(np.sqrt(np.mean(vals**2))), std=float(vals.std()),
                                 min=float(vals.min()), max=float(vals.max()))
        summary[method] = item
    for mode in [f'dx_{v:g}m' for v in (.25, .5, 1.)] + [f'heading_{v:g}deg' for v in (5., 10., 20.)]:
        group = [r for r in records if r['method'] == 'point_to_point' and r['mode'] == mode]
        summary[mode] = dict(same_solution=sum(r['same_solution_as_baseline'] for r in group), total=len(group))
    (args.results_dir / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print('Wrote', args.results_dir / 'icp_results.csv', args.results_dir / 'summary.json')


if __name__ == '__main__':
    main()
