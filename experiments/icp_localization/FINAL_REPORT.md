# Final report — offline libpointmatcher ICP, 2026-09-25

## Outcome

The offline experiment completed. A frozen, explicit all-keyframe LeGO map was
exported; 10 distributed raw RoboSense scans were matched to LeGO poses and
registered using point-to-point, point-to-plane, and coarse point-to-point
libpointmatcher configurations. All 30 unperturbed registrations returned a
finite pose. The strict iteration-stop criterion was met by 9/10 standard
point-to-point, 10/10 point-to-plane, and 8/10 coarse point-to-point runs.
Position and heading agreement are promising **for same-run registration**,
but this experiment does not establish independent localization accuracy or
10 Hz readiness.

## Dataset, map, and processing evidence

The untouched raw SQLite bag has 2,065 `/rslidar_points` messages at roughly
10 Hz. Its LiDAR frame is `rslidar`, and its fields are float32 x/y/z/intensity.
One sampled scan has 16 measured vertical-angle peaks from about -14.7° to
+14.9°, consistent with the currently configured 16 rows and -15° to +15°
projection. This checks scan layout, not the physical mounting extrinsic.

The final processed bag is `bags/lego_2026_09_11_full` (SQLite3, 1.4 GiB,
434.220 s recording span, 22,951 messages). It includes 2,065 ground-cloud,
2,059 segmented-cloud, 2,064 laser-odometry, 412 mapped-odometry,
412 keypose, 41 surrounding-map, and one full-map messages, plus feature
clouds and `/tf`; see `logs/processed_bag_info.txt` for all types and counts.
The full-map service response was successful and the LeGO log reports
**355 keyframes, 131,729 exported points**.

The frozen `map/reference_map.pcd` is a 2,107,823-byte binary PCD with
131,729 finite x/y/z/intensity points, 0 NaN/Inf rows, and a 0.4 m output
voxel grid. It is in `camera_init`. Its bounding box in that frame is
x [-49.79, 102.61], y [-2.39, 16.39], z [-67.51, 63.55] m. Its horizontal
map footprint is about 152 × 131 m. `map/reference_map_metadata.txt` records
the generation procedure and source bag. The original 102,952-point
`lego_maps/lego_loam_final_map.pcd` was preserved byte-for-byte as
`map/previous_surround_export.pcd` (matching SHA-256).

The previous PCD is a saved `/laser_cloud_surround` visualization, not an
all-keyframe map. In the final replay, its last message had 101,483 points;
353 keyframes were inside the configured 500 m radius, but only **175** one-meter
pose voxels were selected. The explicit full export includes all **355**
keyframes, including the final two after the last visualization publication.
Its top and side projections (`results/plots/map_overview.png`) have coherent
walls/ground and no obvious folding or major double structures at the plotted
scale. Numerically, no adjacent saved keyframe jumps exceeded 5 m (maximum
0.765 m), and only 18 of 32,933 sampled map points had no neighbor within 2 m.
It is suitable for an initial same-run ICP experiment. This visual/numerical
check cannot certify survey-level map accuracy or rule out subtle drift.

`trajectory/aft_mapped_to_init.csv` has 412 header-stamped poses over 210.8 s.
Ten raw scans were selected across the route, not adjacent frames. Every
nearest LeGO match was within 200.008 ms, below the documented 300 ms limit;
the exact matches are in `metadata/scans_with_reference_pose.csv`. The
recording clock is in 2026 but LiDAR and LeGO header stamps are in January
2017; matching uses the consistent **header** stamps. This mismatch can break
TF/RViz visualization using bag `/clock` and must be fixed for live work.

## Registration and error metrics

Errors below compare to LeGO, **not external ground truth**. XY and yaw use the
explicit `map -> camera_init` rotation from the local launch file, so XY is
horizontal and yaw is around the resulting vertical Z axis. Values are over
the 10 unperturbed scans. “Nearest” uses the requested nearest mapped pose;
“interpolated” is a diagnostic linear-position/spherical-orientation estimate
between its neighboring mapped poses. It is not a new ground-truth source.

| Configuration | Completed / stopped before 50 iterations | Mean nearest XY error | Mean absolute nearest yaw error | Mean interpolated XY error | Mean absolute interpolated yaw error | Mean ICP compute time (max) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Point-to-point, 0.35/0.5 m voxels | 10/10; 9/10 | 0.110 m | 0.727° | 0.041 m | 0.046° | 0.347 s (1.046 s) |
| Point-to-plane, 0.35/0.5 m voxels | 10/10; 10/10 | 0.111 m | 0.793° | 0.037 m | 0.114° | 0.410 s (0.591 s) |
| Coarse point-to-point, 0.6/0.8 m voxels | 10/10; 8/10 | 0.114 m | 0.768° | 0.050 m | 0.089° | 0.220 s (0.491 s) |

For standard point-to-point, nearest-pose horizontal error was median
0.121 m, RMSE 0.131 m, standard deviation 0.070 m, minimum 0.008 m, maximum
0.256 m. Three-dimensional translation error was mean 0.112 m and RMSE
0.132 m. Absolute nearest yaw error was median 0.070°, RMSE 2.000°, maximum
6.308°; the large maximum is dominated by scan 001's timestamp gap during a
turn. Its independent nearest-map fitness averaged 0.183 m RMS across
2.5 m-gated correspondences. The near-100% geometric inlier fraction reflects
the same-run, dense map and is not a reliable success gate by itself.

The finer point-to-point preprocessing gave the best balance of agreement,
capture, and runtime here. Coarser voxels reduced average compute time but
increased interpolated XY error and reached the 50-iteration limit on two
baselines. Point-to-plane had slightly lower interpolated XY error, but worse
interpolated yaw error and substantially greater runtime; it did not clearly
outperform point-to-point. These are just three sensible configurations, not
an exhaustive parameter optimization. Both keep ground points and filter
scan ranges to 1.5–70 m; no unjustified ground removal was applied.

## Initialization capture and failures

Each offset's success means its point-to-point result was within 0.5 m and
5° yaw of that scan's unperturbed ICP result. This tests the local solution
basin, **not absolute correctness**.

| Initial perturbation | Same solution / 10 scans |
| --- | ---: |
| +0.25 m translation | 10 |
| +0.5 m translation | 10 |
| +1.0 m translation | 9 |
| +5° heading | 9 |
| +10° heading | 2 |
| +20° heading | 0 |

Scan 006 is a clean example: standard point-to-point had 0.011 m nearest XY
error, -0.003° yaw error, 0.160 m nearest-map RMS, and stopped after six
iterations. Its overlay is in `results/plots/scan_006_point_to_point_overlay.png`.

Scan 001 is the worst unperturbed nearest-pose comparison: 0.256 m XY and
-6.308° yaw, with point-to-point hitting the 50-iteration limit. Its LeGO
nearest pose is 200 ms away while local rotation is about 30.6°/s; pose
interpolation changes the yaw comparison to -0.184°. Its XY discrepancy
remains about 0.257 m after interpolation, so time offset alone does not
explain that part. It is a low-translation, high-turn-rate segment; its nearest
saved keyframe is about 1.8 s away because keyframes require 0.3 m motion and
there is no rotation threshold. Its saved overlay shows better structural fit
after ICP, but continued refinement/scan motion may matter. The keypose
history showed zero final-versus-initial position revisions in this run, so
loop-closure position shifts do not explain the discrepancy.

The +20° run for scan 004 demonstrates a wrong local solution: 1.613 m XY
error, +16.605° yaw error, 0.679 m nearest-map RMS versus 0.164 m at baseline,
and the 50-iteration limit. Its red wall remains angled against the gray map
in `results/plots/scan_004_heading_20deg_point_to_point_overlay.png`.
Most +10° and all +20° trials failed the same-solution criterion, typically
with larger fitness error and iteration limits. The environment's broad
ground and repeated structures, a local optimizer, and a 2.5 m correspondence
gate make a misleading match possible. Even the failed scan 004 case retained
about 97.6% neighbors within that gate, demonstrating why an online system
needs a stronger innovation/geometry check.

## Leakage, frames, performance, and deployment gate

The map and scans come from the same traversal. Scan 006's exact acquisition
timestamp is a saved LeGO keyframe, so features derived from that scan are in
the map. The other nine sampled stamps are not exact saved keyframe stamps,
but they are surrounded by feature keyframes from the same route. This is
same-run data leakage; the very small interpolated errors can be optimistic.
Use a frozen map from run A and a separate revisit run B for the next test.

The C++ reading point cloud is rotated from RoboSense to LeGO camera axes as
`(x,y,z)_camera=(y,z,x)_rslidar` before ICP. `/aft_mapped_to_init` is the
`camera_init -> aft_mapped` postmapping internal sensor pose, directly
comparable to the ICP `camera_init -> camera` pose after that conversion. The
reference `map -> camera_init` rotation is used only for human-readable XY/yaw
metrics. The physical `base_link -> rslidar` transform is **unknown**;
legacy `velodyne` launch names do not define it. No calibrated Scout Mini
body pose or safe robot TF publication follows from this experiment.

Measurements were made on this Intel Core Ultra 7 155H computer (22 logical
CPUs, approximately 14 GiB RAM), not on the Scout Mini's deployment computer.
No CPU affinity or isolated-load benchmark was imposed.
The final standard point-to-point mean (0.347 s) exceeds the 0.100 s
budget for 10 Hz; maximum was 1.046 s and median was 0.238 s. An earlier
complete pass measured 0.255 s mean and 0.534 s maximum; the final pass's
scan 009 runtime outlier shows wall-time variability under machine load,
not a changed registration result. Even the coarse configuration's 0.220 s
mean exceeds the budget. These timings include per-call map
preprocessing and KD-tree setup, and exclude executable startup and PCD I/O.
Near-10-Hz localization is **not demonstrated** on this computer with the
current algorithm. Preload the map, crop local submaps, reuse search structures,
and benchmark on the target CPU; a wheel/IMU pose predictor is also needed
because +10° heading error was frequently outside the capture basin.

The immediate next step is a second independent bag in the same area, with
consistent timestamps and a measured `T_base_link_rslidar`. Keep this PCD
unchanged, evaluate that bag's scans and recovery behavior, then design an
online node with unambiguous `map -> odom -> base_link -> rslidar` TF ownership.
Do not deploy this offline prototype to the Scout Mini yet.

## Direct answers to the requested technical questions

1. Existing PCD full? No: it is one selected/downsampled surrounding
   visualization. The new `reference_map.pcd` is the full saved-keyframe export.
2. How generated? 500 m radius search, 1 m pose voxel selection, transform
   selected corner/surface/outlier keyframes by optimized poses, 0.4 m cloud
   voxel output.
3. How often? Requested every ten mapping cycles; observed 41 messages for
   412 mapped cycles, about 0.2 Hz bag time and 0.1 Hz at 0.5x playback.
4. How many keyframes? The final surrounding message had 353 in radius but
   175 selected pose voxels; the explicit export used all 355 final keyframes.
5. Movement effect? Radius membership and 1 m voxel selection change as the
   robot moves; later keyframes and pose corrections rebuild the cloud.
6. PCD frame? `camera_init`.
7. `/aft_mapped_to_init` meaning? LeGO postmapping, graph-optimized odometry
   baseline, not external truth.
8. Physical/internal pose? The LeGO internal sensor-frame cloud, called
   `aft_mapped` in the odometry child frame; it is not a calibrated base_link.
9. ICP comparison transform? Convert raw `(x,y,z)` to camera `(y,z,x)`, then
   compare `T_camera_init_camera` to LeGO's pose; apply the explicit map-axis
   rotation to both for horizontal XY and yaw metrics.
10. RoboSense extrinsic correct? Not established; no measured
    `T_base_link_rslidar` exists here.
11. Near-pose ICP convergence? 9/10 point-to-point and 10/10 point-to-plane
    stopped before the 50-iteration limit; all returned finite poses.
12. Perturbation range? +0.5 m was reliable (10/10), +1 m and +5° each 9/10,
    +10° only 2/10, +20° 0/10 under the stated same-solution rule.
13. Translation error? Standard point-to-point nearest mean XY 0.110 m,
    mean 3D 0.112 m; interpolated XY mean 0.041 m.
14. Yaw error? Nearest mean absolute 0.727° and median 0.070°;
    interpolated mean absolute 0.046°.
15. Worst scans? Scan 001's turning/timing and iteration limit; scan 004 at
    +20° initial yaw is a visually wrong local solution.
16. Point-to-plane better? No clear overall benefit: similar translation,
    worse interpolated yaw, and 0.410 s versus 0.347 s mean runtime in the
    final pass (standard point-to-point median 0.238 s).
17. Preprocessing best? The 1.5–70 m range plus 0.35/0.5 m voxels gave the
    best balance among tested configurations; coarse 0.6/0.8 m was faster but
    less robust. Ground was retained.
18. Average runtime? 0.347 s point-to-point, 0.410 s point-to-plane,
    0.220 s coarse point-to-point in the final pass.
19. Near-10-Hz live feasible now? No; even average compute exceeds 100 ms.
20. Before deployment? Independent revisit, measured RoboSense extrinsic,
    corrected clocks, map preload/local crop, reliable initial predictor,
    rejection logic, target-CPU benchmark, and single-owner TF integration.

All per-run values, including roll/pitch/yaw, x/y/z errors, filtered counts,
iteration status, correction size, and fitness are in `results/icp_results.csv`.
`results/summary.json`, `results/timestamp_sensitivity.csv`, and
`results/plots/` provide the aggregate and visual evidence. The scripts and
exact reproduction command are in `README.md`.

Final verification: the master script regenerated all 90 trials, metrics,
plots, and representative aligned clouds with exit status 0. All shell scripts
passed `bash -n`, Python scripts passed `py_compile`, the LeGO package test
report had five checks with no errors/failures, and the official libpointmatcher
unit suite previously passed 92/92 tests. Original bags were not rewritten.
