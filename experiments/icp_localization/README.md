# Offline RoboSense ICP localization experiment

This experiment registers individual Scout Mini RoboSense LiDAR scans against
a frozen LeGO-LOAM map using libpointmatcher. It compares the resulting sensor
poses with timestamp-nearest LeGO-LOAM mapped poses. It does **not** estimate
absolute ground-truth error or command the robot. The map and test scans come
from the same recorded traversal, so this is a **same-run proof of concept**;
the results are likely optimistic compared with a new drive.

## Data and outputs

The untouched source bag is
`/home/ompug/ros2_bag_files/rosbag2_2026_09_11-12_01_42`. Its raw input is
`/rslidar_points` (`sensor_msgs/msg/PointCloud2`, frame `rslidar`, 10 Hz,
x/y/z/intensity). `bags/lego_2026_09_11_full` is a newly recorded SQLite ROS
bag containing segmentation, feature, odometry, keypose, visualization-map,
and explicit full-map topics. `map/reference_map.pcd` is the frozen
`camera_init` map. `map/previous_surround_export.pcd` preserves the earlier
export. `trajectory/aft_mapped_to_init.csv` has timestamped LeGO poses.
`scans/scan_001.pcd` through `scan_010.pcd` are raw snapshots spread over
the recording. Metadata, registration JSON/logs, CSV metrics, plots, and
before/after PCD overlays are under `metadata/` and `results/`.

See [PROJECT_STATUS.md](PROJECT_STATUS.md) for the audit,
[FINAL_REPORT.md](FINAL_REPORT.md) for measurements, and the `docs/` files for
map generation, coordinate transforms, libpointmatcher, ICP, and live plans.

The Git repository includes code, configuration, reports, compact trajectory/
scan metadata, measured results, and plots. Bags, PCDs, third-party dependencies,
build products, and per-trial debug artifacts remain local and are ignored.
A fresh clone needs the original raw bag and the dependency build described
below to regenerate those local artifacts; the included metrics describe the
completed run, not outputs automatically available from every clone.

## Coordinate convention

LeGO feature association changes each raw RoboSense `(x,y,z)` point to
internal camera coordinates `(y,z,x)`. Its map uses `camera_init`; its
`/aft_mapped_to_init` pose is the corresponding internal sensor pose, with
`header.frame_id=camera_init` and `child_frame_id=aft_mapped`. The C++ ICP
program applies this same axis rotation to raw scans before alignment and
returns `T_camera_init_camera`, which can be compared directly with that LeGO
pose. For human-readable ground-plane XY and yaw metrics, both poses are
rotated by the launch file's explicit `map -> camera_init` reference rotation.
The physical `base_link -> rslidar` mount remains unknown, so no calibrated
body pose can be reported. See `docs/frame_and_transform_audit.md`.

The raw bag recording clock is in 2026 while its cloud header stamps are in
2017. LeGO preserves header stamps, and every match uses those stamps. At the
approximately 2 Hz mapping rate, the nearest mapped pose can be 0–250 ms from
a 10 Hz scan; the matching threshold is 300 ms. `metadata/scans_with_reference_pose.csv`
records the exact difference. Bag playback is run at 0.5x with `--clock` to
give the mapping process enough CPU time; the clock mismatch can impair RViz
TF displays but not this offline header-time matching.

## Reproduce

Install/build libnabo and libpointmatcher as described in
`docs/libpointmatcher_installation.md`. On a fresh clone, build the ROS workspace
with its existing dependency/bootstrap workflow; rebuild `lego_loam_sr` after
checking out this branch so `/export_full_map` is available. From the workspace
root, run:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select cloud_msgs lego_loam_sr
bash experiments/icp_localization/run_full_experiment.sh
```

The script uses the existing processed bag, map, trajectory, and scans if
present. For a fresh processed bag, pass a **new output path** as argument two:

```bash
bash experiments/icp_localization/run_full_experiment.sh \
  /home/ompug/ros2_bag_files/rosbag2_2026_09_11-12_01_42 \
  "$PWD/experiments/icp_localization/bags/lego_new_replay"
```

To inspect a map and the live LeGO graph:

```bash
python3 experiments/icp_localization/scripts/inspect_pcd.py \
  experiments/icp_localization/map/reference_map.pcd
bash experiments/icp_localization/scripts/check_lego_pipeline.sh
```

The graph diagnostic is intended while launch and playback are running. The
full-map export service is called after replay by `scripts/01_process_bag.sh`.
The export includes all saved corner, surface, and outlier keyframe clouds at
their final optimized poses and applies a 0.4 m voxel grid. The older
`/laser_cloud_surround` topic is a radius-limited and pose-downsampled
visualization, not the source of the frozen reference.

For an independent revisit bag later, extract its scans into a separate
directory, for example `datasets/run_b/scans`, then match against an appropriate
independent reference trajectory if one exists. The ICP runner accepts
`--metadata`, `--map`, and `--results-dir`, so it does not require replaying
LeGO or replacing the frozen map. If no independent reference poses exist, it
can still report registration fitness and consistency, but not pose error.

## ICP method and outputs

`src/icp_localize.cpp` is a C++ libpointmatcher client. It reads float32 binary
PCDs, removes nonfinite points, applies range and voxel filtering from
`config/`, takes a LeGO initial pose, and outputs the registered pose plus
runtime, filtered point counts, iteration count, and weighted match fraction.
Point-to-point, point-to-plane, and a coarser point-to-point configuration are
run on all 10 scans. The point-to-plane mode estimates map normals from 10
neighbors; the coarse mode checks the downsampling/runtime tradeoff. The runner also tests
point-to-point ICP from controlled +0.25/+0.5/+1 m and +5/+10/+20 degree
heading perturbations on each scan. It judges whether each offset reaches the
same pose as that scan's unperturbed run, within 0.5 m and 5 degrees; this is
a local-capture measure, not proof of absolute correctness.

`results/icp_results.csv` includes all per-run poses, pose errors in map
coordinates, point counts, corrections, timing, inlier fraction, and an
independent 2.5 m-gated nearest-map RMSE. `results/summary.json` contains mean,
median, RMSE, standard deviation, min, and max for key metrics. The plots show
the LeGO trajectory versus ICP samples, translation/yaw errors, runtime,
fitness, capture range, and map overview. `results/aligned_clouds` and
corresponding overlay PNGs show representative scans before and after ICP.

## Interpretation and next step

Agreement with LeGO can be small because the same scans helped create its
map. Registration runtime also includes per-call map preprocessing and KD-tree
construction; an online node would preload and crop the map. The practical
next experiment is a second drive through the same area with this PCD held
fixed, measured RoboSense mounting extrinsics, and a consistent clock. Only
then should a live ICP node be considered, with one owner for each TF edge.
