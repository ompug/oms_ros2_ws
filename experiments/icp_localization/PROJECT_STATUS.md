# Project status (audited 2026-09-25)

This is a ROS 2 Humble workspace on Ubuntu 22.04.5, x86_64. The source tree has
`src/LeGO-LOAM` (`lego_loam_sr`) and `src/cloud_msgs`; the installed workspace is
under `install/`. The LeGO-LOAM source is the fishros ROS 2 port based on commit
`088856332b71e4bc4cc0f08cd8282ac370ca444a`, with local workspace fixes.
The original root `README.md` still discusses an earlier Velodyne desktop
example and should not be read as Scout Mini extrinsic calibration.

Experiment computer: Intel Core Ultra 7 155H, 22 logical CPUs, approximately
14 GiB RAM. Offline wall-time measurements are not isolated CPU benchmarks
and do not establish performance on the Scout Mini's deployment computer.

`launch_lego_loam.sh` sources `setup_env.sh` and starts `run.launch.py`. Use
`points_topic:=/rslidar_points rviz:=false use_sim_time:=true
publish_reference_tf:=false` for this offline run. `setup_env.sh` uses ROS domain
42 and localhost discovery by default. It had a missing `_lego_domain_id` line in
the working tree; that one line was restored so environment setup works.

Raw bag: `/home/ompug/ros2_bag_files/rosbag2_2026_09_11-12_01_42`, two SQLite
files totaling 16.8 GiB, 231.324 s overall, 2,065 `/rslidar_points` messages,
and camera streams. It is read only throughout this experiment. A small second
bag `rosbag2_2026_09_18-14_13_15` exists but contains no useful LiDAR route.
The raw cloud uses `rslidar`, fields x/y/z/intensity (float32), roughly 25,000
points per scan. Its observed vertical elevation is -14.74 to +14.97 degrees,
consistent with the configured 16-row -15 to +15 degree projection. The
`loam_config.yaml` still says VLP-16; it describes projection geometry here,
not the physical sensor model. Exact ring calibration and mounting require
independent verification.

The bag's *recording* time is in September 2026, but its LiDAR `header.stamp`
is around Unix time 1483233792 (January 2017). This is a material dataset clock
inconsistency. LeGO preserves the LiDAR header stamps in its outputs, so offline
scan/pose matching uses those stamps. ROS bag `--clock` publishes 2026 record
time; TF/RViz displays using that clock may reject 2017 stamped data. The
pipeline calculations do not depend on RViz TF lookups.

Previously exported `lego_maps/lego_loam_final_map.pcd` is a 1,647,391-byte
binary PCD with 102,952 finite x/y/z/intensity points. Its bounding box in
`camera_init` is x [-49.05, 102.02], y [-2.36, 16.26], z [-67.51, 63.62] m.
`tools/save_lego_map.py` saves only the latest `/laser_cloud_surround` message
on Ctrl-C. That visualization topic is not an exact all-keyframe export.

Important LeGO outputs: `/segmented_cloud`, `/ground_cloud`,
`/laser_cloud_sharp`, `/laser_cloud_flat`, `/laser_cloud_corner_last`,
`/laser_cloud_surf_last`, `/outlier_cloud_last`, `/laser_odom_to_init`,
`/aft_mapped_to_init`, `/key_pose_origin`, `/laser_cloud_surround`. This
experiment adds `/laser_cloud_full` and `/export_full_map` to request a frozen
all-keyframe cloud after replay. See `docs/lego_global_map_behavior.md`.

Observed at 0.5x playback: raw, segmentation, and laser odometry 5 Hz;
map-optimized odometry and key poses 1 Hz; surrounding map about 0.1 Hz.
The first diagnostic processed bag contains 2,062 segmentation, 412 mapped
pose, and 41 surrounding-map messages. Live graph and QoS evidence is in
`logs/pipeline_diagnostics.txt`. The final full-export bag contains 2,065
ground-cloud messages, 2,059 segmented clouds, 412 mapped poses, 41
surrounding maps, and one requested full map. Its frozen PCD has 131,729
finite points from 355 saved keyframes. The measured ICP results are in
`FINAL_REPORT.md`.

Known limits: no measured `base_link` to `rslidar` physical extrinsic; no
independent ground truth; no separate revisit bag; same-run map leakage;
LeGO map quality must be assessed by plots and registration; bag/header clock
mismatch may complicate TF visualization.
