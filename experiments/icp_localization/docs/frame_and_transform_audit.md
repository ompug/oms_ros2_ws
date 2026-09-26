# Frame and transform audit

Notation: `T_A_B` maps a point expressed in frame B into frame A and gives
the pose of B expressed in A. Composition is `T_A_C = T_A_B T_B_C`.

The raw cloud header is `rslidar`; its points are physically in RoboSense
coordinates. `imageProjection.cpp` consumes that cloud with SensorDataQoS
(best effort), and `featureAssociation.cpp` changes each point from
`(x_r, y_r, z_r)` to `(y_r, z_r, x_r)` before feature odometry and mapping.
This is a coordinate-axis conversion with no translation:

```
                 [0 1 0]
R_camera_rslidar = [0 0 1]
                 [1 0 0]
p_camera = R_camera_rslidar p_rslidar
```

The saved feature clouds (`/laser_cloud_corner_last`, surface, outlier) use
`camera`; `/laser_cloud_surround`, `/laser_cloud_full`, and `/key_pose_origin`
use `camera_init`. `MapOptimization::publishTF()` publishes odometry on
`/aft_mapped_to_init` with `header.frame_id=camera_init` and
`child_frame_id=aft_mapped`, plus a same-valued TF. Its pose is the
post-scan-to-map, graph-optimized pose of LeGO's internal sensor-frame cloud;
`aft_mapped` is that pose's frame name. This is a LeGO baseline, not ground
truth. `transformFusion.cpp` publishes an integrated `camera_init -> camera`
pose by combining laser odometry with the last mapping correction. It need not
be identical to the lower-rate `/aft_mapped_to_init` at arbitrary timestamps.
The recorded `/tf` bag confirms three dynamic edges: `camera_init ->
laser_odom` (2,064 messages), `camera_init -> camera` (2,064), and
`camera_init -> aft_mapped` (412). No `rslidar` or `base_link` edge was
recorded in this replay; reference static transforms were disabled.

The orientation stored in `/aft_mapped_to_init` is obtained by the code's
quaternion conversion in `publishTF()`. Numerically its rotation matrix equals
the map transform in `transformPointCloud()`:

```
R_camera_init_camera = Ry(internal_pitch) Rx(internal_roll) Rz(internal_yaw)
T_camera_init_rslidar = T_camera_init_camera T_camera_rslidar
```

The ICP reading is explicitly converted into `camera` axes before alignment,
so the resulting `T_camera_init_camera` is directly comparable with the
timestamp-matched LeGO `/aft_mapped_to_init` pose. If ICP instead used the raw
`rslidar` points, its result would be `T_camera_init_rslidar` and it would need
right multiplication by `T_rslidar_camera` before comparison. Treating those
poses as interchangeable would create a fixed rotation error.

`run.launch.py` optionally publishes a reference-only `map -> camera_init`
rotation with zero translation and yaw +1.570795, pitch 0, roll +1.570795
radians. This is an explicit visualization convention, not a surveyed map
alignment. The experiment uses exactly this fixed rotation when reporting
horizontal **map XY** and ROS XYZ roll/pitch/yaw metrics. In `camera_init`, the
vertical axis is Y, so a naive camera_init XY distance or yaw around Z would
not be a ground-plane/heading metric. `scripts/06_run_icp_experiment.py` rotates
both poses into the reference `map` convention before those calculations.

The launch also defines `camera -> base_link` with zero translation and
hardcoded rotations, and `base_link -> velodyne` as identity. They are
desktop reference assumptions. The last child is a **legacy name** and is not
the RoboSense frame. Neither defines a measured `base_link -> rslidar` edge.
The launch used here disables these reference transforms. There is no measured
physical mounting translation or orientation for the Scout Mini in this
workspace. Therefore this offline experiment estimates the sensor/internal
pose; it cannot yet claim a calibrated base_link pose or publish a safe live
`map -> odom` transform. A measured `T_base_link_rslidar` is required for that.

Raw bag record time is in 2026 while raw cloud header times are in January
2017. LeGO output headers use the cloud stamps; nearest timestamp matching is
valid within those topics. Simulated `/clock` from `ros2 bag play --clock` uses
bag record time, so RViz/TF lookups at the simulated current time can complain
about stale transforms. The online frame tree must use one consistent clock.
