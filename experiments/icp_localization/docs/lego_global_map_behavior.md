# Local LeGO-LOAM map publisher behavior

The authoritative implementation is `src/LeGO-LOAM/src/mapOptmization.cpp` and
`mapOptimization.h`, with values in `config/loam_config.yaml`.

`MapOptimization::run()` processes each feature-association output. It calls
`saveKeyFramesAndFactor()`, corrects poses after loop closure, publishes
`/aft_mapped_to_init` and `/key_pose_origin`, then requests a global-map
publication on **every tenth mapping cycle** via `_publish_global_signal`.
`publishGlobalMapThread()` consumes that signal. This is cycle based, with no
wall timer and no keyframe-change trigger. At the observed raw 10 Hz and
mapping divider 5, the request cadence is about 0.2 Hz in bag time; at 0.5x
playback it was about 0.1 Hz in wall time. Heavy processing can change wall
cadence. The function immediately returns unless
`pubLaserCloudSurround->get_subscription_count() > 0` and keyposes exist.

`publishGlobalMap()` searches saved keyframe positions around the **current
robot position**, using `mapping.global_map_visualization_search_radius`
(configured 500 m). It voxel-downsamples the selected *pose points* at 1.0 m;
it then truncates each surviving pose point's intensity to a keyframe index.
For each selected index it applies the optimized six-degree pose to that
keyframe's saved **corner**, **surface**, and **outlier** clouds. Those saved
clouds were individually downsampled at 0.2, 0.4, and 0.4 m before saving.
Finally it voxel-downsamples the combined visualization cloud at 0.4 m and
publishes `/laser_cloud_surround` in `camera_init`. The publisher rebuilds
from current optimized poses, so loop-closure corrections can move surfaces.
The latest message may also omit up to nine subsequent mapping cycles.

`saveKeyFramesAndFactor()` saves the first mapping frame and later frames only
when position changed at least **0.3 m** since the last keyframe. It has **no
rotation threshold**. Every mapping cycle can publish an odometry pose even
when no new keyframe was saved. Keyframe position and rotation can be revised
after loop closure.

The first recorded replay produced 41 surrounding-map messages. The final
message used 355 saved keyframes inside the 500 m radius, but their positions
occupied only **179 one-meter voxels** (see `scripts/analyze_surround.py`). The
radius covered the route (farthest keyframe from its current pose was 61.16 m),
yet the visualization map selected roughly 179 of 355 keyframes. It is a
downsampled representation of the route, not the entire saved keyframe set.
The earlier `lego_maps/lego_loam_final_map.pcd` was made from this topic, so it
cannot be certified as a complete all-keyframe map.

The final replay had the same pattern: its last surrounding message contained
101,483 points. At that message time 353 saved keyframes were in radius, but
their positions occupied only 175 one-meter voxels; two later keyframes were
saved before `/export_full_map`. That explicit full export produced 131,729
points from all 355 final keyframes. Exact numbers vary slightly between
replays because concurrent callbacks can shift individual update timing.

For this experiment, `/export_full_map` is a Trigger service added to the
mapping node. After playback it snapshots every saved optimized keyframe pose,
transforms all corner/surface/outlier keyframe clouds, downsamples their union
at 0.4 m, and publishes one `/laser_cloud_full` message in `camera_init`.
This bypasses both the 500 m radius and 1 m pose selection. The recorder
subscribes before service invocation; `scripts/02_export_reference.py` extracts
that message to `map/reference_map.pcd`. The ROS bag and the PCD preserve this
provenance. The 0.4 m output is still a voxelized map, intentionally suited to
ICP; it is not a raw undownsampled aggregation.
