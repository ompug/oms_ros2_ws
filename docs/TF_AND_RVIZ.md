# TF and RViz reference model

The desktop launch publishes this reference chain:

```text
map --static--> camera_init --dynamic--> camera --static--> base_link --static--> velodyne
                       |--dynamic--> laser_odom
                       |--dynamic--> aft_mapped
```

Actual static-publisher argument order defines the parent and child. The
historical process names are misleading: `camera_init_to_map` publishes
`map -> camera_init`, `base_link_to_camera` publishes `camera -> base_link`,
and `velodyne_to_base_link` publishes `base_link -> velodyne`.

LeGO-LOAM permutes projected LiDAR coordinates into its camera-style internal
axes. Projection diagnostics now retain the incoming LiDAR frame ID; feature
clouds use `camera`, and mapping outputs use `camera_init`. The zero-offset
`base_link -> velodyne` transform and upstream rotations are dataset reference
assumptions, not measured Scout Mini extrinsics.

RViz uses `map` as its fixed frame and contains displays for raw, segmented,
ground, corner, and surface clouds; integrated odometry; key poses; the global
map; and TF. Raw input uses best-effort subscription reliability so both common
LiDAR publisher QoS modes are compatible. Some diagnostics publish only while
a subscriber is present, so start RViz before playback.

On the robot, launch with `publish_reference_tf:=false` until TF ownership and
measured LiDAR extrinsics are known.

