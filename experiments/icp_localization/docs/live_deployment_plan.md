# Path from offline test to Scout Mini localization

This workspace is not ready to publish a live robot pose. First make a second
bag by revisiting the mapped area, leaving the reference PCD frozen. Reuse the
scan extractor/ICP executable with that bag, and compare with an independent
reference if available. Measure the physical `T_base_link_rslidar` on the
Scout Mini and record its uncertainty. Confirm the RoboSense driver's frame,
scan fields, time source, and 10 Hz cadence on the robot. Fix the bag/header
clock mismatch in any replay used for TF visualization.

An eventual online node should load and downsample the map once; consume
`/rslidar_points` using sensor-data QoS; convert its coordinates to the map's
internal frame; reject nonfinite/range-invalid returns; obtain a prediction
from wheel odometry/IMU or another motion estimator; crop a local submap around
that prediction; run ICP with a maximum compute budget; validate inlier
fraction, residual, correction size, and temporal consistency; then publish a
timestamped sensor pose and covariance/quality status. A stale or rejected
registration should not silently update localization.

The measured offline runtime in `FINAL_REPORT.md` is for ICP compute including
per-call reference preprocessing and KD-tree setup, but excludes process
startup and PCD loading. That is an upper bound on a preloaded-map design
only if local cropping and reusable map indexing actually reduce its cost;
benchmark the real node at 10 Hz on the Scout Mini CPU before integrating it.
Potential optimizations: map cropping/local submaps, one-time map preload and
KD-tree reuse, coarser/multiresolution voxels, source point caps, and motion
prediction. Correctness and reliable convergence take priority over speed.

TF ownership must be decided before deployment. One component should own
`map -> odom`, while a wheel/IMU estimator owns `odom -> base_link`;
the measured static mount publishes `base_link -> rslidar`. An ICP update may
correct `map -> odom` or feed `robot_localization`, then Nav2 can consume the
result. Do not publish simultaneous competing transforms from LeGO,
`robot_localization`, and a new ICP node. The reference launch's `velodyne`
and `camera -> base_link` placeholders must be removed or disabled on the robot.

The offline test is same-run registration, which can overstate accuracy because
test geometry contributes to the map. An independent revisit and recovery
tests (bad initial pose, partial view, moving objects) are the next gate.
