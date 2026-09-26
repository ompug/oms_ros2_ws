# ICP for this experiment

The frozen LeGO PCD is the **reference** cloud in `camera_init`. Each raw
RoboSense scan is the **reading/source** cloud in `rslidar`; its axes are first
converted to LeGO's internal `camera` frame. ICP seeks a rigid rotation R and
translation t such that `p_reference ≈ R p_reading + t`. A matched LeGO pose
provides the first estimate of R,t. The independent perturbation runs test
how sensitive the result is to that estimate.

Each iteration finds nearest reference points (KD-tree), rejects distant or
low-quality correspondences, estimates a corrective transform, and repeats
until the pose change is small or 50 iterations are used. Point-to-point
minimizes three-dimensional distances between paired points. Point-to-plane
minimizes distance along estimated map-surface normals and can handle smooth
surfaces better, but depends on stable normals. We test both.

The YAML filters remove nonfinite points, keep scan ranges 1.5–70 m,
voxelize readings at 0.35 m and the reference at 0.5 m, reject matches beyond
2.5 m, and retain the closest 80% of remaining matches. These settings reduce
the effect of dense ground returns, sparse/outlying geometry, and unequal map
and scan density. They may still discard valid long-range geometry. For the
point-to-plane configuration, map normals use 10 neighbors. A coarser
point-to-point variant uses 0.6 m reading and 0.8 m map voxels to measure the
speed/accuracy tradeoff. We retain ground
because it helps constrain height and tilt; the trial determines whether that
is adequate.

Convergence of the optimization is not proof of correct localization. A poor
initial guess can settle on a repeated structure or another local minimum;
too little overlap, dynamic objects, partial field of view, and sparse map
coverage can do the same. The comparison with LeGO quantifies *agreement with
that baseline*, not absolute pose accuracy. A separate revisit bag and an
independent truth source are needed for stronger accuracy claims.

The reported independent fitness is nearest-map RMS distance for aligned raw
scan points having a neighbor within 2.5 m, plus their inlier fraction. This
is a geometric fit statistic, not a probability or certified localization
error. The libpointmatcher minimizer's weighted match ratio is logged
separately and should not be equated with that RMS.
