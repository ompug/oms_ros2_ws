# Integration decisions — 2026-10-30

Source snapshot: `experiment/icp-localization` at `2fdd1f2` (active when this task started).
Push base: `origin/main` at `1ccb331`; work branch: `robot/relocalization-integration`.
Basing the push on the experiment branch would include 1681 unrelated changed files,
so the standalone package is committed from a separate clean worktree on main.
An independent process committed advisor/poster deliverables (`3766c46`) to the shared
checkout during this task. That commit and its existing working files are preserved on
`work/preserve-advisor-deliverables-3766c46`; it is not pushed with this integration.
Existing dirty/untracked experiment, deliverable, map, RViz and documentation files
remain outside this change.

1. **Read-only prior work.** Inspected `~/previous_agx_work/lego_pose_bridge.py`,
   `lego_relocalize_node.py`, `ugv_ws/src/config/navigation.yaml`, the Woodward node/edge
   CSVs, and `robosense_ws/helios16_config_host_time.yaml`. Also read the guide, trial
   harness, autonomy monitor, base bringup, navigation/sensor launch files, graph loader,
   planner, follower, safety and LiDAR publisher code. No writes or builds occurred in
   that directory. For integration testing only, selected prior navigation packages and
   config were copied to `/tmp/ugv_prior_test/src` and built there; none of those
   unmodified copies enter Git.
2. **Two requested files are missing.** Searches of the archive and home directory did
   not find `start_wally_lego.sh` or `cyclone_robot.xml`. The guide documents the launcher
   but is not its source. `start_robot_base.sh` contains an inline Cyclone configuration
   (interface `wlx00106034d494`, peer `192.168.12.1`), saved here only as an explicitly
   labeled *template*. Deployment requires the real robot's XML. The integration uses
   the existing navigation launch's public parameters; no absent launcher behavior is
   claimed tested. `robot_env.sh` fails if the selected DDS file does not exist.
3. **ROS packaging.** `ugv_relocalization` is a self-contained ament_python package with
   installed configs, reference map, launch, calibration/coverage commands and tests.
   The map is the experiment's `experiments/icp_localization/map/reference_map.pcd`
   (raw LeGO `camera_init`, 131729 points, about 2.1 MB); it is below 20 MB, so it is
   committed directly. It is not the separate `lego_maps/lego_loam_final_map.pcd`.
   Core modules are copied from our experiment and imports/installation paths adapted.
   Experiment-only runners, plotting, results and source-built registration backends
   are omitted. No drive publisher or TF broadcaster is implemented by either package node.
4. **Portable ICP backend.** Use the experiment's Open3D robust point-to-plane registrar,
   its Tukey config, multistart search, structure/attitude gate, and startup ambiguity
   challengers. This is an explicit deployment choice rather than claiming the
   libpointmatcher experiment winner runs from pip. Libpointmatcher and TEASER require
   separate native builds; they are not needed by this package. Open3D does not expose
   an iteration-cap flag, so convergence is not asserted by a fictitious iteration count.
   Structural and attitude acceptance remain mandatory. T1 measures the resulting
   capture basin; ±1–2 m/±10° success is a test target, not an established robot result.
5. **Frame contract.** `T_graph_refmap` maps *raw reference camera_init* into the OSM
   graph `map`. `refmap_ros` is the reference map expressed in ROS axes using the exact
   legacy permutation `(cam_z, cam_x, cam_y)`. Reference ICP outputs are LiDAR poses;
   graph outputs use measured `T_base_link_rslidar` to obtain base_link poses. The bridge
   conjugates relative LiDAR motion by this extrinsic to compensate the lever arm.
   The graph calibration is planar, obtained from physically surveyed base at A aimed
   toward B, an accepted stationary reference ICP pose and the extrinsic. Saved files
   include map/graph hashes and measurement metadata. Mismatched hashes or extrinsics
   fail closed. No surveyed calibration or measured extrinsic was available locally;
   none has been invented or committed.
6. **Original bridge cannot safely consume delayed ICP unchanged.** It does consume
   `/initialpose` and solve a clean SE(2) pin, but ignores the message stamp and solves
   against `_last_raw`. A fix computed while moving would be applied at the wrong time.
   Ship a modified copy, `ugv_relocalization/bridge.py`, behind `launch_bridge` (false by
   default). It interpolates history at the fix stamp, validates frame/finite poses,
   applies lever-arm compensation, and rejects excessive changes. Its output interface
   stays `/lego/odometry`, `map`, `base_link`, best-effort, no TF. Do not run it with the
   old bridge. A calibrated launch refuses `launch_bridge:=false`. An uncalibrated
   passive launch can coexist with the old stack and never injects a graph pose.
7. **Fallback and motion limits.** Every bridge starts with the old analytic math-pin;
   node/facing bearing comes from actual graph coordinates (A→B is 0 rad), not the
   CSV's uniform -0.498828 yaw column. Default `rough` startup runs fine ICP plus
   verification, seeded from the nominated graph pose when calibrated, or `ref_start`
   otherwise. `multistart` runs the full global search; `math_pin` does no registration.
   Rough startup cannot move >3 m/15° from its prior. Periodic corrections reject
   >0.25 m/2° relative to prediction, at 5 s intervals. The bridge independently enforces
   the same runtime bounds. Its first large re-pin requires two seconds stationary
   within 5 cm/2°; a first correction already within runtime bounds is safe while moving.
   Loss of support or gate rejection keeps odometry; it never triggers an unbounded
   global reinitialization during a mission. Start missions only after an accepted
   bridge status. There is no auto-drive, drive-measured heading or velocity publisher.
8. **Coverage cannot be surveyed without calibration.** The local saved figure is
   `figures/graph_coverage_provisional.png`, with edge CSV alongside it. All edges are
   orange/unknown because no real graph transform exists yet. It is not a valid
   coverage claim. The installed `coverage` command regenerates a calibrated overlay
   and marks any edge having unsupported 0.5 m samples red/odometry-only. Runtime uses
   the same 5 m radius / 30-point local support test around the predicted reference pose.
   Holes remain holes; a global bounding box/convex hull is not used. Rejected registration
   still protects geometrically supported but feature-poor areas. Figures are excluded
   from the push as requested.
9. **Sept 11 cross-check is underdetermined.** Read the original 16.8 GiB bag metadata
   and its first LiDAR message without changing the bag. Topics are LiDAR, camera image,
   camera info and camera metadata. There is no GPS/graph-pose observation. The processed
   reference trajectory begins at its own arbitrary origin; identifying this with A
   would assume the alignment being checked. The first raw LiDAR header is
   1483233792801513433 ns; recording time is 1789142503139997503 ns. The audit report is
   `/tmp/ugv_crosscheck.json`. A supplementary trajectory-to-graph shape search tested
   every named start node with ±2 m translation bounds and 12 initial yaw basins. Using
   103 reference-trajectory samples (~162 m driven), H and D fit almost equally poorly:
   RMSE 2.048 m versus 2.055 m; A is 3.439 m. This is a heuristic assuming the bag drove
   along graph edges, not surveyed evidence. It does not establish a unique node or
   justify assuming A. Reproduce with `bag_crosscheck --trajectory <aft_mapped_to_init.csv>`;
   no independent graph node match can be claimed.
10. **Real inputs and timing.** The old bridge and guide establish `/integrated_to_init`
    as its live input and `/lego/odometry` as navigation state. The file `navigation.yaml`
    still defaults to the older `/liorf/mapping/odometry`; the package's modified copy
    changes that field and the launch explicitly passes `/lego/odometry`. `/odom` is
    never assumed. LeGO `/laser_odom_to_init` and `/aft_mapped_to_init` exist in the
    processed bag and are recorded for diagnostics. Runtime domain is 42 with Cyclone
    and the real robot XML. Host-time LiDAR config is adapted only in its angle-file
    path, leaving `use_lidar_clock: false` and the original IP/ports intact. The driver
    publishes RELIABLE clouds (source inspected), so the scan subscription uses reliable
    depth 1: the initial best-effort replay lost large cloud fragments on this host's
    small UDP receive buffer. LeGO odometry uses a compatible sensor-data subscription.
11. **Replay synchronization.** Raw and processed bags were recorded on different days;
    concatenating their recording times does not synchronize scans with odometry.
    `prepare_replay.py` reads both input SQLite files read-only and writes a small new
    bag sorted by sensor header time. Original serialized messages/headers are retained.
    The local 30 s fixture `/tmp/ugv_september_replay` contains 301 raw scans and 300
    integrated odometry messages (~115 MB), excluded from Git. Replay supplies `/clock`
    and all package/navigation nodes use `use_sim_time`. Interpolation refuses large
    odometry gaps; asynchronous fixes retain their measurement timestamp. The inherited
    predictor's single-sample exact-time indexing bug is fixed and regression tested.
12. **Graph consistency without changing prior code.** The original CSV graph is converted
    into `config/graph.yaml`, including original yaw/length/status and CSV SHA256 values.
    At launch, CSVs are generated in a temporary directory for the old stack's existing
    file arguments; planner and calibration therefore share the same graph. No unmodified
    archive CSV copy is committed. Optional navigation includes the existing installed
    launch and disables auto-plan; goals require an explicit `/goal_node` message.
13. **Verification and scope of evidence.** Unit tests exercise axis/lever-arm calibration,
    invalid transforms, exact graph bearing, odometry interpolation/gaps, local coverage
    holes, delayed bridge fixes, wrong frames and runtime jump rejection. Launch tests
    exercise rosbag2 playback, ICP, graph /initialpose, corrected bridge odometry and
    synthetic calibration capture. The prior planner/follower were also launched from
    the temporary build; A→I produced a dense path/controller messages and replanned
    around `A___A_B_1`. Test drive outputs are remapped to `/test/`, on a loopback-only
    Cyclone domain. This proves software integration, not robot autonomy or real graph
    alignment. Multistart on actual Sept 11 scan_001 independently accepted in 8.35 s
    (same-run map proof of concept). Physical T0–T4 and independent revisit accuracy
    remain for the robot operator.
14. **Host dependency limitation handled locally.** Cyclone RMW was absent and sudo
    requires a password. Official ROS binary packages were downloaded and extracted
    under `/tmp/ugv_dds` (RMW 1.3.5, Cyclone 0.10.5) for testing via local library/prefix
    paths. No system package or robot DDS file was changed. The robot should use its
    normal installed `ros-humble-rmw-cyclonedds-cpp`. Build via the venv's Python running
    `/usr/bin/colcon`, so installed entry points use the interpreter containing Open3D.
    Navigation's missing `networkx` dependency is included in requirements.

## Copied/adapted file origins

All paths below are relative to the workspace unless explicitly prefixed `~/`.

| Destination | Origin | Adaptation |
|---|---|---|
| `ugv_relocalization/core/*` | `experiments/relocalization/relocalization/` same relative module names | Minimal registration/gate modules; installed IO resolution; result-only ICP module |
| `ugv_relocalization/runtime.py` | `experiments/relocalization/nodes/reloc_runtime.py` | Package imports, unused modes removed, single-sample predictor fix |
| `ugv_relocalization/ros_common.py` | `experiments/relocalization/nodes/reloc_ros_common.py` | Package imports, retain message conversions; remove experiment domain policy |
| `ugv_relocalization/bridge.py` | `~/previous_agx_work/lego_pose_bridge.py` | Timestamp history, frame/jump checks, lever arm; original output preserved |
| `config/frames.yaml`, `gate.yaml`, `pipeline.yaml` | `experiments/relocalization/configs/` same names | Installed under package share |
| `config/fine.yaml` | `experiments/relocalization/configs/fine/o3d_p2plane_tukey.yaml` | Selected portable existing backend |
| `config/multistart.yaml` | `experiments/relocalization/configs/global/multistart.yaml` | Installed under package share |
| `config/navigation.yaml` | `~/previous_agx_work/ugv_ws/src/config/navigation.yaml` | `/lego/odometry` input and provenance comment |
| `config/graph.yaml` | `~/previous_agx_work/ugv_ws/src/config/woodward_{nodes,edges}.csv` | YAML interface + source hashes; runtime CSV export |
| `config/helios16_config_host_time.yaml` | `~/previous_agx_work/robosense_ws/helios16_config_host_time.yaml` | Angle path moved under robot driver workspace |
| `config/cyclone_robot.template.xml` | Inline XML in `~/previous_agx_work/start_robot_base.sh` | Explicit template; not the missing original file |
| `maps/reference_map.pcd` | `experiments/icp_localization/map/reference_map.pcd` | Binary map copied unchanged, below 20 MB |

All remaining package files are new integration code, launch, tools, documentation or tests.

## Final validation record

- `colcon build --packages-select ugv_relocalization --symlink-install`: passed.
- Direct pytest: 8 unit cases plus the synthetic replay/calibration launch case passed.
- `RELOC_TEST_MODE=multistart` synthetic replay: all 9 pytest cases passed; startup
  accepted in 10.46 s, then two accepted periodic corrections and bridge updates;
  replay reached the end of the 25 s fixture and calibration capture saved a valid file.
- Colcon/pytest with the real Sept 11 synchronized 30 s slice and prior navigation:
  9 tests, 0 errors, 0 failures. It replayed to the end, accepted startup plus four
  periodic corrections, published five graph fixes and ~298 graph odometry messages,
  produced the A→I dense path/controller stream, and replanned after blocking its first
  edge. Graph transform/extrinsic in this test are explicitly synthetic fixtures.
- The archived follower defaults `use_turn_then_drive=True`, despite retaining pure
  pursuit parameters. This integration preserves that setting and bounds corrections
  independently of the controller choice.
- `extras_require={"test": ["pytest"]}` makes current setuptools/colcon discover pytest;
  deprecated `tests_require` was silently ignored by this host's setuptools.
- Navigation checks run during playback after accepted startup; simulated controller
  timers stop when the bag's `/clock` stops, so issuing a goal after EOF is not a valid
  live-navigation integration test. The final launch test observes the whole replay
  and performs goal/blockage checks concurrently.
- Bags, cross-check JSON, replay logs and coverage PNG/CSV stay local, outside the commit.
- The original navigation nodes log a double-shutdown exception on launch SIGINT after
  assertions pass; their read-only implementation is preserved. Our two nodes exit
  cleanly. The integration test checks running behavior and does not claim to repair
  shutdown handling in the previous stack.
- A separate clean worktree based on `origin/main`, containing only the 57 package files,
  also passed a normal (non-symlink) colcon build and all 9 pytest cases, including
  the full synthetic replay and calibration capture. This checks that no experiment
  source tree, results directory or editable-install artifact is required at runtime.
