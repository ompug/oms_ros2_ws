# PROJECT_PLAN.md — ROS 2 Humble + LeGO-LOAM local baseline

## 1. Objective and execution boundary

Establish a reproducible native ROS 2 Humble workspace at `/home/ompug/oms_ros2_ws` that builds, launches, receives compatible LiDAR data, and produces meaningful odometry and mapping outputs.

Robot deployment, SSH, motor commands, and robot-specific tuning are excluded.

**Current status (2026-09-10):** the desktop implementation is built and its
unit, launch, QoS, shutdown, synthetic runtime, TF, and RViz initialization
checks pass. System-wide rosdep initialization awaits one privileged command.
Recorded-data testing is blocked by the official Google Drive download quota.
Robot deployment remains outside this desktop phase.

## 2. Current machine assessment

Inspected September 10, 2026:

| Item | Finding |
|---|---|
| Workspace | Empty directory; no existing project files |
| OS | Ubuntu 22.04.5 LTS, Jammy |
| Architecture | x86_64 / amd64 |
| Kernel | `6.8.0-138-generic` |
| CPU | Intel Core Ultra 7 155H; 22 logical CPUs |
| Memory | Approximately 14 GiB total, 12 GiB available |
| Swap | None |
| Disk | Approximately 83 GiB available |
| Shell | Bash |
| Locale | UTF-8 already configured |
| Python | 3.10.12 |
| ROS | `/opt/ros` absent; `ros2` unavailable |
| ROS environment | `ROS_DISTRO` and overlay variables unset |
| Development tools | Git, GCC/G++, CMake, colcon, rosdep unavailable |
| Libraries | PCL, Eigen, Boost development packages and GTSAM absent |
| GUI | `DISPLAY=:0`, `WAYLAND_DISPLAY=wayland-0`; actual RViz access unverified |
| Apt | Universe enabled; ROS repository absent; `jammy-updates` entries absent |

Available Ubuntu candidates include PCL 1.12.1, Eigen 3.4.0, Boost 1.74 and CMake 3.22.1.

Copy these findings into `docs/ENVIRONMENT.md`, then append installed versions and verification results.

## 3. Repository assessment

Use [fishros/LeGO-LOAM-ROS2](https://github.com/fishros/LeGO-LOAM-ROS2), pinned to:

```text
088856332b71e4bc4cc0f08cd8282ac370ca444a
```

Inspected manifests, CMake, launch, configuration, projection, association, mapping, fusion, shared headers, RViz configuration, and open issues.

The repository contains:

- `cloud_msgs`: generates `CloudInfo`.
- `lego_loam_sr`: one executable containing four ROS nodes.
- In-process channels connecting projection, association and mapping.
- Separate worker threads for association, mapping, global-map publication and loop closure.
- No IMU integration in this port.

Critical findings:

- Projection subscribes to **`/rslidar_points`**, not `/lidar_points`.
- Launch remaps `/lidar_points` to `/velodyne_points`, so the current remap is ineffective.
- Launch defines `use_sim_time` but does not apply it.
- RViz starts unconditionally.
- Several parameter declarations read uninitialized variables.
- Empty clouds reach `front()` and `back()` without checks.
- Invalid projected points lack the intensity sentinel tested by ground removal.
- Ground classification differs from original upstream mathematics.
- Debug clouds publish only when subscribers exist.
- Frames are hard-coded; projection relabels coordinates as `base_link` without transforming them.

Upstream pull request #10 independently identifies the topic mismatch. Issue reports are diagnostic context, not proof that this machine has the same failures.

## 4. Architecture overview

Preserve the existing algorithm structure:

```text
ROS 2 bag / synthetic scan source
  → image_projection
  → feature_association
  → map_optimization

feature_association odometry + map correction
  → transform_fusion
  → integrated odometry and TF
```

Use native Humble packages and the default middleware initially. Test processes use a project-specific ROS domain and localhost discovery.

Do not introduce ROS 1, Docker, navigation, wheel-odometry fusion or an alternative SLAM algorithm.

## 5. Dependency analysis

Prefer **`ros-humble-gtsam`**. The official Jammy amd64 package index currently lists:

```text
4.2.0-3jammy.20260226.005743
```

The [Humble distribution manifest](https://github.com/ros/rosdistro/blob/master/humble/distribution.yaml) also declares GTSAM 4.2.0-3.

The code uses established GTSAM 4.x APIs: `Pose3`, `Rot3`, `Point3`, prior/between factors, diagonal noise, `Values`, nonlinear graphs and ISAM2. Inspection provides no evidence requiring 4.0.0-alpha2. Compilation and runtime tests must establish actual compatibility.

`libgtsam-dev` has no candidate in this machine’s configured Ubuntu repositories. Do not add a PPA while the official Humble package is available.

Correct manifests before rosdep:

- Declare build and runtime ROS dependencies using appropriate `<depend>` entries.
- Include `tf2_geometry_msgs`, `eigen3_cmake_module`, `eigen`, `gtsam`, Boost and PCL dependency keys.
- Declare launch, `launch_ros`, `ament_index_python`, and RViz runtime dependencies.
- Correct `cloud_msgs` build dependency on `std_msgs` and generator declarations.
- Match CMake linkage to directly used dependencies; avoid relying on incidental transitive includes.

If packaged GTSAM demonstrably cannot be used, build pinned stable GTSAM 4.2 beneath `third_party/`, with a project-local install prefix, system Eigen, portable CPU settings and low parallelism. Document the specific package failure before using this fallback.

## 6. Workspace and source control

Use the requested layout with documentation and scripts alongside the colcon workspace. Add:

- `tests/` for regression tests and runtime probes.
- `patches/` for changes against pinned upstream.
- `third_party/` only if a source dependency becomes necessary.
- `.venv/` for isolated dataset tooling.
- Project-local download and cache directories.

Initialize a local Git repository after saving the plan. Vendor the pinned upstream source as ordinary tracked files, preserving its license. Record upstream URL, commit and import procedure; keep subsequent fixes in separate commits and export their patch series.

Ignore generated `build/`, `install/`, `log/`, `logs/`, virtual environments, caches and downloaded datasets. Track `data/README.md` and dataset metadata.

Do not push remotely.

## 7. Installation strategy

Follow the [official Humble Ubuntu installation instructions](https://github.com/ros2/ros2_documentation/blob/humble/source/Installation/Ubuntu-Install-Debs.rst), whose source was accessible when the rendered documentation site denied access.

- [x] Save the plan and create the project directories.
- [x] Record initial package inventory and apt configuration.
- [x] Restore official Jammy updates repositories with a documented, backed-up change.
- [x] Install the official `ros2-apt-source` package, retaining its version and installer underneath the project.
- [x] Refresh apt metadata and simulate installation.
- [x] Resolve the documented systemd/udev prerequisite without accepting removal of desktop or core system packages.
- [x] Install `ros-humble-desktop`, required compiler/build tools, Git, colcon, rosdep and Python venv support.
- [x] Import pinned upstream and repair manifests.
- [ ] Initialize rosdep only if necessary; keep its user cache beneath the project.
- [ ] Run:

```bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
```

- [x] Record every newly installed package and version using before/after inventories.

Do not modify shell startup files or GPU configuration. The HTTPS endpoint for `packages.ros.org` returned a hostname-certificate error during inspection; use the official signed apt configuration, preserving signature verification.

## 8. Build and source-change strategy

Make small, separately documented changes:

1. Normalize the internal subscription to `/lidar_points`; expose its destination through a launch argument.
2. Use sensor-data QoS for LiDAR reception, accepting reliable and best-effort publishers. Preserve internal output QoS unless testing demonstrates a problem. This follows [ROS 2 QoS compatibility rules](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst).
3. Replace uninitialized parameter declarations with explicit defaults matching the YAML; validate scan dimensions, FOV, ground index, scan period and mapping divider.
4. Reject empty, malformed or unusable scans safely; filter nonfinite coordinates and check range before division.
5. Initialize missing projection cells consistently and restore original upstream ground-angle calculation and absolute angular comparison, with regression tests.
6. Skip invalid feature index ranges for sparse scans.
7. Apply simulated time to all four nodes and RViz.
8. Add conditional RViz and reference-static-TF launch controls.
9. Repair any demonstrated dependency/API incompatibility at its source.

Build from an underlay-only shell:

```bash
source /opt/ros/humble/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build \
  --symlink-install \
  --executor sequential \
  --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Capture complete output under `logs/` and preserve failure exit status through `tee`. Reduce concurrency to one if memory exhaustion is confirmed. Do not add arbitrary compatibility flags or disable loop closure to obtain a successful build.

## 9. Local testing strategy

Each test records command, versions, duration, outcome and evidence in `docs/TESTING.md`.

- **ROS:** verify CLI, package discovery, and C++ talker/Python listener message exchange.
- **Build:** confirm both packages, executable discovery, package prefixes, dynamic-library resolution and clean rebuild.
- **Launch:** discover all four nodes, inspect effective parameters and topic endpoints; require no fatal errors.
- **Shutdown:** test Ctrl-C before data, during playback and after playback; verify bounded exit without orphaned workers.
- **Input regressions:** empty/all-invalid clouds, zero range, missing fields, sparse rings and invalid parameters.
- **QoS:** demonstrate reception from reliable and best-effort publishers.
- **Projection:** test known VLP-16 angles and both positive/negative ground slopes around the threshold.
- **Runtime:** subscribe before playback to trigger conditional cloud publication.
- **TF:** verify transforms at input timestamps and detect duplicate authorities or disconnected frames.
- **RViz:** validate actual display initialization and topic rendering, not merely process survival.

Synthetic scans must include ground and varied surfaces. Use them for deterministic plumbing and projection tests; do not present them as recorded-data SLAM validation.

## 10. Dataset strategy

Start with the upstream [Jackal VLP-16 dataset](https://github.com/RobustFieldAutonomyLab/jackal_dataset_20170608). Its linked public folder was readable and lists `2017-06-08-15-49-45_0.bag` and three subsequent segments.

- [ ] Download the first segment into `data/raw/`; record URL, filename, size and SHA-256. **Blocked:** official Google Drive quota.
- [ ] Inspect its connection metadata, scan fields, timestamps, frame and coverage. **Blocked:** no bytes supplied by the host.
- [x] Create `.venv/` using system Python.
- [x] Install pinned `rosbags==0.11.5`, whose published Python requirement includes 3.10; lock resolved dependencies.
- [ ] Convert only `/velodyne_points` into Humble-compatible SQLite3 rosbag2, explicitly selecting compatible writer metadata and ROS 2 Humble types.
- [ ] Verify with `ros2 bag info` before launch.
- [ ] Replay at 1× with `/clock`; exclude recorded `/tf`, `/tf_static`, odometry and IMU.
- [ ] Run the complete segment twice with fresh processes; restart between runs rather than looping timestamps backward.

[Rosbags supports ROS 1/ROS 2 conversion](https://ternaris.gitlab.io/rosbags/topics/convert.html), so no ROS 1 system installation is planned.

If the first segment lacks adequate motion/features, inspect the subsequent segments. If acquisition or conversion fails, record the exact blocker and continue all independent tests.

Runtime acceptance requires advancing timestamps, finite poses and normalized quaternions, nonempty segmented/features/map outputs, multiple keyframes during motion, and a plausible trajectory consistent with the scene. Report input/output counts and dropped-scan evidence. Output existence alone is insufficient; trajectory accuracy remains unquantified without independent ground truth.

## 11. ROS interfaces and LiDAR contract

Add launch arguments:

```text
points_topic:=/velodyne_points
params_file:=<installed VLP-16 configuration>
use_sim_time:=true
rviz:=true
publish_reference_tf:=true
```

Do not change public message definitions.

| Topic | Purpose |
|---|---|
| `/velodyne_points` | Default external PointCloud2 input |
| `/segmented_cloud`, `/ground_cloud` | Projection diagnostics |
| `/laser_cloud_sharp`, `/laser_cloud_less_sharp` | Corner features |
| `/laser_cloud_flat`, `/laser_cloud_less_flat` | Surface features |
| `/laser_odom_to_init` | Feature odometry |
| `/aft_mapped_to_init` | Mapping correction |
| `/integrated_to_init` | Combined odometry |
| `/key_pose_origin` | Keyframe positions |
| `/laser_cloud_surround` | Global-map visualization |

Input assumptions:

- One full rotating scan per message, approximately 10 Hz.
- VLP-16 baseline: 16 evenly spaced channels, −15° to +15°, 1800 horizontal bins.
- Metres; conventional LiDAR axes, with the code later permuting coordinates into camera-style axes.
- Finite float32 XYZ; baseline includes float32 intensity for PCL conversion.
- Original intensity is overwritten internally; reflectivity is not used by the algorithm.
- `ring` and per-point time fields are not consumed.
- Organized PointCloud2 is not required, but scan ordering matters because the first and last points determine sweep orientation.
- Preserve scan timestamps and use a consistent clock.
- A frame-ID change alone does not transform point coordinates.

## 12. TF and RViz strategy

Preserve the reference local chain and exact upstream rotations:

```text
map → camera_init → camera → base_link → velodyne
```

Also inspect `camera_init → laser_odom` and `camera_init → aft_mapped`.

The static publisher names are misleading; document parent/child relationships from their actual arguments.

Use `map` as the reference RViz fixed frame. Include raw, segmented, ground and feature clouds, integrated odometry, key poses, map and TF. This port has no confirmed native `nav_msgs/Path` trajectory output; visualize odometry history and key poses.

The zero-offset `base_link → velodyne` transform and projection’s frame relabeling are local reference assumptions. They must not become robot extrinsics.

## 13. Configuration and known risks

Preserve VLP-16 defaults:

- Scan period 0.1 s; ground index 7; mounting angle 0°.
- Segmentation: 60°, minimum five points across three lines.
- Feature thresholds 0.1; nearest-feature search distance 5.
- Mapping divider five.
- Loop closure enabled; history radius 7 m, history count 25, fitness threshold 0.3.
- Surrounding search radius/count 50; global visualization radius 500 m.

Document parameter units, affected nodes, derived angular resolutions and hard-coded voxel/keyframe thresholds.

Nonuniform vertical channels require a different projection model; changing only channel count/FOV is insufficient. The original upstream ring-based option is absent from this port. Solid-state sensors are not assumed compatible.

Other risks include dropped scans through unbuffered channels, blocked shutdown, feature starvation producing uninformative odometry, and unverified loop-closure behavior on a short bag. Track each separately.

## 14. Scout Mini future integration

Create `docs/SCOUT_MINI_PORTING.md` with explicit TBDs:

- Onboard architecture, OS, ROS distribution, storage and available memory.
- Exact Scout ROS 2 driver/version, verified `/cmd_vel` interface, wheel-odometry topic and TF ownership.
- URDF, `robot_state_publisher`, base frames.
- LiDAR manufacturer/model, driver, channels, scan rate, angular layout and return mode.
- PointCloud2 fields/order, timestamps, synchronization, topic and frame.
- Measured LiDAR-to-base translation/rotation, mounting height and angle.

| Future interface | Mapping |
|---|---|
| Verified Scout LiDAR topic: TBD | `points_topic` |
| Verified LiDAR frame/extrinsics: TBD | Replace reference assumptions |
| LeGO integrated odometry | Downstream consumer and frame conversion: TBD |
| LeGO point-cloud map | Visualization; navigation representation: TBD |
| Scout wheel odometry: TBD | Validation/fusion design: TBD |

Choose TF ownership only after inspecting the robot. Wheel odometry, SLAM, static publishers and robot-state publishing must not compete for the same child frame. Evaluate a `map → odom → base_link → lidar_frame` arrangement without assuming LeGO currently supplies that interface.

## 15. Scripts and implementation checklist

Create all requested scripts:

- `setup_env.sh`: resolve root from its own location, source Humble and an existing overlay safely and repeatedly; preserve caller shell options.
- `install_dependencies.sh`: idempotent checks, logged apt changes, rosdep and isolated dataset tools.
- `build.sh`: underlay-only build, bounded concurrency, complete logs.
- `clean_build.sh`: remove only canonical project `build/`, `install/`, `log/`; reject unsafe roots and symlinked targets; rebuild.
- `launch_lego_loam.sh`: source environment, forward launch arguments.
- `test.sh`: bounded automated tests, owned-process cleanup and machine-readable results.

Add dataset fetch/conversion helpers and a source/dependency lock record.

- [x] Save plan.
- [x] Establish documentation and Git baseline.
- [x] Install and verify ROS/toolchain.
- [x] Import upstream and apply traceable fixes.
- [ ] Resolve dependencies and build. **Build passed; rosdep initialization remains.**
- [x] Verify launch, parameters, QoS and shutdown.
- [x] Run regression and synthetic tests.
- [ ] Acquire, convert and replay recorded data.
- [ ] Verify TF, odometry, mapping and RViz.
- [x] Encode successful procedures in scripts.
- [x] Clean-build and repeat scripted runtime validation.
- [x] Finish README, environment, build, testing, troubleshooting and porting documents.

## 16. Definition of done

All requested deliverables exist; both packages clean-build; ROS communication works; nodes launch and terminate correctly; compatible clouds are consumed; recorded-data processing produces credible odometry and mapping; TF is explained; RViz works or its limitation is documented.

Every dependency, source revision, patch and reproduction command is recorded. No success claim exceeds observed evidence.

Final reporting must distinguish **passed**, **failed**, **blocked** and **not tested**, including loop closure and trajectory accuracy.

## 17. Rollback and safety

Keep all non-system artifacts beneath the workspace. Back up any necessary apt configuration changes and retain package inventories.

Never alter shell startup files, install into `/usr/local`, disable package/TLS verification, touch GPU configuration, connect to the robot or publish motor commands.

Clean scripts must preserve source, datasets and evidence. Stop only processes started by this project. Revert source changes through Git; document package rollback without automatically removing shared dependencies.

Port by cloning or rsyncing source/configuration/documentation, then reinstall dependencies and rebuild on the target. Exclude generated binaries and overlays from cross-machine transfers.
