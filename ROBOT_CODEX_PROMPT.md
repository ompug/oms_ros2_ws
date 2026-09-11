# Codex prompt for Scout Mini robot integration

Copy everything below this line into Codex while it is running on the robot.

---

You are working directly on my Scout Mini robot. Your objective is to finish
the robot phase of my ROS 2 Humble LeGO-LOAM project and leave it reproducible,
tested, documented, committed, and pushed to GitHub.

The public repository is:

```text
https://github.com/ompug/oms_ros2_ws.git
```

Work autonomously and persist until LeGO-LOAM consumes the robot's real LiDAR,
publishes valid odometry and mapping outputs, has a correct single-owner TF
tree, and can be launched reproducibly after reboot. Ask me only when you need
a sudo password, physical access, a manual driving action, or information that
cannot be observed from the robot. Never invent robot specifications.

## Safety boundary

- Do not publish to `/cmd_vel` or any other actuator or motor-control topic.
- Do not change motor-controller, battery, CAN, network, firmware, bootloader,
  GPU, or system startup configuration unless I explicitly authorize it.
- Do not drive the robot. When motion is needed to validate mapping, prepare
  the system, tell me exactly what is ready, and ask me to drive it manually
  with the existing verified teleoperation method.
- Inspect existing ROS nodes, topics, services, parameters, launch files, URDF,
  and TF publishers before changing them.
- Preserve working robot drivers and configurations. Back up any system or
  robot-specific file before editing it.
- Do not use the Jackal dataset. It was an optional desktop test fixture; the
  robot's real LiDAR is the integration input.

## Start here

1. Clone or update the repository without discarding local robot changes:

   ```bash
   git clone https://github.com/ompug/oms_ros2_ws.git
   cd oms_ros2_ws
   git status
   ./robot_setup.sh --preflight
   ```

   If it is already cloned, fetch first, inspect the status and history, and
   integrate upstream changes safely. Never use `git reset --hard`, force-push,
   or delete uncommitted work.

2. Read these files before implementation:

   - `PROJECT_PLAN.md`
   - `README.md`
   - `docs/SCOUT_MINI_PORTING.md`
   - `docs/PARAMETERS.md`
   - `docs/TF_AND_RVIZ.md`
   - `docs/TESTING.md`
   - `docs/TROUBLESHOOTING.md`

3. Create a `robot-integration` branch from the current public `main`. Use the
   configured Git identity. Check `gh auth status`; if GitHub authentication is
   missing, ask me to run the normal interactive `gh auth login` flow. Do not
   use token workarounds. Commit coherent, reviewable changes and push the
   branch regularly. Never force-push.

## Inspect the robot before installation

Record observed results in `docs/SCOUT_MINI_PORTING.md` and timestamped files
under `logs/`. At minimum determine:

- CPU architecture, Ubuntu version, kernel, RAM, swap, available storage, and
  whether the host is a Jetson or another onboard computer.
- Installed ROS distribution, RMW implementation, ROS domain settings, and
  whether the LiDAR driver runs on this computer or another ROS 2 host.
- Scout driver repository/package/version, active launch files, node names,
  `/cmd_vel` interface, wheel-odometry topic, and existing TF ownership. These
  interfaces are for inspection only; do not command the base.
- Robot URDF, `robot_state_publisher`, base frame names, odometry frame names,
  and every publisher of `/tf` and `/tf_static`.
- LiDAR manufacturer and exact model, driver/version, topic, frame, message
  rate, point count, QoS, timestamp source, PointCloud2 field names/types,
  `point_step`, organization, scan ordering, channel/ring layout, return mode,
  and measured or configured LiDAR-to-base extrinsics.

Use commands such as `ros2 node list`, `ros2 topic list -t`,
`ros2 topic info --verbose`, `ros2 topic hz`, bounded `ros2 topic echo --once`,
`ros2 param dump`, `ros2 run tf2_tools view_frames`, and inspection of installed
launch/URDF files. Bound commands that could wait forever. Do not expose secrets
or publish machine-specific logs to GitHub without reviewing them.

Determine whether `ROS_LOCALHOST_ONLY=1` can see the driver. Keep localhost-only
discovery for synthetic tests. Use normal network discovery for live LiDAR only
if the driver is on another host and the robot's existing ROS network requires
it; document the selected domain and discovery settings.

## Install and build natively

The automated installer supports Ubuntu 22.04 Jammy on amd64 and arm64. The
desktop baseline was validated on amd64, so this robot must be built and tested
natively.

- On supported Jammy hosts, run `./robot_setup.sh --install` for a fresh system
  or `./robot_setup.sh --build` if dependencies are already installed.
- Start with `CMAKE_BUILD_PARALLEL_LEVEL=1` on constrained hardware.
- If the OS, architecture, ROS distribution, or apt repositories differ, do
  not force Jammy packages onto the machine. Record the mismatch, inspect the
  platform, and make the smallest supportable adaptation.
- Use official Ubuntu and ROS repositories with signature verification. Do not
  add a PPA or build GTSAM from source unless the official ROS package is
  demonstrably unavailable or incompatible, and record the evidence first.
- Keep all source builds and caches under the workspace. Do not install custom
  libraries into `/usr/local`.

Require a clean native build, package/executable discovery, no missing `ldd`
libraries, passing colcon tests, and a passing `./test.sh` synthetic suite.
Verify bounded Ctrl-C shutdown and confirm no orphaned project processes.

## Integrate the real LiDAR

The current projection model assumes a rotating VLP-16-style scan with 16
evenly spaced vertical channels from -15 to +15 degrees, about 10 Hz, and
float32 `x`, `y`, `z`, and `intensity` fields. It does not consume `ring` or
per-point time fields. Confirm the actual sensor satisfies the model before
changing only YAML values.

If the robot has a different channel count, nonuniform vertical angles,
different scan ordering, a solid-state LiDAR, or missing float32 intensity,
analyze the projection code and implement a correct, tested sensor adaptation.
Do not claim compatibility based only on matching topic names. Preserve the
existing VLP-16 path and add regression tests for any new model or input
normalization.

For the first live launch, use the observed topic and disable the desktop
reference transforms:

```bash
source setup_env.sh
./launch_lego_loam.sh \
  points_topic:=<observed_lidar_topic> \
  use_sim_time:=false \
  rviz:=false \
  publish_reference_tf:=false
```

Set `rviz:=true` only when the robot has a working display, or visualize from a
properly configured remote ROS 2 computer. Start diagnostic subscribers before
the LiDAR stream because some clouds publish only when subscribers exist.

## Resolve TF deliberately

Do not reuse the desktop's zero-offset `base_link -> velodyne` assumption. Use
the robot's measured/calibrated LiDAR extrinsics and existing URDF. Build one
connected TF tree with exactly one publisher for every child frame. Prevent
LeGO-LOAM, the Scout driver, `robot_state_publisher`, static publishers, and any
localization system from competing for the same child.

Document the final mapping among the robot's `map`, `odom`, base, and LiDAR
frames and LeGO-LOAM's historical `camera_init`, `camera`, `laser_odom`, and
`aft_mapped` frames. If an adapter or launch change is necessary, implement it
explicitly and test timestamps, orientation conventions, and transform
connectivity. A frame-ID rename is not a coordinate transform.

## Live acceptance tests

First test while the robot is stationary. Then, when the pipeline is stable,
ask me to drive a short loop manually in a safe area. You must monitor and
record:

- Stable input frequency and advancing sensor timestamps.
- Nonempty `/segmented_cloud` and `/ground_cloud`.
- Nonempty corner and surface feature clouds.
- Advancing `/laser_odom_to_init`, `/aft_mapped_to_init`, and
  `/integrated_to_init` with finite positions and normalized quaternions.
- Multiple `/key_pose_origin` keyframes during motion.
- A growing `/laser_cloud_surround` map with geometry consistent with the
  observed area.
- One connected TF tree, one parent per child, correct timestamps, and no
  duplicate authorities.
- RViz rendering of raw, segmented, ground, feature, odometry, key-pose, map,
  and TF displays when visualization is available.
- CPU, memory, temperature, dropped scans, callback backlog, warnings, and
  clean bounded shutdown.

Record a short robot LiDAR bag for repeatable regression if storage allows.
Keep large bags ignored by Git. Replay with `use_sim_time:=true` and `/clock`,
using fresh LeGO-LOAM processes for repeated runs. Do not loop time backward in
one process.

Run the live stationary and motion test at least twice. Report trajectory and
map credibility honestly. Output topic existence alone is insufficient. If the
available motion is too short to evaluate loop closure, mark loop closure as
not tested rather than passed.

## Deliverables and completion

- Update the porting checklist with every observed robot fact and remove the
  corresponding TBD markers.
- Add robot-specific configuration and launch helpers without overwriting the
  tested desktop VLP-16 baseline.
- Make startup after a fresh clone concise and document exact commands. Do not
  add an automatic boot service unless I explicitly request one after manual
  launch is stable.
- Update build, testing, TF, parameter, and troubleshooting documentation with
  commands, timestamps, versions, outcomes, and limitations.
- Keep generated overlays, logs, credentials, bags, and machine secrets out of
  Git.
- Commit all source and documentation changes to `robot-integration`, push it
  to `origin`, and give me the GitHub branch or pull-request URL.
- Finish with a clear table of passed, failed, blocked, and not-tested items.

The task is complete only when the robot's real LiDAR has been consumed
successfully, odometry and mapping behavior have been assessed during actual
motion, TF is correct and conflict-free, repeatable launch instructions work,
and all reviewable changes are pushed. If a physical or external blocker
prevents completion, finish every independent task and record the exact blocker
and next command.
