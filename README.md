# ROS 2 Humble LeGO-LOAM workspace

This workspace vendors `fishros/LeGO-LOAM-ROS2` at commit
`088856332b71e4bc4cc0f08cd8282ac370ca444a`, fixes its Humble launch and input
safety defects, and provides repeatable native build, test, dataset, and
migration workflows. It is currently a personal-computer baseline. Nothing in
this project connects to a robot or publishes motor commands.

## Local setup

Ubuntu 22.04 on amd64 or arm64 is supported by the setup scripts. The completed
desktop validation in this repository was performed on amd64; the target robot
must build and run the suite natively before real LiDAR use.

```bash
./install_dependencies.sh
./clean_build.sh
./test.sh
```

The installer adds the official ROS 2 apt source, installs Humble Desktop and
the packaged GTSAM 4.2 dependency, runs rosdep, and creates an isolated Python
environment for bag conversion. It records apt and package evidence under
`logs/`. It does not edit shell startup files.

For an interactive shell:

```bash
source setup_env.sh
```

This selects ROS domain 42 and localhost-only discovery unless those variables
are already set.

## Run synthetic or recorded data

```bash
./launch_lego_loam.sh rviz:=true use_sim_time:=true
```

Launch arguments are:

- `points_topic:=/velodyne_points`
- `params_file:=<installed VLP-16 YAML>`
- `use_sim_time:=true`
- `rviz:=true`
- `publish_reference_tf:=true`

The included `map`, `camera_init`, `camera`, `base_link`, and `velodyne` static
transforms are reference assumptions for desktop data only.

To prepare the Jackal recording:

```bash
./scripts/fetch_dataset.sh
./scripts/convert_dataset.sh
source setup_env.sh
ros2 bag play data/converted/2017-06-08-15-49-45_0 --clock --rate 1.0
```

The official Google Drive host currently refuses the dataset download because
its quota is exceeded. The exact file IDs and current blocker are recorded in
`docs/DATASET.md`; rerun the fetch helper when the host permits access.

Start LeGO-LOAM and its subscribers before playback. Restart both launch and
playback for a second run instead of looping timestamps backward.

## Robot quick start

For a complete Codex handoff on the robot, copy the prompt in
[`ROBOT_CODEX_PROMPT.md`](ROBOT_CODEX_PROMPT.md) into a new Codex session
running on the robot.

On the robot, clone the public repository and record its platform before
installing anything:

```bash
git clone https://github.com/ompug/oms_ros2_ws.git
cd oms_ros2_ws
./robot_setup.sh --preflight
```

After reviewing `docs/SCOUT_MINI_PORTING.md`, use `--install` for a fresh
Ubuntu 22.04 amd64/arm64 host, or `--build` when its dependencies are already
installed:

```bash
./robot_setup.sh --install
```

The bootstrap builds natively and runs the synthetic suite with localhost-only
ROS discovery. It does not publish velocity commands. Real LiDAR launch must
wait until the topic, PointCloud2 layout, timestamps, frames, and measured
extrinsics have been recorded in the porting checklist.

## Robot migration details

Copy the tracked source, configuration, scripts, and documentation. Exclude
`build/`, `install/`, `log/`, `logs/`, `.venv/`, caches, and downloaded bags.
On the robot, complete every item in `docs/SCOUT_MINI_PORTING.md`, install native
dependencies, and rebuild. Disable `publish_reference_tf` until measured robot
extrinsics and TF ownership are configured.

See `docs/BUILD.md`, `docs/TESTING.md`, `docs/TROUBLESHOOTING.md`, and
`docs/SCOUT_MINI_PORTING.md` for operational details and current evidence.
