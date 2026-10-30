# Scout Mini relocalization

Passive scan-to-reference localization for the previous graph navigation stack. Inputs are
`/rslidar_points` and LeGO `/integrated_to_init`, **not `/odom`**. Outputs are accepted
`/reloc/pose` and `/reloc/correction` in `refmap_ros`, JSON status, and calibrated graph
`/initialpose`. The bridge outputs `/lego/odometry` (`map`, `base_link`). Neither package
node publishes TF, Twist, or motor commands. Navigation is separately optional.

The package retains the experiment's multistart search, fine ICP, structure/attitude gate,
and competing-hypothesis ambiguity check. The portable fine backend is the experiment's
Open3D Tukey point-to-plane implementation; it does not require source-built
libpointmatcher/TEASER. Its capture performance must be measured in T1.

**Deployment status:** the source snapshot lacks `start_wally_lego.sh` and
`cyclone_robot.xml`; their behavior was recovered from the guide, launch files and
`start_robot_base.sh`, but the missing files were not silently replaced. The real graph
calibration and measured LiDAR extrinsic must be obtained on the robot. Until calibration
is supplied, ICP runs for inspection and the bridge keeps its math-pin. The saved local
coverage figure is provisional; no edge is claimed covered without calibration.

## Install on the robot

The previous robot workspaces must already be built (`~/ugv_ws`, `~/robosense_ws`,
`~/scout_humble_ws`, and a compatible LeGO workspace). The read-only archive is never
edited. The following assumes this repository is `~/oms_ros2_ws`:

```bash
cd ~/oms_ros2_ws
git fetch origin robot/relocalization-integration
git switch --track origin/robot/relocalization-integration  # first checkout only
# Subsequent checkout: git switch robot/relocalization-integration
source /opt/ros/humble/setup.bash
source ~/ugv_ws/install/setup.bash
source ~/robosense_ws/install/setup.bash
python3 -m venv --system-site-packages .venv-robot
.venv-robot/bin/python -m pip install -r src/ugv_relocalization/requirements.txt
# Running colcon through the venv is essential: it sets the installed node's interpreter.
.venv-robot/bin/python /usr/bin/colcon build --packages-select ugv_relocalization --symlink-install
source install/setup.bash
source src/ugv_relocalization/scripts/robot_env.sh
ros2 pkg prefix rmw_cyclonedds_cpp
```

If the Cyclone RMW package is absent, install `ros-humble-rmw-cyclonedds-cpp` using the
robot's system package manager. Use the robot's actual `~/cyclone_robot.xml`; the packaged
`cyclone_robot.template.xml` is only an explicitly labeled reconstruction of the old
base script's interface/peer settings. The host-time LiDAR YAML is installed in
`$(ros2 pkg prefix --share ugv_relocalization)/config/helios16_config_host_time.yaml`.
It keeps `use_lidar_clock: false`. Verify its host IP matches the robot.

## Bring up sensors, then localization

Use separate terminals, sourcing the same environments in each. Keep motors disabled
for calibration and T0/T1. Avoid running the old full launcher alongside this launch:
there must be only one pose bridge and one navigation stack.

```bash
# LiDAR (sourcing ~/robosense_ws/install/setup.bash first):
ros2 run rslidar_sdk rslidar_sdk_node --ros-args -p \
  config_path:="$(ros2 pkg prefix --share ugv_relocalization)/config/helios16_config_host_time.yaml"

# LeGO from THIS repository, if built (colcon build --packages-up-to lego_loam_sr):
ros2 launch lego_loam_sr run.launch.py points_topic:=/rslidar_points \
  use_sim_time:=false rviz:=false publish_reference_tf:=false
# The archived LeGO workspace and start_wally_lego.sh are absent locally; if using the
# robot's original SLAM launcher, start only its sensor/SLAM portion and stop its bridge/nav.

# Default: rough placement, fine ICP only. Without calibration, ref_start is in
# the reference ROS axes, e.g. zero for a placement near the reference bag's origin.
ros2 launch ugv_relocalization relocalization.launch.py mode:=rough launch_bridge:=true
# Fully automatic global proposals (no commanded drive or drive-measured heading):
ros2 launch ugv_relocalization relocalization.launch.py mode:=multistart launch_bridge:=true
# Exact graph math-pin fallback, with no scan registration:
ros2 launch ugv_relocalization relocalization.launch.py mode:=math_pin \
  launch_bridge:=true start_node:=A facing_node:=B
```

`launch_bridge` defaults to false to permit passive inspection beside the old stack.
Calibrated operation requires `launch_bridge:=true` and stopping the old bridge. Its
`/initialpose` callback ignores message timestamps and would otherwise re-pin a moving
robot against the latest odometry after asynchronous ICP. The modified copy uses
measurement-time history, measured lever-arm compensation, and jump guards. Default
math-pin parameters come from node coordinates and the start→facing bearing, not the
CSV's uniform yaw column. `launch_navigation` defaults to false. Enabling it includes
his installed `navigation_stack.launch.py`, passes `/lego/odometry` and the packaged
graph through its existing arguments, and keeps automatic mission start disabled.

## Calibration and calibrated launch

Measure `T_base_link_rslidar` (x y z roll pitch yaw, metres/radians). Store six comma
separated values in `EXTRINSIC` in every robot terminal; zero is not a measured extrinsic.

```bash
read -r -p 'Measured x,y,z,roll,pitch,yaw: ' EXTRINSIC
export EXTRINSIC
IFS=, read -r -a E <<< "$EXTRINSIC"
mkdir -p ~/.config/ugv_relocalization
# Terminal 1: physically place base_link at A, aim at B, restart SLAM there.
# Start capture BEFORE the ICP launch in terminal 2; run the bridge with no calibration.
ros2 run ugv_relocalization calibrate \
  --output ~/.config/ugv_relocalization/calibration.yaml --node A --facing B \
  --extrinsic "${E[@]}"
# Terminal 2 (same measured EXTRINSIC): global startup in the reference map:
ros2 launch ugv_relocalization relocalization.launch.py mode:=multistart \
  launch_bridge:=true start_node:=A facing_node:=B extrinsic:="$EXTRINSIC"
```

Capture requires an accepted startup pose, a bridge pose at the exact requested graph
pin within 5 cm/1°, and two seconds of stationary odometry. It saves
`T_graph_refmap = T_graph_base × inverse(T_refROS_base) × T_refROS_camera_init`,
with planar base poses. The saved transform's input is **raw reference camera_init**,
not the live SLAM frame. The file records graph/map hashes, extrinsic, node pair, stamp
and method. After capture, stop the uncalibrated localization launch and start:

```bash
ros2 launch ugv_relocalization relocalization.launch.py mode:=rough \
  calibration:="$HOME/.config/ugv_relocalization/calibration.yaml" \
  extrinsic:="$EXTRINSIC" extrinsic_measured:=true \
  launch_bridge:=true start_node:=A facing_node:=B
# Add launch_navigation:=true only for T3/T4 after T0–T2 pass.
ros2 run ugv_relocalization coverage \
  --calibration ~/.config/ugv_relocalization/calibration.yaml \
  --output ~/relocalization_results/graph_coverage.png
```

`map_path:=/absolute/reference_map.pcd` selects another map (also supply `--map-path`
to calibration/coverage). A changed map, graph or extrinsic requires fresh calibration.
`ref_start:=x,y,z,roll,pitch,yaw` supplies an uncalibrated rough reference pose; calibrated
rough seeds are derived from `start_node`/`facing_node`. Startup is bounded to 3 m/15°
from the rough prior. Periodic corrections are limited to 0.25 m/2° from prediction;
rejections keep odometry, without automatic global resets while driving. An initial
bridge re-pin exceeding these bounds requires two seconds stationary (5 cm/2°).
No map support within 5 m/30 reference points means odometry-only operation. This point
support mask is conservative geometry support, not a guarantee of localization quality.

## Replay and tests

```bash
source /opt/ros/humble/setup.bash
source ~/oms_ros2_ws/install/setup.bash
cd ~/oms_ros2_ws
source src/ugv_relocalization/scripts/test_env.sh
.venv-robot/bin/python -m pytest src/ugv_relocalization/test -q
# For a Sept 11 replay, combine raw scans with the processed LeGO output by SENSOR headers:
.venv-robot/bin/python src/ugv_relocalization/scripts/prepare_replay.py \
  --raw ~/ros2_bag_files/rosbag2_2026_09_11-12_01_42 \
  --lego experiments/icp_localization/bags/lego_2026_09_11_full \
  --output /tmp/ugv_september_replay --duration 30
RELOC_TEST_BAG=/tmp/ugv_september_replay RELOC_TEST_NAV=true \
  .venv-robot/bin/python -m pytest src/ugv_relocalization/test/test_replay_launch.py -q -s
# Manual replay (use separate terminals, same test environment):
ros2 launch ugv_relocalization relocalization.launch.py use_sim_time:=true \
  mode:=rough launch_bridge:=true
ros2 bag play /tmp/ugv_september_replay --clock 100
ros2 run ugv_relocalization bag_crosscheck \
  ~/ros2_bag_files/rosbag2_2026_09_11-12_01_42 --output /tmp/start_crosscheck.json \
  --trajectory experiments/icp_localization/trajectory/aft_mapped_to_init.csv
```

The launch test uses a clearly labeled identity graph calibration and zero extrinsic as
fixtures. It checks `/initialpose` and bridge output, not real graph registration. With
`RELOC_TEST_NAV=true`, the installed prior planner/follower run and replan A→I around a
blocked edge; all drive outputs are remapped under `/test/`. The default synthetic replay
also exercises calibration capture. Tests use loopback-only Cyclone peers; re-source
`robot_env.sh` **after unsetting CYCLONEDDS_URI** before returning to robot networking.

Sept 11 has LiDAR/camera topics, no GPS, graph pose or landmark annotation. Its first
LiDAR header is 1483233792801513433 ns while recording time is
1789142503139997503 ns. Therefore its start cannot independently be assigned to a graph
node from direct observations in this bag. A trajectory-to-graph shape search also stays
ambiguous: H and D yield nearly equal 2.05 m RMSE; A gives 3.44 m. These fits assume the
robot followed graph edges and are not calibration. The start remains unverified.
This is same-run replay, not independent-revisit evidence. Follow [ROBOT_TEST_PLAN.md](ROBOT_TEST_PLAN.md)
for the first independent revisits. Coverage figures and bags stay outside Git.
