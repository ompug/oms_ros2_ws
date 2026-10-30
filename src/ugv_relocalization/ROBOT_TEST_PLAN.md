# Scout Mini robot tests

These are **unperformed robot tests**. Replay passed on the development host; no physical
calibration, surveyed graph alignment or independent revisit has yet been collected.
Every test below records a bag. Keep every bag outside Git. Preserve the map/calibration
hashes and trial notes with each bag. Do not use same-run replay as independent evidence.

## Before T0

Complete the README install/calibration steps. Have a supervisor holding the physical
e-stop, a clear pedestrian-free test area, and the robot's RC/manual takeover ready.
Initially disable motor power or lift the wheels. The software e-stop is an additional
control and does not replace the physical button. A teleop node may bypass the old
navigation safety mux; the physical stop must remain usable throughout T2.

In **every terminal**, source the following (use the robot's actual workspace paths):

```bash
source /opt/ros/humble/setup.bash
source ~/ugv_ws/install/setup.bash
source ~/robosense_ws/install/setup.bash
source ~/oms_ros2_ws/install/setup.bash
cd ~/oms_ros2_ws
unset CYCLONEDDS_URI
source src/ugv_relocalization/scripts/robot_env.sh
PKG=$(ros2 pkg prefix --share ugv_relocalization)
CAL="$HOME/.config/ugv_relocalization/calibration.yaml"
# Set the SIX MEASURED x,y,z,roll,pitch,yaw values, separated by commas:
read -r -p 'Measured base_link<-rslidar extrinsic: ' EXTRINSIC
export EXTRINSIC
mkdir -p ~/relocalization_results ~/relocalization_bags
```

Do not start `start_wally_lego.sh` beside this integration. Start LiDAR and LeGO using the
README commands; do not start a second bridge. Use `use_sim_time:=false` live. Start SLAM
after placement and hold still. Calibration at physical A facing B requires the capture
command to run before the uncalibrated `mode:=multistart` launch. Do not enable correction
in navigation until that calibration has been saved and its coverage overlay checked.

## T0 — topics, TF and clocks

Start a recording terminal **before** the checks:

```bash
bash "$PKG/scripts/record_trial.sh" T0_sanity
```

In another terminal, start passive calibrated localization and the navigation safety
stack. Automatic planning is disabled:

```bash
ros2 launch ugv_relocalization relocalization.launch.py mode:=rough \
  launch_bridge:=true launch_navigation:=true start_node:=A facing_node:=B \
  calibration:="$CAL" extrinsic:="$EXTRINSIC" extrinsic_measured:=true
```

Check physical e-stop and software stop while wheels cannot move:

```bash
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool '{data: true}'
.venv-robot/bin/python "$PKG/scripts/check_zero_twist.py" --seconds 3
ros2 topic list -t
ros2 topic info -v /rslidar_points
ros2 topic info -v /integrated_to_init
ros2 topic info -v /lego/odometry
ros2 node info /ugv_relocalization
ros2 node info /lego_pose_bridge
ros2 param get /ugv_relocalization use_sim_time
ros2 param get /lego_pose_bridge use_sim_time
ros2 param get /ugv_waypoint_manager use_sim_time
ros2 param get /ugv_waypoint_manager odom_topic
ros2 topic echo /integrated_to_init --once --field header
ros2 topic echo /rslidar_points --once --field header
ros2 topic echo /reloc/status --once
ros2 topic echo /reloc/bridge_status --once
ros2 run tf2_tools view_frames
ros2 topic info -v /tf
ros2 topic info -v /tf_static
```

Pass criteria: domain 42/Cyclone on all participants; LiDAR frame `rslidar`, LeGO odometry
frame `camera_init`; increasing host-time sensor headers near system wall time, not 2017;
no `/clock` publisher required live; all live nodes have `use_sim_time=false`; navigation
reads `/lego/odometry` with `frame_id=map`; exactly one bridge publishes it. Observe at
least one accepted startup and one accepted periodic correction plus an accepted bridge
status at the corresponding stamp. Package node publisher lists contain no drive topic
and no TF topic. Reliable scan QoS must match the driver; the LeGO odom subscription
accepts best-effort or reliable publishers.

Inspect the TF tree for duplicate child parents and duplicate transform authorities.
There must be **at most one map→odom publisher**, and exactly one if that transform exists
in the selected base/SLAM configuration. This integration deliberately adds none: graph
navigation already consumes a `map`-frame Odometry message and performs an identity
map→map lookup. Do not add an identity graph-map→live-camera_init transform to make RViz
look aligned; that would assert an uncalibrated frame relationship. The repository's
LeGO command uses `publish_reference_tf:=false`. Check the base driver's own TF before
adding any sensor transforms. If the base already publishes odom→base_link, do not let
an old LeGO static camera→base_link publisher assign a second parent.

`check_zero_twist.py` fails on a nonzero message or no messages. Release the software
stop only under supervision, after physical-stop behavior has been demonstrated:

```bash
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool '{data: false}'
```

Finish each recording with Ctrl-C and verify its metadata exists before moving on.

## T1 — independent static revisits

First generate the real coverage overlay and inspect its edge CSV:

```bash
ros2 run ugv_relocalization coverage --calibration "$CAL" \
  --output ~/relocalization_results/graph_coverage.png
```

Nominate A, B, C and D as the four stations, **only after the calibrated overlay confirms
they and their approach segments have support**. If any are outside coverage, record
that exclusion and choose three supported graph stations in addition to A. Do not claim
an unsupported station passed. Candidate facing pairs are A→B, B→A, C→B, and D→`__F_G_5`.
Bearings are calculated from the supplied graph coordinates.

Run **5 independent repeats × 4 stations × 3 modes = 60 bags**. Reuse the interactive
placement/repeat and per-trial CSV pattern of `~/previous_agx_work/reloc_trials.sh`; do not
run its drive-measured heading or auto-drive code. This package never commands a heading
drive. Each repeat should use the same surveyed floor marks across the three modes.
Measure the true base pose with tape/surveyed marks; record true graph x/y/yaw, signed
placement error, node/facing pair, mode, repeat, and environmental changes in trial notes.

For mode (a), place 1–2 m from the nominated node and ±10° from the nominated bearing.
Use both translation signs and both yaw signs over five repeats. Restore those same
marked poses in modes (b) and (c), so the math-pin baseline is measured under the same
placement errors. An additional exact-placement math-pin check is useful for validating
the marks. Hold still throughout startup. **Restart SLAM after every placement**; the
math-pin assumes the current SLAM origin is the robot's boot pose.

Example commands for A, repeat 1 (set START/FACE/MODE/REP for every cell):

```bash
START=A; FACE=B; MODE=rough; REP=1
# Recording terminal, started before the localization process:
bash "$PKG/scripts/record_trial.sh" "T1_${START}_${MODE}_${REP}"
# Localization terminal, after placing robot and restarting LeGO at the marked pose:
ros2 launch ugv_relocalization relocalization.launch.py mode:="$MODE" \
  start_node:="$START" facing_node:="$FACE" launch_bridge:=true \
  calibration:="$CAL" extrinsic:="$EXTRINSIC" extrinsic_measured:=true
# No navigation for these static tests. Record accepted pose or allow 180 s for failure.
ros2 topic echo /reloc/status
ros2 topic echo /reloc/bridge_status
ros2 topic echo /lego/odometry --once
```

For (b) set `MODE=multistart`. For (c) set `MODE=math_pin`; no registration events are
expected and the bridge retains its analytic graph pin. Stop the localization launch
and bag cleanly for each cell, then reposition, restart SLAM and repeat. Record time to
first accepted ICP pose **and** first accepted bridge update; gate acceptance alone is
not a successful navigation tie-in. Compute horizontal/yaw error from the surveyed true
pose, acceptance rate, median/p95 time and mean/SD/max errors per mode and station. Count
timeouts, rejected jumps, and false accepts. Initial large graph re-pins require two
seconds stationary. Accepted periodic updates must remain ≤0.25 m/2° from prediction.
T1 is the first independent-revisit dataset, not another replay of the map-building run.

## T2 — supervised teleop, passive localization

After T1 passes, choose a short path entirely inside green coverage. Keep the navigation
stack off and use the robot's validated RC/manual teleop procedure with a physical e-stop
supervisor. Start a bag and the calibrated localization launch; do not issue a graph goal:

```bash
bash "$PKG/scripts/record_trial.sh" T2_passive_teleop
ros2 launch ugv_relocalization relocalization.launch.py mode:=rough launch_bridge:=true \
  start_node:=A facing_node:=B calibration:="$CAL" \
  extrinsic:="$EXTRINSIC" extrinsic_measured:=true
# Separate teleop terminal if the installed keyboard teleop is the validated procedure:
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

At walking speed or below, drive straight sections and gentle turns, stop, and revisit
A. Compare LeGO prediction, accepted reference fixes, and graph odometry offline. Watch
correction rejection, lag and bridge jumps. A coverage rejection must produce an
`odometry_only` event and continued bridge odometry, with no global-reset jump. Do not
leave map coverage to provoke it during this first robot test; exercise that behavior
on replay. Stop immediately on a pose jump, stale clocks, wrong heading, or unsafe drive
output. Finish and verify the recording.

## T3 — autonomous A→I mission against math-pin baseline

Use A→I only if the **entire planned route** is inside confirmed coverage. Otherwise select
a supported route and record why A→I was excluded. Match speed, route, payload, map,
calibration and environment. Run at least 3 paired repeats, alternate correction/baseline
order, restore physical A facing B and restart SLAM for every mission.

```bash
# Recording terminal (change mode/repeat in the label):
bash "$PKG/scripts/record_trial.sh" T3_A_I_rough_01
# Launch terminal, after placing robot at A facing B:
ros2 launch ugv_relocalization relocalization.launch.py mode:=rough \
  launch_bridge:=true launch_navigation:=true start_node:=A facing_node:=B goal_node:=I \
  calibration:="$CAL" extrinsic:="$EXTRINSIC" extrinsic_measured:=true
# Monitor terminal: run the existing, unmodified script on the ROBOT, not the read-only archive:
python3 ~/autonomy_monitor.py --ros-args -p run_id:=T3_rough_01 -p start:=A -p goal:=I
# Safety/goal terminal: physical e-stop held ready, verify stopped output before release:
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool '{data: true}'
.venv-robot/bin/python "$PKG/scripts/check_zero_twist.py" --seconds 3
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool '{data: false}'
ros2 topic pub --once /goal_node std_msgs/msg/String '{data: I}'
```

Observe arrival and stop. Ctrl-C the monitor to flush its CSV, then finish the bag.
Repeat the same commands with `mode:=math_pin` and a baseline run_id/bag label. Compare
`autonomy_monitor.py` maximum/RMS cross-track error, duration, completed distance, rejects,
pose jumps and interventions. Its cross-track metric uses the **estimated** graph pose;
it is not independent ground truth and cannot by itself prove improved physical accuracy.
Use surveyed marks/video or an external reference to assess physical path error.

## T4 — blocked-edge replan with correction enabled

Repeat a supported A→I mission using a new recording, the T3 correction launch and monitor
with `T4` labels. Before entering the target edge, while the supervisor can stop the robot,
block the graph edge A→first subnode (an alternate route exists in the archived graph):

```bash
bash "$PKG/scripts/record_trial.sh" T4_A_I_blocked_01
# After the T3 correction launch and goal have been started in other terminals:
ros2 topic pub --once /blocked_edge ugv_msgs/msg/BlockedEdge \
  '{edge_id: A___A_B_1, source: supervised_T4}'
ros2 topic echo /planned_path --once
ros2 topic echo /planner_status --once
```

Check the replacement route in RViz/edge IDs **before allowing further motion**. The
alternate A→H→G… route may extend beyond reference coverage; if so, stop and choose a
supported blocked-edge mission instead. The planner must exclude the blocked edge,
continue receiving `/lego/odometry`, and keep correction jumps bounded throughout the
replan. Record route IDs, blockage time, cross-track error, completion, fallback intervals
and interventions. Keep the software/physical stop available and never use a pedestrian
as the obstacle. Finish the bag and monitor CSV even if the mission is aborted.

## Handoff evidence

For each bag, retain test/mode/station/repeat, measured true placement, git SHA,
map/graph/calibration hashes, host-time configuration, TF authority inspection,
acceptance/rejection counts, time to accepted bridge update, any odometry-only intervals,
mission/blocked-edge outcome and e-stop interventions. Save bags, coverage figures,
monitor CSVs and trial notes under the operator's data directories; none enter Git.
