#!/usr/bin/env bash
# Every robot trial gets a separate bag. Stop with Ctrl-C and wait for metadata.yaml.
set -euo pipefail
name=${1:?usage: record_trial.sh T1_A_rough_01}
mkdir -p "$HOME/relocalization_bags"
ros2 bag record -o "$HOME/relocalization_bags/${name}_$(date +%Y%m%d_%H%M%S)" \
    /rslidar_points /integrated_to_init /laser_odom_to_init /aft_mapped_to_init \
    /lego/odometry /reloc/pose /reloc/correction /reloc/status /reloc/bridge_status \
    /initialpose /tf /tf_static /clock /planned_path /dense_path /goal_node \
    /current_node /blocked_edge /current_edge /route_progress /planner_status \
    /cmd_vel /cmd_vel_raw /heartbeat /emergency_stop
