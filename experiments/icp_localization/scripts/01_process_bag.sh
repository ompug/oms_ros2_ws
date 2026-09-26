#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd -P)"
EXP="${ROOT}/experiments/icp_localization"
RAW="${1:-/home/ompug/ros2_bag_files/rosbag2_2026_09_11-12_01_42}"
OUT="${2:-${EXP}/bags/lego_2026_09_11}"
RATE="${RATE:-0.5}"
if [[ ! -f "${RAW}/metadata.yaml" ]]; then echo "Missing raw bag: ${RAW}" >&2; exit 2; fi
if [[ -e "${OUT}" ]]; then echo "Output already exists: ${OUT}" >&2; exit 2; fi
source "${ROOT}/setup_env.sh"
mkdir -p "${EXP}/bags" "${EXP}/logs"
cleanup() {
  [[ -z "${REC_PID:-}" ]] || { kill -TERM "${REC_PID}" 2>/dev/null || true; wait "${REC_PID}" 2>/dev/null || true; }
  [[ -z "${LAUNCH_PID:-}" ]] || { kill -TERM "${LAUNCH_PID}" 2>/dev/null || true; wait "${LAUNCH_PID}" 2>/dev/null || true; }
}
trap cleanup EXIT INT TERM
"${ROOT}/launch_lego_loam.sh" rviz:=false use_sim_time:=true \
  publish_reference_tf:=false points_topic:=/rslidar_points \
  > "${EXP}/logs/lego_launch.log" 2>&1 & LAUNCH_PID=$!
sleep 6
ros2 bag record -o "${OUT}" \
  /laser_cloud_surround /laser_cloud_full /aft_mapped_to_init /key_pose_origin \
  /laser_odom_to_init /laser_cloud_corner_last /laser_cloud_surf_last \
  /outlier_cloud_last /segmented_cloud /ground_cloud /segmented_cloud_info \
  /laser_cloud_sharp /laser_cloud_flat /integrated_to_init /tf /tf_static \
  > "${EXP}/logs/lego_record.log" 2>&1 & REC_PID=$!
sleep 5
ros2 bag play "${RAW}" --clock --rate "${RATE}" --topics /rslidar_points \
  > "${EXP}/logs/lego_play.log" 2>&1
sleep 10
timeout 180 ros2 service call /export_full_map std_srvs/srv/Trigger '{}' \
  > "${EXP}/logs/full_map_export.log" 2>&1
sleep 3
cleanup
trap - EXIT INT TERM
ros2 bag info "${OUT}" > "${EXP}/logs/processed_bag_info.txt"
echo "Processed bag: ${OUT}"
