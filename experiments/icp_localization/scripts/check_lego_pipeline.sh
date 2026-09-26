#!/usr/bin/env bash
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd -P)"
source "${ROOT}/setup_env.sh"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID} ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
echo "Nodes:"
ros2 node list
for topic in /clock /rslidar_points /segmented_cloud /laser_cloud_sharp \
  /laser_cloud_corner_last /laser_odom_to_init /aft_mapped_to_init \
  /key_pose_origin /laser_cloud_surround; do
  echo "===== ${topic} ====="
  ros2 topic info --verbose "${topic}" 2>&1 | head -28
  if [[ "${topic}" != /clock ]]; then
    timeout 6 ros2 topic hz "${topic}" 2>&1 | tail -3 || true
  fi
  if [[ "${topic}" == /rslidar_points || "${topic}" == /segmented_cloud || \
        "${topic}" == /laser_cloud_surround ]]; then
    timeout 5 ros2 topic echo --once --field header.frame_id "${topic}" 2>&1 | head -3 || true
  fi
done
for node in /image_projection /feature_association /map_optimization; do
  echo "===== ${node} ====="
  ros2 node info "${node}" 2>&1 | head -90 || true
  ros2 param get "${node}" use_sim_time 2>&1 || true
done
