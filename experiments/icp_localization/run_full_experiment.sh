#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
EXP="${ROOT}/experiments/icp_localization"
RAW="${1:-/home/ompug/ros2_bag_files/rosbag2_2026_09_11-12_01_42}"
PROCESSED="${2:-${EXP}/bags/lego_2026_09_11_full}"
source "${ROOT}/setup_env.sh"
mkdir -p "${EXP}/logs/matplotlib"
export MPLCONFIGDIR="${EXP}/logs/matplotlib"

if [[ ! -f "${PROCESSED}/metadata.yaml" ]]; then
  bash "${EXP}/scripts/01_process_bag.sh" "${RAW}" "${PROCESSED}"
fi
if [[ ! -e "${EXP}/map/reference_map.pcd" ]]; then
  prior_args=()
  if [[ -f "${ROOT}/lego_maps/lego_loam_final_map.pcd" ]]; then
    prior_args=(--prior-pcd "${ROOT}/lego_maps/lego_loam_final_map.pcd")
  fi
  python3 "${EXP}/scripts/02_export_reference.py" --bag "${PROCESSED}" \
    --output "${EXP}/map/reference_map.pcd" "${prior_args[@]}"
fi
python3 "${EXP}/scripts/09_validate_map.py" --bag "${PROCESSED}"
if [[ ! -e "${EXP}/trajectory/aft_mapped_to_init.csv" ]]; then
  python3 "${EXP}/scripts/03_export_trajectory.py" --bag "${PROCESSED}" \
    --output "${EXP}/trajectory/aft_mapped_to_init.csv"
fi
if [[ ! -e "${EXP}/metadata/scans.csv" || ! -e "${EXP}/scans/scan_001.pcd" ]]; then
  python3 "${EXP}/scripts/04_extract_scans.py" --bag "${RAW}" \
    --output-dir "${EXP}/scans"
fi
python3 "${EXP}/scripts/analyze_pose_revisions.py" --bag "${PROCESSED}"
python3 "${EXP}/scripts/05_match_poses.py" \
  --scans "${EXP}/metadata/scans.csv" \
  --trajectory "${EXP}/trajectory/aft_mapped_to_init.csv" \
  --output "${EXP}/metadata/scans_with_reference_pose.csv"
cmake -S "${EXP}/src" -B "${EXP}/build" -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="${ROOT}/third_party/install"
cmake --build "${EXP}/build" -j 2
python3 "${EXP}/scripts/06_run_icp_experiment.py"
python3 "${EXP}/scripts/10_analyze_timing.py"
python3 "${EXP}/scripts/11_save_selected_clouds.py"
python3 "${EXP}/scripts/07_generate_plots.py"
python3 "${EXP}/scripts/08_visualize_registrations.py"
python3 "${EXP}/scripts/08_visualize_registrations.py" --snapshots scan_006 scan_004_heading_20deg
