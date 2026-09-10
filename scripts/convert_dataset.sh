#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
SEGMENT="${1:-0}"
case "${SEGMENT}" in
  0) STEM="2017-06-08-15-49-45_0" ;;
  1) STEM="2017-06-08-15-50-45_1" ;;
  2) STEM="2017-06-08-15-51-45_2" ;;
  3) STEM="2017-06-08-15-52-45_3" ;;
  *) echo "Usage: $0 [0|1|2|3]" >&2; exit 2 ;;
esac
SRC="${ROOT}/data/raw/${STEM}.bag"
DST="${ROOT}/data/converted/${STEM}"

if [[ ! -s "${SRC}" ]] || [[ ! -x "${ROOT}/.venv/bin/rosbags-convert" ]]; then
  echo "Fetch the dataset and install the pinned Python tools first." >&2
  exit 1
fi
if [[ -e "${DST}" ]]; then
  echo "Refusing to overwrite existing converted bag: ${DST}" >&2
  exit 1
fi

"${ROOT}/.venv/bin/rosbags-convert" --src "${SRC}" --dst "${DST}" \
  --dst-storage sqlite3 --dst-version 5 --dst-typestore ros2_humble \
  --include-topic /velodyne_points

source "${ROOT}/setup_env.sh"
ros2 bag info "${DST}" | tee "${ROOT}/data/converted/ros2-bag-info.txt"
