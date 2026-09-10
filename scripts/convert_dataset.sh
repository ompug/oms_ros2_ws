#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
SRC="${ROOT}/data/raw/2017-06-08-15-49-45_0.bag"
DST="${ROOT}/data/converted/2017-06-08-15-49-45_0"

if [[ ! -s "${SRC}" ]] || [[ ! -x "${ROOT}/.venv/bin/rosbags-convert" ]]; then
  echo "Fetch the dataset and install the pinned Python tools first." >&2
  exit 1
fi
if [[ -e "${DST}" ]]; then
  echo "Refusing to overwrite existing converted bag: ${DST}" >&2
  exit 1
fi

"${ROOT}/.venv/bin/rosbags-convert" --src "${SRC}" --dst "${DST}" \
  --dst-storage sqlite3 --dst-version 5 --include-topic /velodyne_points

source "${ROOT}/setup_env.sh"
ros2 bag info "${DST}" | tee "${ROOT}/data/converted/ros2-bag-info.txt"

