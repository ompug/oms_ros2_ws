#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
if [[ ! -f "${ROOT}/.oms_ros2_ws-root" ]] ||
   [[ ! -f "${ROOT}/PROJECT_PLAN.md" ]] ||
   [[ ! -f "${ROOT}/src/LeGO-LOAM/package.xml" ]] ||
   [[ ! -f "${ROOT}/src/cloud_msgs/package.xml" ]] ||
   [[ "${ROOT}" == "/" ]] || [[ "${ROOT}" == "${HOME}" ]]; then
  echo "Refusing to clean an unexpected workspace root: ${ROOT}" >&2
  exit 1
fi

for name in build install log; do
  target="${ROOT}/${name}"
  if [[ -L "${target}" ]]; then
    echo "Refusing to remove symlinked target: ${target}" >&2
    exit 1
  fi
  if [[ -e "${target}" ]]; then
    rm -rf -- "${target}"
  fi
done

exec "${ROOT}/build.sh"
