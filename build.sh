#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
LOG_FILE="${ROOT}/logs/build-${STAMP}.log"
mkdir -p "${ROOT}/logs"

if [[ ! -r /opt/ros/humble/setup.bash ]]; then
  echo "Install ROS 2 Humble first with ${ROOT}/install_dependencies.sh" >&2
  exit 1
fi

(
  unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH ROS_DISTRO ROS_VERSION
  set +u
  source /opt/ros/humble/setup.bash
  set -u
  cd "${ROOT}"
  export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
  export COLCON_LOG_PATH="${ROOT}/log"
  colcon build --symlink-install --executor sequential \
    --event-handlers console_direct+ \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
) 2>&1 | tee "${LOG_FILE}"

echo "Build log: ${LOG_FILE}"
