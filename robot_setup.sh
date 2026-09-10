#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
MODE="${1:---preflight}"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
REPORT="${ROOT}/logs/robot-preflight-${STAMP}.txt"
mkdir -p "${ROOT}/logs"

case "${MODE}" in
  --preflight|--build|--install) ;;
  *)
    echo "Usage: $0 [--preflight|--build|--install]" >&2
    exit 2
    ;;
esac

OS_CODENAME="$(. /etc/os-release && printf '%s' "${VERSION_CODENAME:-unknown}")"
HOST_ARCH="$(dpkg --print-architecture 2>/dev/null || uname -m)"

{
  echo "timestamp_utc=${STAMP}"
  echo "git_revision=$(git -C "${ROOT}" rev-parse HEAD 2>/dev/null || echo unavailable)"
  echo "hostname=$(hostname)"
  echo "os_codename=${OS_CODENAME}"
  echo "architecture=${HOST_ARCH}"
  echo "kernel=$(uname -srmo)"
  echo "cpu_count=$(nproc)"
  free -h
  df -h "${ROOT}"
  if [[ -r /opt/ros/humble/setup.bash ]]; then
    echo "ros_humble=installed"
  else
    echo "ros_humble=not-installed"
  fi
  echo "ros2_on_current_path=$(command -v ros2 2>/dev/null || echo no)"
} | tee "${REPORT}"

echo "Preflight report: ${REPORT}"

if [[ "${MODE}" == "--preflight" ]]; then
  echo "Review docs/SCOUT_MINI_PORTING.md, then run $0 --install or $0 --build."
  exit 0
fi

if [[ "${OS_CODENAME}" != "jammy" ]] ||
   [[ "${HOST_ARCH}" != "amd64" && "${HOST_ARCH}" != "arm64" ]]; then
  echo "Automated setup requires Ubuntu Jammy on amd64 or arm64." >&2
  exit 1
fi

if [[ "${MODE}" == "--install" ]]; then
  "${ROOT}/install_dependencies.sh"
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY=1
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-1}"
"${ROOT}/clean_build.sh"
"${ROOT}/test.sh"

echo "Robot-side native build and localhost tests passed."
echo "Complete docs/SCOUT_MINI_PORTING.md before launching with real LiDAR data."
