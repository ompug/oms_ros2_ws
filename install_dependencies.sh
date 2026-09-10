#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
LOG_DIR="${ROOT}/logs"
DOWNLOAD_DIR="${ROOT}/downloads"
CACHE_DIR="${ROOT}/cache"
ROS_HOME_DIR="${ROOT}/.ros"
mkdir -p "${LOG_DIR}/apt" "${DOWNLOAD_DIR}" "${CACHE_DIR}/pip" "${ROS_HOME_DIR}"

OS_CODENAME="$(. /etc/os-release && printf '%s' "${VERSION_CODENAME:-}")"
HOST_ARCH="$(dpkg --print-architecture)"
if [[ "${OS_CODENAME}" != "jammy" ]] ||
   [[ "${HOST_ARCH}" != "amd64" && "${HOST_ARCH}" != "arm64" ]]; then
  echo "This installer supports Ubuntu Jammy on amd64 or arm64; found ${OS_CODENAME:-unknown} ${HOST_ARCH}." >&2
  exit 1
fi

if ! sudo -v; then
  echo "sudo authentication is required for ROS and system dependencies." >&2
  exit 1
fi

dpkg-query -W -f='${binary:Package}\t${Version}\n' 2>/dev/null | sort \
  > "${LOG_DIR}/inventory-before-${STAMP}.tsv"
sudo cp --preserve=mode,timestamps /etc/apt/sources.list \
  "${LOG_DIR}/apt/sources.list-before-${STAMP}"

sudo apt-get update 2>&1 | tee "${LOG_DIR}/apt/update-base-${STAMP}.log"
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
  ca-certificates curl software-properties-common

if ! rg -q '^[[:space:]]*deb .* jammy-updates ' /etc/apt/sources.list /etc/apt/sources.list.d 2>/dev/null; then
  sudo add-apt-repository -y \
    'deb http://us.archive.ubuntu.com/ubuntu/ jammy-updates main restricted universe multiverse'
fi
sudo add-apt-repository -y universe
sudo apt-get update 2>&1 | tee "${LOG_DIR}/apt/update-jammy-${STAMP}.log"

ROS_APT_SOURCE_VERSION="$(curl -fsSL \
  https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | sed -n 's/.*"tag_name"[[:space:]]*:[[:space:]]*"\([^"]*\)".*/\1/p')"
if [[ -z "${ROS_APT_SOURCE_VERSION}" ]]; then
  echo "Could not resolve the current official ros-apt-source release." >&2
  exit 1
fi
ROS_APT_DEB="${DOWNLOAD_DIR}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo "${UBUNTU_CODENAME}")_all.deb"
if [[ ! -s "${ROS_APT_DEB}" ]]; then
  curl -fL --retry 3 -o "${ROS_APT_DEB}" \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo "${UBUNTU_CODENAME}")_all.deb"
fi
dpkg-deb --info "${ROS_APT_DEB}" > "${LOG_DIR}/apt/ros2-apt-source-${STAMP}.txt"
sudo dpkg -i "${ROS_APT_DEB}"
sudo apt-get update 2>&1 | tee "${LOG_DIR}/apt/update-ros-${STAMP}.log"

PACKAGES=(
  ros-humble-desktop
  ros-humble-gtsam
  ros-dev-tools
  build-essential
  cmake
  git
  python3-colcon-common-extensions
  python3-rosdep
  python3-venv
  ripgrep
)
SIM_LOG="${LOG_DIR}/apt/install-simulation-${STAMP}.log"
apt-get --simulate install "${PACKAGES[@]}" | tee "${SIM_LOG}"
if rg -q '^Remv (ubuntu-desktop|ubuntu-desktop-minimal|systemd|systemd-sysv|udev|network-manager)(:|[[:space:]])' "${SIM_LOG}"; then
  echo "Refusing an apt transaction that removes a desktop or core system package." >&2
  exit 1
fi
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y "${PACKAGES[@]}" \
  2>&1 | tee "${LOG_DIR}/apt/install-${STAMP}.log"

export ROS_HOME="${ROS_HOME_DIR}"
set +u
source /opt/ros/humble/setup.bash
set -u
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update --rosdistro humble
rosdep install --from-paths "${ROOT}/src" --ignore-src --rosdistro humble -r -y \
  2>&1 | tee "${LOG_DIR}/rosdep-${STAMP}.log"

python3 -m venv "${ROOT}/.venv"
PIP_CACHE_DIR="${CACHE_DIR}/pip" "${ROOT}/.venv/bin/python" -m pip install --upgrade pip
PIP_CACHE_DIR="${CACHE_DIR}/pip" "${ROOT}/.venv/bin/python" -m pip install \
  'rosbags==0.11.5' 'gdown==5.2.0'
"${ROOT}/.venv/bin/python" -m pip freeze > "${ROOT}/docs/python-requirements.lock"

{
  echo '# Installed versions'
  echo
  printf 'Recorded: `%s`\n\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo '```text'
  dpkg-query -W -f='${binary:Package}\t${Version}\n' \
    ros-humble-desktop ros-humble-gtsam ros2-apt-source build-essential \
    cmake git python3-colcon-common-extensions python3-rosdep python3-venv
  gcc --version | head -n 1
  g++ --version | head -n 1
  python3 --version
  echo '```'
} > "${ROOT}/docs/INSTALLED_VERSIONS.md"

dpkg-query -W -f='${binary:Package}\t${Version}\n' 2>/dev/null | sort \
  > "${LOG_DIR}/inventory-after-${STAMP}.tsv"
comm -13 "${LOG_DIR}/inventory-before-${STAMP}.tsv" \
  "${LOG_DIR}/inventory-after-${STAMP}.tsv" \
  > "${LOG_DIR}/inventory-installed-${STAMP}.tsv"

ros2 --help >/dev/null
colcon --help >/dev/null
git --version
cmake --version | head -n 1
echo "Dependencies installed. Evidence is under ${LOG_DIR}."
