#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
source "${ROOT}/setup_env.sh"
exec ros2 launch lego_loam_sr run.launch.py "$@"

