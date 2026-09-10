#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "Source this file: source ${BASH_SOURCE[0]}" >&2
  exit 2
fi

_lego_saved_options="$(set +o)"
_lego_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"

if [[ ! -r /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble is not installed at /opt/ros/humble" >&2
  eval "${_lego_saved_options}"
  unset _lego_saved_options _lego_root
  return 1
fi

source /opt/ros/humble/setup.bash
if [[ -r "${_lego_root}/install/setup.bash" ]]; then
  source "${_lego_root}/install/setup.bash"
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

eval "${_lego_saved_options}"
unset _lego_saved_options _lego_root

