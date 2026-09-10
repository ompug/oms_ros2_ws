#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "Source this file: source ${BASH_SOURCE[0]}" >&2
  exit 2
fi

_lego_saved_options="$(set +o)"
_lego_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
_lego_domain_id="${ROS_DOMAIN_ID:-42}"
_lego_localhost_only="${ROS_LOCALHOST_ONLY:-1}"

if [[ ! -r /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble is not installed at /opt/ros/humble" >&2
  eval "${_lego_saved_options}"
  unset _lego_saved_options _lego_root _lego_domain_id _lego_localhost_only
  return 1
fi

set +u
source /opt/ros/humble/setup.bash
if [[ -r "${_lego_root}/install/setup.bash" ]]; then
  source "${_lego_root}/install/setup.bash"
fi

export ROS_DOMAIN_ID="${_lego_domain_id}"
export ROS_LOCALHOST_ONLY="${_lego_localhost_only}"

eval "${_lego_saved_options}"
unset _lego_saved_options _lego_root _lego_domain_id _lego_localhost_only
