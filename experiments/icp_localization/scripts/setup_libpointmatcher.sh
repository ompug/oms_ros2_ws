#!/usr/bin/env bash
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "Source this file: source ${BASH_SOURCE[0]}" >&2
  exit 2
fi
_icp_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd -P)"
_icp_prefix="${_icp_root}/third_party/install"
export CMAKE_PREFIX_PATH="${_icp_prefix}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
export LD_LIBRARY_PATH="${_icp_prefix}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export PATH="${_icp_prefix}/bin:${PATH}"
unset _icp_root _icp_prefix
