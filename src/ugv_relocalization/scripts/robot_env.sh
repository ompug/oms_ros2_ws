#!/usr/bin/env bash
# Source after ROS/workspace setup. Do not silently substitute an unverified DDS interface.
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$HOME/cyclone_robot.xml}"
if [[ "$CYCLONEDDS_URI" == file://* && ! -f "${CYCLONEDDS_URI#file://}" ]]; then
    echo "Missing robot CycloneDDS file: ${CYCLONEDDS_URI#file://}" >&2
    return 1 2>/dev/null || exit 1
fi
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-2}"
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-2}"
