#!/usr/bin/env bash
# Explicit loopback peer list; no connection to a robot on domain 42.
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2
_ugv_test_share=$(ros2 pkg prefix --share ugv_relocalization)
export CYCLONEDDS_URI="file://${_ugv_test_share}/config/cyclone_test_loopback.xml"
unset _ugv_test_share
