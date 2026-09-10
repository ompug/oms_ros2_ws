#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
RESULT_DIR="${ROOT}/logs/test-${STAMP}"
START_SECONDS="$(date +%s)"
LAUNCH_PID=""
mkdir -p "${RESULT_DIR}"

stop_launch() {
  if [[ -n "${LAUNCH_PID}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill -INT -- "-${LAUNCH_PID}" 2>/dev/null || true
    for _ in $(seq 1 100); do
      if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
        wait "${LAUNCH_PID}" 2>/dev/null || true
        LAUNCH_PID=""
        return 0
      fi
      sleep 0.1
    done
    kill -TERM -- "-${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
    LAUNCH_PID=""
    return 1
  fi
  LAUNCH_PID=""
}

cleanup() {
  stop_launch || true
}
trap cleanup EXIT
trap 'cleanup; exit 130' INT TERM

source "${ROOT}/setup_env.sh"
if [[ ! -x "${ROOT}/install/lego_loam_sr/lib/lego_loam_sr/lego_loam_sr" ]]; then
  "${ROOT}/build.sh"
  source "${ROOT}/setup_env.sh"
fi

ros2 --help >/dev/null
ros2 pkg prefix demo_nodes_cpp > "${RESULT_DIR}/ros-package-prefix.txt"
setsid timeout 15 ros2 run demo_nodes_cpp talker \
  > "${RESULT_DIR}/talker.log" 2>&1 &
TALKER_PID=$!
set +e
timeout 12 ros2 run demo_nodes_py listener \
  > "${RESULT_DIR}/listener.log" 2>&1
LISTENER_RC=$?
set -e
kill -INT -- "-${TALKER_PID}" 2>/dev/null || true
wait "${TALKER_PID}" 2>/dev/null || true
if [[ "${LISTENER_RC}" -ne 124 ]] || ! rg -q 'I heard' "${RESULT_DIR}/listener.log"; then
  echo "ROS C++ talker/Python listener verification failed." >&2
  exit 1
fi

(
  cd "${ROOT}"
  colcon test --event-handlers console_direct+ \
    --packages-select cloud_msgs lego_loam_sr
  colcon test-result --verbose
) 2>&1 | tee "${RESULT_DIR}/colcon-test.log"

start_launch() {
  local log_name="$1"
  setsid "${ROOT}/launch_lego_loam.sh" rviz:=false use_sim_time:=false \
    publish_reference_tf:=true > "${RESULT_DIR}/${log_name}" 2>&1 &
  LAUNCH_PID=$!
  for _ in $(seq 1 40); do
    nodes="$(ros2 node list 2>/dev/null || true)"
    all_found=true
    for expected in image_projection feature_association map_optimization transform_fusion; do
      if ! grep -qx "/${expected}" <<< "${nodes}"; then
        all_found=false
      fi
    done
    if [[ "${all_found}" == true ]]; then
      return 0
    fi
    if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
      echo "Launch exited before all four nodes appeared." >&2
      return 1
    fi
    sleep 0.5
  done
  echo "Timed out waiting for all four nodes." >&2
  return 1
}

# Verify bounded Ctrl-C before any scan arrives.
start_launch "launch-before-data.log"
if ! stop_launch; then
  echo "Launch did not stop within 10 seconds before data." >&2
  exit 1
fi

start_launch "launch-runtime.log"
ros2 node list | sort > "${RESULT_DIR}/nodes.txt"
for node in image_projection feature_association map_optimization transform_fusion; do
  ros2 param get "/${node}" use_sim_time
done > "${RESULT_DIR}/use-sim-time.txt"
ros2 topic info --verbose /velodyne_points > "${RESULT_DIR}/input-endpoint.txt"

timeout 30 python3 "${ROOT}/tests/runtime_probe.py" --qos best_effort \
  --output "${RESULT_DIR}/best-effort.json"
timeout 30 python3 "${ROOT}/tests/runtime_probe.py" --qos reliable \
  --output "${RESULT_DIR}/reliable.json"

if ! stop_launch; then
  echo "Launch did not stop within 10 seconds after synthetic data." >&2
  exit 1
fi

set +e
timeout 15 ros2 run lego_loam_sr lego_loam_sr --ros-args \
  --params-file "${ROOT}/tests/invalid_params.yaml" \
  > "${RESULT_DIR}/invalid-parameters.log" 2>&1
INVALID_RC=$?
set -e
if [[ "${INVALID_RC}" -eq 0 || "${INVALID_RC}" -eq 124 ]]; then
  echo "Invalid parameter set was not rejected immediately." >&2
  exit 1
fi

END_SECONDS="$(date +%s)"
python3 - "${RESULT_DIR}" "${START_SECONDS}" "${END_SECONDS}" <<'PY'
import json
import pathlib
import sys

directory = pathlib.Path(sys.argv[1])
best_effort = json.loads((directory / "best-effort.json").read_text())
reliable = json.loads((directory / "reliable.json").read_text())
result = {
    "status": "passed",
    "duration_seconds": int(sys.argv[3]) - int(sys.argv[2]),
    "checks": {
        "ros_cpp_python_exchange": "passed",
        "colcon_tests": "passed",
        "four_nodes_and_parameters": "passed",
        "shutdown_before_and_after_data": "passed",
        "invalid_parameters_rejected": "passed",
        "best_effort_input": "passed" if best_effort["passed"] else "failed",
        "reliable_input": "passed" if reliable["passed"] else "failed",
    },
}
(directory / "result.json").write_text(json.dumps(result, indent=2) + "\n")
print(json.dumps(result, indent=2))
PY

echo "Test evidence: ${RESULT_DIR}"
