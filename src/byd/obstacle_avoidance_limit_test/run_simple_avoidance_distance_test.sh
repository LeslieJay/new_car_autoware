#!/usr/bin/env bash
set -euo pipefail

ROOT="/home/nvidia/autoware"
SCRIPT="${ROOT}/src/byd/obstacle_avoidance_limit_test/scripts/simple_avoidance_distance_test.py"
CONFIG="${ROOT}/src/byd/obstacle_avoidance_limit_test/config/simple_avoidance_distance.yaml"
OUT="${ROOT}/log/$(date +%Y%m%d)/simple_avoidance_distance_test"
MODE="simple_avoidance"

# Start planning_simulator.launch.xml separately with matching isolation flags:
#   Simple Avoidance:          launch_simple_avoidance:=true
#                              launch_simple_lc_avoidance:=false
#   Simple LC Avoidance:       launch_simple_avoidance:=false
#                              launch_simple_lc_avoidance:=true
#   Both modes:                 launch_obstacle_stop_module:=false
#                              launch_dynamic_obstacle_stop_module:=false
# The wrapper intentionally does not start the simulator, so a stale launch
# with the opposite module enabled cannot be mistaken for this test.

if [[ "${1:-}" == "--mode" ]]; then
  MODE="${2:?--mode requires simple_avoidance or simple_lc_avoidance}"
  shift 2
fi

cd "${ROOT}"
python3 "${SCRIPT}" --config "${CONFIG}" --mode "${MODE}" --output-dir "${OUT}_${MODE}" --record-bag "$@"
