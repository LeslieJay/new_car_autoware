#!/bin/bash
# 单点 smoke test：放置一个障碍物并检查绕障模块响应
# 用法: ./run_smoke_test.sh [intrusion_m]

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AUTOWARE_ROOT="/home/nvidia/autoware"
INTRUSION="${1:-0.4}"

cd "${AUTOWARE_ROOT}"
source install/setup.bash

chmod +x "${SCRIPT_DIR}"/scripts/*.py

python3 "${SCRIPT_DIR}/scripts/setup_scenario.py" \
  --config "${SCRIPT_DIR}/config/baseline.yaml"

sleep 5

python3 "${SCRIPT_DIR}/scripts/place_obstacle.py" \
  --config "${SCRIPT_DIR}/config/baseline.yaml" \
  --metrics "${SCRIPT_DIR}/config/metrics.yaml" \
  --intrusion "${INTRUSION}" \
  --clear-first \
  --verify \
  --hold 8
