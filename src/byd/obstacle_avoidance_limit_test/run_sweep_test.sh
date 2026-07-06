#!/bin/bash
# 一键执行绕障极限 sweep 测试（需 planning_simulator 已运行）
# 用法: ./run_sweep_test.sh [--skip-setup]

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AUTOWARE_ROOT="/home/nvidia/autoware"

cd "${AUTOWARE_ROOT}"
source install/setup.bash

chmod +x "${SCRIPT_DIR}"/scripts/*.py

echo "Step 1/2: Setup scenario (initial pose + goal)..."
python3 "${SCRIPT_DIR}/scripts/setup_scenario.py" \
  --config "${SCRIPT_DIR}/config/baseline.yaml"

echo ""
echo "Step 2/2: Run lateral intrusion sweep..."
python3 "${SCRIPT_DIR}/scripts/obstacle_avoidance_sweep.py" \
  --config "${SCRIPT_DIR}/config/baseline.yaml" \
  --metrics "${SCRIPT_DIR}/config/metrics.yaml" \
  --output-dir "${SCRIPT_DIR}/results" \
  --skip-setup \
  "$@"

echo ""
echo "Done. Check results under ${SCRIPT_DIR}/results/"
