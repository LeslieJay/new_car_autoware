#!/usr/bin/env bash
# 正向前进终点精度测试 — 发车前模式基线自检
# 要求: gate_mode=AUTO(0), operation_mode=AUTONOMOUS(2), is_paused=false, engage=true

set -eo pipefail

FIX="${1:-}"

source /opt/ros/humble/setup.bash
if [ -f /home/nvidia/autoware/install/setup.bash ]; then
  # shellcheck disable=SC1091
  source /home/nvidia/autoware/install/setup.bash
fi

read_topic_once() {
  local topic=$1
  local field=$2
  timeout 5 ros2 topic echo "${topic}" --once 2>/dev/null | awk -v f="${field}" '$1 ~ f":" {print $2; exit}'
}

echo "=== 正向前进模式基线自检 ==="

gate_mode=$(read_topic_once /control/current_gate_mode data)
op_mode=$(read_topic_once /control/vehicle_cmd_gate/operation_mode mode)
auto_ctrl=$(read_topic_once /control/vehicle_cmd_gate/operation_mode is_autoware_control_enabled)
in_trans=$(read_topic_once /control/vehicle_cmd_gate/operation_mode is_in_transition)
is_paused=$(read_topic_once /control/vehicle_cmd_gate/is_paused data)
is_start=$(read_topic_once /control/vehicle_cmd_gate/is_start_requested data)
engage=$(read_topic_once /api/autoware/get/engage engage)

echo "  gate_mode (0=AUTO, 1=EXTERNAL):     ${gate_mode:-unknown}"
echo "  operation_mode (2=AUTONOMOUS, 3=LOCAL): ${op_mode:-unknown}"
echo "  is_autoware_control_enabled:        ${auto_ctrl:-unknown}"
echo "  is_in_transition:                   ${in_trans:-unknown}"
echo "  is_paused:                          ${is_paused:-unknown}"
echo "  is_start_requested:                 ${is_start:-unknown}"
echo "  engage:                             ${engage:-unknown}"

fail=0
warn=0

if [ "${gate_mode}" != "0" ]; then
  echo "FAIL: gate_mode 应为 AUTO(0)，当前=${gate_mode:-missing}" >&2
  fail=1
fi

if [ "${op_mode}" != "2" ]; then
  echo "FAIL: operation_mode 应为 AUTONOMOUS(2)，当前=${op_mode:-missing}" >&2
  fail=1
fi

if [ "${auto_ctrl}" != "true" ]; then
  echo "FAIL: is_autoware_control_enabled 应为 true" >&2
  fail=1
fi

if [ "${is_paused}" = "true" ]; then
  echo "FAIL: is_paused=true，车辆处于暂停" >&2
  fail=1
fi

if [ "${engage}" != "true" ]; then
  echo "WARN: engage 不为 true，可能影响控制输出" >&2
  warn=1
fi

if [ "${is_start}" = "false" ]; then
  echo "WARN: is_start_requested=false，部分场景下可能无法起步" >&2
  warn=1
fi

if [ "${fail}" -eq 0 ] && [ "${warn}" -eq 0 ]; then
  echo "=== PASS: 模式基线满足正向前进精度测试要求 ==="
  exit 0
fi

if [ "${FIX}" = "--fix" ] && [ "${fail}" -ne 0 -o "${warn}" -ne 0 ]; then
  echo ""
  echo "=== 尝试自动修复 (autonomous_mode_run.sh) ==="
  SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  bash "${SCRIPT_DIR}/../autonomous_mode_run.sh"
  echo ""
  echo "=== 修复后重新检查 ==="
  exec "$0"
fi

if [ "${fail}" -ne 0 ]; then
  echo ""
  echo "自检未通过。可执行: $0 --fix" >&2
  exit 1
fi

echo "=== PASS (with warnings) ==="
exit 0
