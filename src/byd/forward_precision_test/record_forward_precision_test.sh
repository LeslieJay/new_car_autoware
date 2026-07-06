#!/usr/bin/env bash
# 正向前进终点精度测试 — 录包脚本
# 用法: ./record_forward_precision_test.sh [输出目录名]
# 发车前请先运行 verify_forward_mode_baseline.sh

set -eo pipefail

OUT_NAME="${1:-forward_precision_$(date +%Y%m%d_%H%M%S)}"
OUT_DIR="/home/nvidia/autoware/log/${OUT_NAME}"

source /opt/ros/humble/setup.bash
if [ -f /home/nvidia/autoware/install/setup.bash ]; then
  # shellcheck disable=SC1091
  source /home/nvidia/autoware/install/setup.bash
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=== 发车前模式自检 ==="
if ! bash "${SCRIPT_DIR}/verify_forward_mode_baseline.sh"; then
  echo "模式基线未通过，拒绝录包。修复后重试: verify_forward_mode_baseline.sh --fix" >&2
  exit 1
fi

mkdir -p "${OUT_DIR}"

TOPICS=(
  /control/current_gate_mode
  /control/vehicle_cmd_gate/operation_mode
  /control/vehicle_cmd_gate/is_paused
  /control/vehicle_cmd_gate/is_start_requested
  /control/trajectory_follower/control_cmd
  /control/command/control_cmd
  /planning/mission_planning/goal
  /planning/trajectory
  /planning/mission_remaining_distance_time
  /planning/mission_planning/state
  /planning/route_state
  /localization/kinematic_state
  /vehicle/status/velocity_status
  /vehicle/status/gear_status
  /vehicle/status/control_mode
)

echo ""
echo "=== 开始录包: ${OUT_DIR} ==="
echo "关键话题: ${#TOPICS[@]} 个"
echo ""
echo "发车步骤:"
echo "  1. ros2 topic pub --once /planning/mission_planning/goal geometry_msgs/msg/PoseStamped \\"
echo "       \"{header: {frame_id: 'map'}, pose: {position: {x: 261.444300524526, y: -28.175441045963222, z: 89.18}, orientation: {x: -0.0033794693663022498, y: 0.003372215997922873, z: 0.24660542673218327, w: 0.9691042105224306}}}\""
echo "  2. 等待车辆到达终点并停稳"
echo "  3. Ctrl+C 停止录包"
echo ""

ros2 bag record -o "${OUT_DIR}" "${TOPICS[@]}"
