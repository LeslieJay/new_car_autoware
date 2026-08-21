#!/usr/bin/env bash
# 终点停车精度测试数据记录脚本。
# 只记录数据，不发布目标、不修改参数。
# 用法: ./record_stop_accuracy_test.sh [测试名称]

set -eo pipefail

TEST_NAME="${1:-stop_accuracy_$(date +%Y%m%d_%H%M%S)}"
RESULT_ROOT="${RESULT_ROOT:-/home/nvidia/autoware/log}"
CAN_INTERFACE="${CAN_INTERFACE:-can0}"
OUT_DIR="${RESULT_ROOT}/${TEST_NAME}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASELINE_SCRIPT="${SCRIPT_DIR}/verify_forward_mode_baseline.sh"
CAN_PID=""

source /opt/ros/humble/setup.bash
if [[ -f /home/nvidia/autoware/install/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /home/nvidia/autoware/install/setup.bash
fi

stop_can_recording() {
  if [[ -n "${CAN_PID}" ]] && kill -0 "${CAN_PID}" 2>/dev/null; then
    kill -TERM "${CAN_PID}" 2>/dev/null || true
    wait "${CAN_PID}" 2>/dev/null || true
  fi
  CAN_PID=""
}
trap stop_can_recording EXIT INT TERM

for executable in ros2 candump; do
  command -v "${executable}" >/dev/null || {
    printf '缺少命令：%s\n' "${executable}" >&2
    exit 2
  }
done

[[ -f "${BASELINE_SCRIPT}" ]] || {
  printf '缺少模式检查脚本：%s\n' "${BASELINE_SCRIPT}" >&2
  exit 2
}

if [[ -e "${OUT_DIR}" ]]; then
  printf '输出目录已存在，请更换测试名称：%s\n' "${OUT_DIR}" >&2
  exit 2
fi

printf '=== 终点停车测试：环境检查 ===\n'
bash "${BASELINE_SCRIPT}"
ip link show "${CAN_INTERFACE}" >/dev/null 2>&1 || {
  printf 'CAN接口不存在：%s\n' "${CAN_INTERFACE}" >&2
  exit 2
}

TOPICS=(
  /planning/mission_planning/goal
  /planning/trajectory
  /planning/mission_remaining_distance_time
  /planning/mission_planning/state
  /planning/route_state
  /localization/kinematic_state
  /localization/acceleration
  /vehicle/status/velocity_status
  /vehicle/status/gear_status
  /vehicle/status/control_mode
  /control/current_gate_mode
  /control/vehicle_cmd_gate/operation_mode
  /control/vehicle_cmd_gate/is_paused
  /control/vehicle_cmd_gate/is_start_requested
  /control/trajectory_follower/control_cmd
  /control/trajectory_follower/longitudinal/diagnostic
  /control/trajectory_follower/longitudinal/slope_angle
  /control/command/control_cmd
  /can_driver/debug/control_cmd_rx
  /can_driver/debug/control_cmd_can
  /parameter_events
)

REQUIRED_TOPICS=(
  /planning/trajectory
  /localization/kinematic_state
  /vehicle/status/velocity_status
  /control/trajectory_follower/control_cmd
  /control/command/control_cmd
  /can_driver/debug/control_cmd_can
)

AVAILABLE_TOPICS="$(ros2 topic list 2>/dev/null)"
for topic in "${REQUIRED_TOPICS[@]}"; do
  if ! grep -Fxq "${topic}" <<<"${AVAILABLE_TOPICS}"; then
    printf '关键话题不存在，拒绝开始：%s\n' "${topic}" >&2
    exit 2
  fi
done

mkdir -p "${OUT_DIR}"

{
  printf 'test_name=%s\n' "${TEST_NAME}"
  printf 'start_time=%s\n' "$(date --iso-8601=seconds)"
  printf 'can_interface=%s\n' "${CAN_INTERFACE}"
  printf 'hostname=%s\n' "$(hostname)"
} > "${OUT_DIR}/metadata.env"

ros2 node list 2>/dev/null | sort -u > "${OUT_DIR}/ros_nodes.txt" || true
ros2 topic list -t 2>/dev/null > "${OUT_DIR}/ros_topics.txt" || true

# 保存实际运行参数；节点不存在时仅记录警告，不影响核心数据录制。
for node in /control/trajectory_follower/controller_node_exe /can_node /planning/scenario_planning/velocity_smoother; do
  safe_name="${node#/}"
  safe_name="${safe_name//\//_}"
  if ros2 param dump --no-daemon --spin-time 3.0 "${node}" \
      > "${OUT_DIR}/params_${safe_name}.yaml" 2> "${OUT_DIR}/params_${safe_name}.err"; then
    rm -f "${OUT_DIR}/params_${safe_name}.err"
  else
    printf '警告：未能导出节点参数：%s\n' "${node}" >&2
  fi
done

printf '\n=== 准备开始记录 ===\n'
printf '输出目录：%s\n' "${OUT_DIR}"
printf '记录内容：规划轨迹、定位、实际速度、控制指令、纵向诊断、CAN映射和0x201原始帧。\n'
printf '操作要求：从统一起点出发，车辆到终点完全停稳后再等待3秒，然后按 Ctrl+C。\n'
printf '安全要求：仅在封闭场地测试，确保安全员和急停就位。\n'
printf '确认车辆状态和场地安全后按 Enter 开始：'
read -r _

candump -L "${CAN_INTERFACE},201:7FF" > "${OUT_DIR}/can_201.log" 2>&1 &
CAN_PID=$!
sleep 1
if ! kill -0 "${CAN_PID}" 2>/dev/null; then
  printf 'candump启动失败，请检查：%s\n' "${OUT_DIR}/can_201.log" >&2
  exit 2
fi

printf '\n正在记录。现在发送规划目标；停车并稳定3秒后按 Ctrl+C。\n'
printf 'rosbag目录：%s\n\n' "${OUT_DIR}/rosbag"

set +e
ros2 bag record -o "${OUT_DIR}/rosbag" "${TOPICS[@]}"
BAG_STATUS=$?
set -e

stop_can_recording
printf 'end_time=%s\n' "$(date --iso-8601=seconds)" >> "${OUT_DIR}/metadata.env"
printf '\n数据已保存：%s\n' "${OUT_DIR}"
exit "${BAG_STATUS}"
