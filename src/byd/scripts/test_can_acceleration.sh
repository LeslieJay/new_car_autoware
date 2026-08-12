#!/usr/bin/env bash

set -eo pipefail

WORKSPACE="${AUTOWARE_WORKSPACE:-/home/byd/weicanming/github_projects/new_car_autoware}"
CAN_INTERFACE="${CAN_INTERFACE:-can1}"
TEST_SPEED="${TEST_SPEED:-0.2}"
TEST_ACCELERATION="${TEST_ACCELERATION:-0.2}"
TEST_DECELERATION="${TEST_DECELERATION:--0.5}"
ACCEL_DURATION="${ACCEL_DURATION:-3}"
HOLD_DURATION="${HOLD_DURATION:-2}"
STOP_DURATION="${STOP_DURATION:-4}"
RESULT_ROOT="${RESULT_ROOT:-${WORKSPACE}/log/can_acceleration_test}"
RESULT_DIR="${RESULT_ROOT}/$(date +%Y%m%d_%H%M%S)"

mkdir -p "${RESULT_DIR}"

# ROS setup scripts can reference unset variables, so do not enable nounset here.
source /opt/ros/humble/setup.bash
if [[ -f "${WORKSPACE}/install/setup.bash" ]]; then
  source "${WORKSPACE}/install/setup.bash"
fi

BACKGROUND_PIDS=()
TEST_STARTED=0

control_message() {
  local velocity="$1"
  local acceleration="$2"
  printf '{lateral: {steering_tire_angle: 0.0, steering_tire_rotation_rate: 0.0, is_defined_steering_tire_rotation_rate: false}, longitudinal: {velocity: %s, acceleration: %s, jerk: 0.0, is_defined_acceleration: true, is_defined_jerk: false}}' \
    "${velocity}" "${acceleration}"
}

publish_for() {
  local duration="$1"
  local velocity="$2"
  local acceleration="$3"
  timeout --signal=INT "${duration}" \
    ros2 topic pub -r 50 /control/command/control_cmd \
    autoware_control_msgs/msg/Control \
    "$(control_message "${velocity}" "${acceleration}")" >/dev/null 2>&1 || true
}

send_stop() {
  ros2 topic pub --times 5 -r 50 /control/command/control_cmd \
    autoware_control_msgs/msg/Control \
    "$(control_message 0.0 "${TEST_DECELERATION}")" >/dev/null 2>&1 || true
}

cleanup() {
  if [[ "${TEST_STARTED}" -eq 1 ]]; then
    send_stop
  fi
  for pid in "${BACKGROUND_PIDS[@]}"; do
    kill -INT "${pid}" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

cat <<EOF
CAN 电机转速步长指令实车测试

车辆将执行：
  1. 前进加速到 ${TEST_SPEED} m/s，加速度命令 ${TEST_ACCELERATION} m/s^2，持续 ${ACCEL_DURATION}s
  2. 保持 ${TEST_SPEED} m/s，持续 ${HOLD_DURATION}s
  3. 减速到 0 m/s，加速度命令 ${TEST_DECELERATION} m/s^2，持续 ${STOP_DURATION}s

请架空驱动轮或确保前方至少 3 米净空，并准备急停。
测试结果保存到：${RESULT_DIR}
EOF

read -r -p "确认安全后输入 RUN 并回车：" confirmation
if [[ "${confirmation}" != "RUN" ]]; then
  echo "未确认，测试取消。"
  exit 1
fi

if ! ros2 topic list | grep -qx '/control/command/control_cmd'; then
  echo "错误：未发现 /control/command/control_cmd，请先启动 can_driver。" | tee "${RESULT_DIR}/error.txt"
  exit 2
fi

{
  echo "start_time=$(date --iso-8601=seconds)"
  echo "workspace=${WORKSPACE}"
  echo "can_interface=${CAN_INTERFACE}"
  echo "test_speed_mps=${TEST_SPEED}"
  echo "acceleration_mps2=${TEST_ACCELERATION}"
  echo "deceleration_mps2=${TEST_DECELERATION}"
  echo "accel_duration_s=${ACCEL_DURATION}"
  echo "hold_duration_s=${HOLD_DURATION}"
  echo "stop_duration_s=${STOP_DURATION}"
} > "${RESULT_DIR}/test_info.txt"

ros2 topic info -v /control/command/control_cmd > "${RESULT_DIR}/control_cmd_topic_info.txt" 2>&1 || true
ros2 topic list -t > "${RESULT_DIR}/topic_list.txt" 2>&1 || true
ros2 param dump /can_node > "${RESULT_DIR}/can_node_params.yaml" 2>&1 || true

ros2 topic echo /can_driver/debug/control_cmd_rx > "${RESULT_DIR}/control_cmd_rx.yaml" 2>&1 &
BACKGROUND_PIDS+=("$!")
ros2 topic echo /can_driver/debug/control_cmd_can > "${RESULT_DIR}/control_cmd_can.yaml" 2>&1 &
BACKGROUND_PIDS+=("$!")
ros2 topic echo /vehicle/status/velocity_status > "${RESULT_DIR}/velocity_status.yaml" 2>&1 &
BACKGROUND_PIDS+=("$!")
ros2 topic echo /vehicle/status/steering_status > "${RESULT_DIR}/steering_status.yaml" 2>&1 &
BACKGROUND_PIDS+=("$!")

if command -v ros2 >/dev/null 2>&1; then
  ros2 bag record -o "${RESULT_DIR}/rosbag" \
    /control/command/control_cmd \
    /can_driver/debug/control_cmd_rx \
    /can_driver/debug/control_cmd_can \
    /vehicle/status/velocity_status \
    /vehicle/status/steering_status > "${RESULT_DIR}/rosbag_record.log" 2>&1 &
  BACKGROUND_PIDS+=("$!")
fi

if command -v candump >/dev/null 2>&1 && ip link show "${CAN_INTERFACE}" >/dev/null 2>&1; then
  candump -t a "${CAN_INTERFACE},201:7FF" > "${RESULT_DIR}/can_201.log" 2>&1 &
  BACKGROUND_PIDS+=("$!")
else
  echo "candump 不可用或 ${CAN_INTERFACE} 不存在，未录制原始 CAN。" > "${RESULT_DIR}/candump_warning.txt"
fi

sleep 1
TEST_STARTED=1

echo "[$(date --iso-8601=seconds)] 设置前进挡" | tee -a "${RESULT_DIR}/timeline.log"
ros2 topic pub --once /control/command/gear_cmd \
  autoware_vehicle_msgs/msg/GearCommand '{command: 2}' >/dev/null

echo "[$(date --iso-8601=seconds)] 加速阶段" | tee -a "${RESULT_DIR}/timeline.log"
publish_for "${ACCEL_DURATION}" "${TEST_SPEED}" "${TEST_ACCELERATION}"

echo "[$(date --iso-8601=seconds)] 匀速阶段" | tee -a "${RESULT_DIR}/timeline.log"
publish_for "${HOLD_DURATION}" "${TEST_SPEED}" 0.0

echo "[$(date --iso-8601=seconds)] 减速停车阶段" | tee -a "${RESULT_DIR}/timeline.log"
publish_for "${STOP_DURATION}" 0.0 "${TEST_DECELERATION}"

send_stop
sleep 1
TEST_STARTED=0
echo "end_time=$(date --iso-8601=seconds)" >> "${RESULT_DIR}/test_info.txt"
echo "测试完成。请将该目录反馈给我：${RESULT_DIR}"
