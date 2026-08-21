#!/usr/bin/env bash
# Interactive delay_compensation_time calibration through the full Autoware control chain.

set -euo pipefail

DELAYS=(0.15 0.22 0.29 0.36 0.43)
REPETITIONS="${REPETITIONS:-3}"
GOAL_X="${GOAL_X:-261.444300524526}"
GOAL_Y="${GOAL_Y:--28.175441045963222}"
GOAL_Z="${GOAL_Z:-89.18}"
GOAL_QX="${GOAL_QX:--0.0033794693663022498}"
GOAL_QY="${GOAL_QY:-0.003372215997922873}"
GOAL_QZ="${GOAL_QZ:-0.24660542673218327}"
GOAL_QW="${GOAL_QW:-0.9691042105224306}"
RESULT_ROOT="${RESULT_ROOT:-/home/nvidia/autoware/log/delay_compensation_calibration/$(date +%Y%m%d_%H%M%S)}"
CONTROLLER_NODE="${CONTROLLER_NODE:-}"
DELAY_PARAMETER="${DELAY_PARAMETER:-}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASELINE_SCRIPT="${SCRIPT_DIR}/verify_forward_mode_baseline.sh"
ANALYZER="${SCRIPT_DIR}/analyze_delay_compensation_calibration.py"
ORIGINAL_DELAY=""
RECORDER_PID=""

say() { printf '  %s\n' "$1"; }
pause_enter() { printf '  %s ' "$1"; read -r _; }
stage() { printf '\n=== %s ===\n' "$1"; }

stop_recorder() {
  if [[ -n "${RECORDER_PID}" ]] && kill -0 "${RECORDER_PID}" 2>/dev/null; then
    kill -INT "${RECORDER_PID}" 2>/dev/null || true
    wait "${RECORDER_PID}" 2>/dev/null || true
  fi
  RECORDER_PID=""
}

restore_parameter() {
  stop_recorder
  if [[ -n "${ORIGINAL_DELAY}" && -n "${CONTROLLER_NODE}" && -n "${DELAY_PARAMETER}" ]]; then
    ros2 param set "${CONTROLLER_NODE}" "${DELAY_PARAMETER}" "${ORIGINAL_DELAY}" \
      >/dev/null 2>&1 || printf '警告：恢复原延迟参数失败，请手动恢复为 %s\n' "${ORIGINAL_DELAY}" >&2
  fi
}
trap restore_parameter EXIT INT TERM

source /opt/ros/humble/setup.bash
if [[ -f /home/nvidia/autoware/install/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /home/nvidia/autoware/install/setup.bash
fi

find_delay_parameter() {
  local node parameter
  if [[ -n "${CONTROLLER_NODE}" && -n "${DELAY_PARAMETER}" ]]; then
    return
  fi
  if [[ -n "${CONTROLLER_NODE}" || -n "${DELAY_PARAMETER}" ]]; then
    printf 'CONTROLLER_NODE 与 DELAY_PARAMETER 必须同时指定。\n' >&2
    exit 2
  fi
  while IFS= read -r node; do
    while IFS= read -r parameter; do
      parameter="${parameter//[[:space:]]/}"
      if [[ "${parameter}" == *delay_compensation_time ]]; then
        if [[ -n "${CONTROLLER_NODE}" ]]; then
          printf '找到多个延迟参数，请通过 CONTROLLER_NODE 和 DELAY_PARAMETER 指定。\n' >&2
          exit 2
        fi
        CONTROLLER_NODE="${node}"
        DELAY_PARAMETER="${parameter}"
      fi
    done < <(ros2 param list "${node}" 2>/dev/null || true)
  done < <(ros2 node list 2>/dev/null | sort -u)
  if [[ -z "${CONTROLLER_NODE}" ]]; then
    printf '未找到 delay_compensation_time 参数。\n' >&2
    exit 2
  fi
}

set_delay() {
  local value="$1" output actual
  output=$(ros2 param set "${CONTROLLER_NODE}" "${DELAY_PARAMETER}" "${value}")
  [[ "${output}" == *Successful* ]] || {
    printf '参数设置失败：%s\n' "${output}" >&2
    exit 2
  }
  actual=$(ros2 param get "${CONTROLLER_NODE}" "${DELAY_PARAMETER}" | awk '{print $NF}')
  [[ "${actual}" == "${value}" ]] || {
    printf '参数回读不一致：要求=%s，实际=%s\n' "${value}" "${actual}" >&2
    exit 2
  }
}

TOPICS=(
  /planning/trajectory
  /planning/mission_planning/goal
  /localization/kinematic_state
  /vehicle/status/velocity_status
  /control/trajectory_follower/control_cmd
  /control/command/control_cmd
  /can_driver/debug/control_cmd_rx
  /can_driver/debug/control_cmd_can
)

stage "1/4 环境和安全检查"
for executable in ros2 python3 awk; do
  command -v "${executable}" >/dev/null || { printf '缺少命令：%s\n' "${executable}" >&2; exit 2; }
done
[[ -x "${BASELINE_SCRIPT}" ]] || { printf '缺少基线检查脚本：%s\n' "${BASELINE_SCRIPT}" >&2; exit 2; }
bash "${BASELINE_SCRIPT}"
find_delay_parameter
ORIGINAL_DELAY=$(ros2 param get "${CONTROLLER_NODE}" "${DELAY_PARAMETER}" | awk '{print $NF}')
mkdir -p "${RESULT_ROOT}"
say "控制器：${CONTROLLER_NODE}"
say "参数：${DELAY_PARAMETER}"
say "原始值：${ORIGINAL_DELAY}s"
say "结果目录：${RESULT_ROOT}"

stage "2/4 确认固定测试条件"
say "候选延迟：${DELAYS[*]} s；每个重复 ${REPETITIONS} 次。"
say "终点：x=${GOAL_X}, y=${GOAL_Y}, z=${GOAL_Z}"
say "每轮必须从同一起点出发，并保持路线、载荷和场地一致。"
say "确保封闭场地、急停可用且车旁有安全员。"
pause_enter "确认上述终点和安全条件后按 Enter 开始；Ctrl-C退出："

stage "3/4 逐轮测试"
for delay in "${DELAYS[@]}"; do
  for repetition in $(seq 1 "${REPETITIONS}"); do
    run_name="delay_${delay/./p}_rep_${repetition}"
    run_dir="${RESULT_ROOT}/${run_name}"
    mkdir -p "${run_dir}"
    printf '\n--- delay=%ss，重复=%s/%s ---\n' "${delay}" "${repetition}" "${REPETITIONS}"
    pause_enter "车辆回到统一起点并确认安全后按 Enter："
    bash "${BASELINE_SCRIPT}"
    set_delay "${delay}"
    {
      printf 'delay_compensation_time=%s\n' "${delay}"
      printf 'repetition=%s\n' "${repetition}"
      printf 'controller_node=%s\n' "${CONTROLLER_NODE}"
      printf 'delay_parameter=%s\n' "${DELAY_PARAMETER}"
      printf 'goal_x=%s\ngoal_y=%s\ngoal_z=%s\n' "${GOAL_X}" "${GOAL_Y}" "${GOAL_Z}"
    } > "${run_dir}/metadata.env"
    ros2 bag record -o "${run_dir}/rosbag" "${TOPICS[@]}" \
      >"${run_dir}/recorder.log" 2>&1 &
    RECORDER_PID=$!
    sleep 2
    kill -0 "${RECORDER_PID}" 2>/dev/null || {
      printf 'rosbag录制启动失败，请检查 %s\n' "${run_dir}/recorder.log" >&2
      exit 2
    }
    ros2 topic pub --once /planning/mission_planning/goal geometry_msgs/msg/PoseStamped \
      "{header: {frame_id: 'map'}, pose: {position: {x: ${GOAL_X}, y: ${GOAL_Y}, z: ${GOAL_Z}}, orientation: {x: ${GOAL_QX}, y: ${GOAL_QY}, z: ${GOAL_QZ}, w: ${GOAL_QW}}}}"
    pause_enter "车辆完全停稳（速度<0.01m/s持续至少0.5s）后按 Enter 保存本轮："
    stop_recorder
    say "已保存：${run_dir}"
  done
done

stage "4/4 汇总并恢复参数"
python3 "${ANALYZER}" "${RESULT_ROOT}"
set_delay "${ORIGINAL_DELAY}"
ORIGINAL_DELAY=""
say "原参数已恢复。"
say "汇总：${RESULT_ROOT}/delay_calibration_summary.csv"
say "明细：${RESULT_ROOT}/delay_calibration_details.json"
