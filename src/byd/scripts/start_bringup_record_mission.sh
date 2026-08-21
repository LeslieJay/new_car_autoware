#!/usr/bin/env bash

set -eo pipefail

WORKSPACE="${AUTOWARE_WORKSPACE:-/home/byd/weicanming/github_projects/new_car_autoware}"
SCRIPT_PATH="${WORKSPACE}/src/byd/scripts/start_bringup_record_mission.sh"
ROSBAG_SCRIPT="${WORKSPACE}/src/byd/scripts/rosbag_record_command.sh"
LOG_ROOT="${AUTOWARE_LOG_ROOT:-${HOME}/autoware/log}"

setup_ros_environment() {
  source /opt/ros/humble/setup.bash
  if [[ -f "${WORKSPACE}/install/setup.bash" ]]; then
    source "${WORKSPACE}/install/setup.bash"
  fi
  if [[ -f "${WORKSPACE}/src/byd/mission_publish_ws/install/setup.bash" ]]; then
    source "${WORKSPACE}/src/byd/mission_publish_ws/install/setup.bash"
  fi
}

wait_before_close() {
  local status="$1"
  echo
  echo "进程已退出，状态码：${status}"
  read -r -p "按回车关闭该终端..." _
  exit "${status}"
}

run_worker() {
  local worker="$1"
  local session_dir="$2"
  local status=0

  setup_ros_environment
  cd "${WORKSPACE}"

  case "${worker}" in
    bringup)
      if ! ros2 pkg prefix byd_launch >/dev/null 2>&1; then
        echo "错误：找不到 byd_launch。请先编译并 source 工作空间。" | tee -a "${session_dir}/bringup.log"
        wait_before_close 2
      fi
      export ROS_LOG_DIR="${session_dir}/ros"
      mkdir -p "${ROS_LOG_DIR}"
      echo "[$(date --iso-8601=seconds)] 启动 BYD parallel bringup" | tee -a "${session_dir}/parallel_bringup.log"
      ros2 launch byd_launch parallel_bringup.launch.py \
        log_root:="${session_dir}" 2>&1 | tee -a "${session_dir}/parallel_bringup.log" || status=${PIPESTATUS[0]}
      ;;
    rosbag)
      echo "[$(date --iso-8601=seconds)] 启动 rosbag 记录" | tee -a "${session_dir}/rosbag.log"
      bash "${ROSBAG_SCRIPT}" "${session_dir}" 2>&1 | tee -a "${session_dir}/rosbag.log" || status=${PIPESTATUS[0]}
      ;;
    mission)
      if ! ros2 pkg prefix mission_loop >/dev/null 2>&1; then
        echo "错误：找不到 mission_loop。请先编译并 source 工作空间。" | tee -a "${session_dir}/mission_loop.log"
        wait_before_close 2
      fi
      echo "[$(date --iso-8601=seconds)] 启动 mission_loop" | tee -a "${session_dir}/mission_loop.log"
      ros2 launch mission_loop mission_loop.launch.py 2>&1 | tee -a "${session_dir}/mission_loop.log" || status=${PIPESTATUS[0]}
      ;;
    *)
      echo "未知工作模式：${worker}"
      exit 2
      ;;
  esac

  wait_before_close "${status}"
}

if [[ "${1:-}" == "--worker" ]]; then
  if [[ "$#" -ne 3 ]]; then
    echo "用法：$0 --worker <bringup|rosbag|mission> <session_dir>"
    exit 2
  fi
  run_worker "$2" "$3"
fi

if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo "错误：未找到 gnome-terminal，无法按要求打开独立终端。"
  exit 1
fi

today_dir="${LOG_ROOT}/$(date +%Y%m%d)"
session_dir="${today_dir}/$(date +%H%M%S)"
mkdir -p "${session_dir}"

{
  echo "start_time=$(date --iso-8601=seconds)"
  echo "workspace=${WORKSPACE}"
  echo "session_dir=${session_dir}"
} > "${session_dir}/session_info.txt"

echo "本次日志目录：${session_dir}"

gnome-terminal --title="BYD Bringup" -- "${SCRIPT_PATH}" --worker bringup "${session_dir}"
sleep 2
gnome-terminal --title="ROS Bag Record" -- "${SCRIPT_PATH}" --worker rosbag "${session_dir}"
sleep 2
gnome-terminal --title="Mission Loop" -- "${SCRIPT_PATH}" --worker mission "${session_dir}"

echo "已打开三个终端：bringup、rosbag、mission_loop。"
