#!/usr/bin/env bash

set -eo pipefail

WORKSPACE="${AUTOWARE_WORKSPACE:-/home/nvidia/autoware}"
SCRIPT_PATH="${WORKSPACE}/src/byd/scripts/start_bringup_record_mission.sh"
ROSBAG_SCRIPT="${WORKSPACE}/src/byd/scripts/rosbag_record_command.sh"
LOG_ROOT="/mnt/driving_recorder"
MIN_FREE_BYTES=$((200 * 1024 * 1024 * 1024))
STARTUP_TIMEOUT_SECONDS="${STARTUP_TIMEOUT_SECONDS:-60}"
ROSBAG_NODE="${ROSBAG_NODE:-/rosbag2_recorder}"

if [[ ! "${STARTUP_TIMEOUT_SECONDS}" =~ ^[1-9][0-9]*$ ]]; then
  echo "错误：STARTUP_TIMEOUT_SECONDS 必须为正整数。"
  exit 2
fi

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

wait_for_stage_start() {
  local marker_file="$1"
  local stage_name="$2"
  local deadline_epoch
  deadline_epoch=$(($(date +%s) + STARTUP_TIMEOUT_SECONDS))

  until [[ -f "${marker_file}" ]]; do
    if (( $(date +%s) >= deadline_epoch )); then
      echo "错误：等待 ${stage_name} 启动超时（${STARTUP_TIMEOUT_SECONDS} 秒）。"
      return 1
    fi
    sleep 0.2
  done
}

wait_for_ros_node() {
  local node_name="$1"
  local deadline_epoch
  deadline_epoch=$(($(date +%s) + STARTUP_TIMEOUT_SECONDS))

  until ros2 node list 2>/dev/null | grep -Fxq "${node_name}"; do
    if (( $(date +%s) >= deadline_epoch )); then
      echo "错误：等待 ROS 节点 ${node_name} 就绪超时（${STARTUP_TIMEOUT_SECONDS} 秒）。"
      return 1
    fi
    sleep 1
  done
}

wait_for_localization_initialized() {
  local initialization_state

  while true; do
    initialization_state="$(
      timeout 5 ros2 topic echo --once \
        --qos-durability transient_local \
        /api/localization/initialization_state \
        autoware_adapi_v1_msgs/msg/LocalizationInitializationState 2>/dev/null || true
    )"
    if grep -Eq '^[[:space:]]*state:[[:space:]]*3[[:space:]]*$' <<<"${initialization_state}"; then
      return 0
    fi
    sleep 1
  done
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
      mkdir -p "${ROS_LOG_DIR}" "${session_dir}/stages"
      echo "[$(date --iso-8601=seconds)] 启动 BYD bringup" | tee -a "${session_dir}/bringup.log"
      touch "${session_dir}/bringup.started"
      ros2 launch byd_launch parallel_bringup.launch.py \
        log_root:="${session_dir}/stages" 2>&1 | tee -a "${session_dir}/bringup.log" || status=${PIPESTATUS[0]}
      ;;
    rosbag)
      echo "[$(date --iso-8601=seconds)] 启动 rosbag 记录" | tee -a "${session_dir}/rosbag.log"
      touch "${session_dir}/rosbag.started"
      bash "${ROSBAG_SCRIPT}" "${session_dir}" 2>&1 | tee -a "${session_dir}/rosbag.log" || status=${PIPESTATUS[0]}
      ;;
    mission)
      if ! ros2 pkg prefix mission_loop >/dev/null 2>&1; then
        echo "错误：找不到 mission_loop。请先编译并 source 工作空间。" | tee -a "${session_dir}/mission_loop.log"
        wait_before_close 2
      fi
      echo "[$(date --iso-8601=seconds)] 等待初始位姿完成（/api/localization/initialization_state = INITIALIZED）" | tee -a "${session_dir}/mission_loop.log"
      wait_for_localization_initialized
      echo "[$(date --iso-8601=seconds)] 初始位姿已完成" | tee -a "${session_dir}/mission_loop.log"
      echo "[$(date --iso-8601=seconds)] 等待 Autoware 路由服务就绪（/api/routing/set_route_points）" | tee -a "${session_dir}/mission_loop.log"
      until ros2 service list 2>/dev/null | grep -Fxq "/api/routing/set_route_points"; do
        sleep 1
      done
      echo "[$(date --iso-8601=seconds)] 等待 Autoware 开始接受终点（/planning/mission_planning/goal）" | tee -a "${session_dir}/mission_loop.log"
      until ros2 topic info /planning/mission_planning/goal 2>/dev/null |
        grep -Eq "Subscription count: [1-9][0-9]*"; do
        sleep 1
      done
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

if [[ ! -d "${LOG_ROOT}" ]]; then
  echo "错误：存储目录 ${LOG_ROOT} 不存在，请确认磁盘已正确挂载。"
  exit 1
fi

if [[ ! -w "${LOG_ROOT}" ]]; then
  echo "错误：存储目录 ${LOG_ROOT} 不可写，请检查目录权限。"
  exit 1
fi

if ! available_bytes="$(df --output=avail -B1 "${LOG_ROOT}" | tail -n 1 | tr -d '[:space:]')" ||
  [[ ! "${available_bytes}" =~ ^[0-9]+$ ]]; then
  echo "错误：无法获取 ${LOG_ROOT} 的可用存储空间。"
  exit 1
fi

if (( available_bytes < MIN_FREE_BYTES )); then
  available_space="$(df -h --output=avail "${LOG_ROOT}" | tail -n 1 | tr -d '[:space:]')"
  echo "错误：${LOG_ROOT} 可用空间仅 ${available_space}，至少需要 200 GiB，已退出。"
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
wait_for_stage_start "${session_dir}/bringup.started" "parallel bringup"

gnome-terminal --title="ROS Bag Record" -- "${SCRIPT_PATH}" --worker rosbag "${session_dir}"
wait_for_stage_start "${session_dir}/rosbag.started" "rosbag"
setup_ros_environment
echo "等待 rosbag recorder 就绪（${ROSBAG_NODE}）..."
wait_for_ros_node "${ROSBAG_NODE}"

gnome-terminal --title="Mission Loop" -- "${SCRIPT_PATH}" --worker mission "${session_dir}"

echo "已按顺序启动：parallel bringup → rosbag → mission_loop 等待器。"
