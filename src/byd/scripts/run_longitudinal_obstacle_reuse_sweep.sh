#!/usr/bin/env bash
set -Eeuo pipefail

# One planning_simulator instance, six independent obstacle/route cases.
# Do not set ROS_DOMAIN_ID, RMW_IMPLEMENTATION, or ROS_LOCALHOST_ONLY here.

WORKSPACE="/home/nvidia/autoware"
LOG_ROOT="${LOG_ROOT:-${WORKSPACE}/log/20260911/longitudinal_sweep}"
DISTANCE_LIST="${LONGITUDINAL_DISTANCES:-6 8 10 12 15 20}"
read -r -a DISTANCES <<<"${DISTANCE_LIST}"
MAX_OBSERVE_SEC="${MAX_OBSERVE_SEC:-180}"
BLOCKED_SEC="${BLOCKED_SEC:-15}"
MAP_READY_TIMEOUT_SEC="${MAP_READY_TIMEOUT_SEC:-90}"
ROUTE_READY_TIMEOUT_SEC="${ROUTE_READY_TIMEOUT_SEC:-90}"
PLANNING_READY_TIMEOUT_SEC="${PLANNING_READY_TIMEOUT_SEC:-45}"
TRAJECTORY_READY_TIMEOUT_SEC="${TRAJECTORY_READY_TIMEOUT_SEC:-30}"
RUN_ID="$(date +%Y%m%d_%H%M%S)"
RUN_ROOT="${LOG_ROOT}/reuse_${RUN_ID}"
SESSION_DIR="${RUN_ROOT}/session"

LAUNCH_PID=""
LAUNCH_PGID=""
BAG_PID=""
OBSTACLE_PID=""
MONITOR_PID=""

mkdir -p "${SESSION_DIR}"

stop_pid() {
  local pid="${1:-}"
  local signal="${2:-INT}"
  local wait_seconds="${3:-10}"
  [[ -n "${pid}" ]] || return 0
  kill -0 "${pid}" 2>/dev/null || return 0
  kill -"${signal}" "${pid}" 2>/dev/null || true
  for _ in $(seq 1 $((wait_seconds * 10))); do
    kill -0 "${pid}" 2>/dev/null || return 0
    sleep 0.1
  done
  kill -TERM "${pid}" 2>/dev/null || true
  wait "${pid}" 2>/dev/null || true
}

stop_process_group() {
  local pgid="${1:-}"
  local signal="${2:-INT}"
  local wait_seconds="${3:-10}"
  [[ "${pgid}" =~ ^[0-9]+$ ]] || return 0
  if ! ps -eo pgid= | awk -v pgid="${pgid}" '$1 == pgid {found=1} END {exit !found}'; then
    return 0
  fi
  kill -"${signal}" -- "-${pgid}" 2>/dev/null || true
  for _ in $(seq 1 $((wait_seconds * 10))); do
    if ! ps -eo pgid= | awk -v pgid="${pgid}" '$1 == pgid {found=1} END {exit !found}'; then
      return 0
    fi
    sleep 0.1
  done
  kill -TERM -- "-${pgid}" 2>/dev/null || true
}

cleanup() {
  set +e
  stop_pid "${MONITOR_PID}" INT 3
  stop_pid "${OBSTACLE_PID}" INT 15
  stop_pid "${BAG_PID}" INT 10
  stop_process_group "${LAUNCH_PGID}" INT 20
  MONITOR_PID=""
  OBSTACLE_PID=""
  BAG_PID=""
  LAUNCH_PID=""
  LAUNCH_PGID=""
}

trap cleanup EXIT
trap 'exit 130' INT TERM

set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE}/install/setup.bash"
set -u
export ROS_LOG_DIR="${SESSION_DIR}/ros"
mkdir -p "${ROS_LOG_DIR}"

LAUNCH_LOG="${SESSION_DIR}/launch.log"
setsid --wait ros2 launch autoware_launch planning_simulator.launch.xml \
  launch_obstacle_stop_module:=false \
  launch_dynamic_obstacle_stop_module:=false \
  rviz:=false \
  >"${LAUNCH_LOG}" 2>&1 &
LAUNCH_PID=$!
LAUNCH_PGID="${LAUNCH_PID}"

MODULE_READY=0
MODULE_OUTPUT=""
for _ in $(seq 1 90); do
  if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    break
  fi
  MODULE_OUTPUT="$(ros2 param get --no-daemon --include-hidden-nodes --spin-time 3 \
    /planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner \
    launch_modules 2>&1 || true)"
  printf '%s\n' "${MODULE_OUTPUT}" >"${SESSION_DIR}/launch_modules.txt"
  if [[ "${MODULE_OUTPUT}" == *"String values are:"* ]]; then
    MODULE_READY=1
    break
  fi
  sleep 1
done
if [[ "${MODULE_READY}" != 1 ]]; then
  printf '%s\n' "motion_velocity_planner launch_modules 未就绪" >&2
  exit 1
fi
if grep -Eq 'ObstacleStopModule|DynamicObstacleStopModule' "${SESSION_DIR}/launch_modules.txt"; then
  printf '%s\n' "检测到 obstacle stop 模块已加载，停止 sweep" >&2
  exit 1
fi

pause_vehicle() {
  local output_path="$1"
  ros2 service call /control/vehicle_cmd_gate/set_pause \
    tier4_control_msgs/srv/SetPause "{pause: true}" \
    >"${output_path}" 2>&1
}

start_bag() {
  local case_dir="$1"
  ros2 bag record -o "${case_dir}/bag" \
    /simulation/dummy_perception_publisher/object_info \
    /simulation/dummy_perception_publisher/output/debug/ground_truth_objects \
    /simulation/debug/ground_truth_objects \
    /perception/object_recognition/objects \
    /perception/object_recognition/tracking/objects \
    /localization/kinematic_state \
    /vehicle/status/velocity_status \
    /vehicle/status/gear_status \
    /vehicle/status/control_mode \
    /control/current_gate_mode \
    /planning/scenario_planning/lane_driving/behavior_planning/path \
    /planning/scenario_planning/lane_driving/path \
    /planning/scenario_planning/lane_driving/trajectory \
    /planning/scenario_planning/velocity_smoother/trajectory \
    /planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/debug/processing_time_ms \
    /planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/debug/obstacle_slow_down/processing_time_ms \
    /planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/debug/obstacle_cruise/processing_time_ms \
    /planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/debug/out_of_lane/processing_time_ms_diag \
    /planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/debug/processing_time_detail_ms/obstacle_slow_down \
    /planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/debug/processing_time_detail_ms/obstacle_cruise \
    /planning/trajectory \
    /planning/planning_factors/simple_avoidance \
    /planning/planning_factors/obstacle_stop \
    /planning/planning_factors/dynamic_obstacle_stop \
    /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/simple_avoidance \
    >"${case_dir}/bag_record.log" 2>&1 &
  BAG_PID=$!
  sleep 1
  kill -0 "${BAG_PID}" 2>/dev/null
}

stop_bag() {
  stop_pid "${BAG_PID}" INT 10
  BAG_PID=""
}

stop_obstacle() {
  stop_pid "${OBSTACLE_PID}" INT 15
  OBSTACLE_PID=""
}

kill_previous_obstacle_publishers() {
  stop_obstacle
  local stale_pid
  local stale_pids
  stale_pids="$(pgrep -f '/home/nvidia/autoware/src/byd/scripts/place_route_obstacle.py' || true)"
  if [[ -z "${stale_pids}" ]]; then
    printf '%s\n' 'publisher_process_stopped=true (no stale publisher found)'
    return 0
  fi
  while read -r stale_pid; do
    [[ -n "${stale_pid}" ]] || continue
    [[ "${stale_pid}" == "$$" ]] && continue
    kill -INT "${stale_pid}" 2>/dev/null || true
  done <<<"${stale_pids}"
  sleep 0.5
  stale_pids="$(pgrep -f '/home/nvidia/autoware/src/byd/scripts/place_route_obstacle.py' || true)"
  while read -r stale_pid; do
    [[ -n "${stale_pid}" ]] || continue
    [[ "${stale_pid}" == "$$" ]] && continue
    kill -TERM "${stale_pid}" 2>/dev/null || true
  done <<<"${stale_pids}"
  printf '%s\n' 'publisher_process_stopped=true'
}

make_case_window() {
  local case_dir="$1"
  local start_epoch="$2"
  local end_epoch="$3"
  awk -v start="${start_epoch}" -v end="${end_epoch}" '
    {
      if (match($0, /\[[0-9]+\.[0-9]+\]/)) {
        stamp = substr($0, RSTART + 1, RLENGTH - 2)
        if (stamp >= start && stamp <= end) print
      }
    }
  ' "${LAUNCH_LOG}" >"${case_dir}/case_window.log"
  cat "${case_dir}/obstacle.log" "${case_dir}/autonomous.log" \
    >>"${case_dir}/case_window.log" 2>/dev/null || true
}

for distance in "${DISTANCES[@]}"; do
  CASE_DIR="${LOG_ROOT}/${distance}m/reuse_${RUN_ID}"
  mkdir -p "${CASE_DIR}"
  case_start_epoch="$(date +%s.%N)"
  {
    printf 'distance=%sm\n' "${distance}"
    printf 'launch_obstacle_stop_module=false\n'
    printf 'launch_dynamic_obstacle_stop_module=false\n'
    printf 'rviz=false\n'
    printf 'sim_init_and_set_goal.py\n'
    printf 'place_route_obstacle.py --longitudinal %sm --intrusion 0.5 --shoulder right --label unknown --hold 0 --wait-for-subscribers 2 --skip-active-check\n' "${distance}"
    printf 'wait for a fresh simple_avoidance path while vehicle_cmd_gate is paused\n'
    printf 'autonomous_mode_run.sh\n'
  } >"${CASE_DIR}/command.log"

  printf '开始 longitudinal=%sm，复用同一个 Autoware 进程\n' "${distance}"
  kill_previous_obstacle_publishers >"${CASE_DIR}/clear_before_add.log" 2>&1
  if ! python3 "${WORKSPACE}/src/byd/scripts/place_route_obstacle.py" \
    --clear-once >>"${CASE_DIR}/clear_before_add.log" 2>&1; then
    printf '%s\n' "${distance}m 快速 DELETEALL 发送失败，停止 sweep" >&2
    exit 1
  fi
  if ! pause_vehicle "${CASE_DIR}/pause_before.log"; then
    printf '%s\n' "车辆暂停失败，停止 sweep" >&2
    exit 1
  fi
  start_bag "${CASE_DIR}"

  map_ready_log="${CASE_DIR}/map_ready.log"
  printf 'timeout_sec=%s\n' "${MAP_READY_TIMEOUT_SEC}" >"${map_ready_log}"
  if timeout "${MAP_READY_TIMEOUT_SEC}s" ros2 topic echo /map/vector_map --once \
    > /dev/null 2>&1; then
    printf '%s\n' 'vector_map_received=true' >>"${map_ready_log}"
  else
    printf '%s\n' 'vector_map_received=false' >>"${map_ready_log}"
    printf '%s\n' "${distance}m 地图未就绪，停止 sweep" >&2
    stop_bag
    exit 1
  fi

  route_log_start_line="$(wc -l <"${LAUNCH_LOG}")"
  if ! python3 "${WORKSPACE}/src/byd/scripts/sim_init_and_set_goal.py" \
    >>"${CASE_DIR}/command.log" 2>&1; then
    printf '%s\n' "${distance}m 起终点设置失败，停止 sweep" >&2
    stop_bag
    exit 1
  fi
  route_ready=0
  route_ready_log="${CASE_DIR}/route_ready.log"
  {
    printf 'launch_log_start_line=%s\n' "${route_log_start_line}"
    printf 'timeout_sec=%s\n' "${ROUTE_READY_TIMEOUT_SEC}"
  } >"${route_ready_log}"
  for _ in $(seq 1 $((ROUTE_READY_TIMEOUT_SEC * 10))); do
    if tail -n +$((route_log_start_line + 1)) "${LAUNCH_LOG}" 2>/dev/null | \
      rg -q 'Route set via set_waypoint_route'; then
      route_ready=1
      printf '%s\n' 'route_published=true' >>"${route_ready_log}"
      break
    fi
    if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
      break
    fi
    sleep 0.1
  done
  if [[ "${route_ready}" != 1 ]]; then
    printf '%s\n' 'route_published=false' >>"${route_ready_log}"
    printf '%s\n' "${distance}m 起点/终点服务已返回但路线未真正发布，停止 sweep" >&2
    stop_bag
    exit 1
  fi
  if ! pause_vehicle "${CASE_DIR}/pause_after_init.log"; then
    printf '%s\n' "${distance}m 起终点设置后暂停失败，停止 sweep" >&2
    stop_bag
    exit 1
  fi

  # The final /planning/trajectory is normally refreshed before an obstacle is
  # inserted.  While vehicle_cmd_gate is paused, a new avoidance path can be
  # generated without the downstream trajectory being republished immediately;
  # requiring a second message after obstacle insertion falsely rejects a valid
  # test and also leaves the vehicle unable to start.  Establish the planner's
  # baseline readiness here, while the route is already active and no obstacle
  # has been published yet.
  trajectory_ready_log="${CASE_DIR}/trajectory_ready.log"
  {
    printf 'phase=before_obstacle\n'
    printf 'timeout_sec=%s\n' "${TRAJECTORY_READY_TIMEOUT_SEC}"
  } >"${trajectory_ready_log}"
  if timeout "${TRAJECTORY_READY_TIMEOUT_SEC}s" ros2 topic echo /planning/trajectory --once \
    > /dev/null 2>&1; then
    printf '%s\n' 'fresh_trajectory=true' >>"${trajectory_ready_log}"
  else
    printf '%s\n' 'fresh_trajectory=false' >>"${trajectory_ready_log}"
    printf '%s\n' "${distance}m 路线就绪后未收到基线 planning trajectory，停止 sweep" >&2
    stop_bag
    exit 1
  fi

  motion_start_epoch="$(date +%s.%N)"
  avoidance_log_start_line="$(wc -l <"${LAUNCH_LOG}")"

  python3 "${WORKSPACE}/src/byd/scripts/place_route_obstacle.py" \
    --longitudinal "${distance}" \
    --intrusion 0.5 \
    --shoulder right \
    --label unknown \
    --hold 0 \
    --wait-for-subscribers 2 \
    --skip-active-check \
    >"${CASE_DIR}/obstacle.log" 2>&1 &
  OBSTACLE_PID=$!

  obstacle_ready=0
  for _ in $(seq 1 150); do
    if rg -q '发送 DummyObject.ADD|障碍物生效确认成功' "${CASE_DIR}/obstacle.log" 2>/dev/null; then
      obstacle_ready=1
      break
    fi
    if ! kill -0 "${OBSTACLE_PID}" 2>/dev/null; then
      break
    fi
    sleep 0.1
  done
  if [[ "${obstacle_ready}" != 1 ]]; then
    printf '%s\n' "${distance}m 障碍物未确认三条链路均为 1，停止 sweep" >&2
    stop_obstacle
    stop_bag
    exit 1
  fi

  planning_ready=0
  planning_ready_log="${CASE_DIR}/planning_ready.log"
  {
    printf 'launch_log_start_line=%s\n' "${avoidance_log_start_line}"
    printf 'timeout_sec=%s\n' "${PLANNING_READY_TIMEOUT_SEC}"
  } >"${planning_ready_log}"
  for _ in $(seq 1 $((PLANNING_READY_TIMEOUT_SEC * 10))); do
    if tail -n +$((avoidance_log_start_line + 1)) "${LAUNCH_LOG}" 2>/dev/null | \
      rg -q '\[SIMPLE_AVOIDANCE\].*avoidance path generated'; then
      planning_ready=1
      printf '%s\n' 'fresh_avoidance_path=true' >>"${planning_ready_log}"
      break
    fi
    if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
      break
    fi
    sleep 0.1
  done
  if [[ "${planning_ready}" != 1 ]]; then
    printf '%s\n' 'fresh_avoidance_path=false' >>"${planning_ready_log}"
    printf '%s\n' "${distance}m 未在暂停状态下生成新的 simple_avoidance 路径，停止 sweep" >&2
    stop_obstacle
    stop_bag
    exit 1
  fi

  python3 "${WORKSPACE}/src/byd/scripts/monitor_longitudinal_obstacle_case.py" \
    --distance "${distance}" \
    --max-observe-sec "${MAX_OBSERVE_SEC}" \
    --blocked-sec "${BLOCKED_SEC}" \
    --out "${CASE_DIR}/monitor.json" \
    >"${CASE_DIR}/monitor.log" 2>&1 &
  MONITOR_PID=$!

  bash "${WORKSPACE}/src/byd/scripts/autonomous_mode_run.sh" \
    >"${CASE_DIR}/autonomous.log" 2>&1 || true
  set +e
  wait "${MONITOR_PID}"
  monitor_rc=$?
  set -e
  MONITOR_PID=""
  case_window_end_epoch="$(date +%s.%N)"
  printf 'monitor_exit=%s\n' "${monitor_rc}" >>"${CASE_DIR}/command.log"

  ros2 service call /control/vehicle_cmd_gate/set_pause \
    tier4_control_msgs/srv/SetPause "{pause: true}" \
    >"${CASE_DIR}/pause_after.log" 2>&1 || true
  stop_obstacle
  clear_confirmed=1
  printf '%s\n' 'publisher_process_stopped=true' >"${CASE_DIR}/clear_after.log"

  make_case_window "${CASE_DIR}" "${case_start_epoch}" "${case_window_end_epoch}"
  stop_bag

  set +e
  python3 "${WORKSPACE}/src/byd/scripts/analyze_longitudinal_obstacle_case.py" \
    --bag "${CASE_DIR}/bag" \
    --distance "${distance}" \
    --out "${CASE_DIR}/result.json" \
    --monitor "${CASE_DIR}/monitor.json" \
    --motion-start-epoch "${motion_start_epoch}" \
    --clear-confirmed \
    --log "${CASE_DIR}/case_window.log" \
    >"${CASE_DIR}/analysis.log" 2>&1
  set -e
done

python3 "${WORKSPACE}/src/byd/scripts/summarize_longitudinal_obstacle_sweep.py" \
  "${LOG_ROOT}" --run-id "${RUN_ID}" --out "${RUN_ROOT}"
printf 'sweep 完成，结果: %s\n' "${RUN_ROOT}/summary.csv"
