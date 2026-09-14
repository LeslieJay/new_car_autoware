#!/usr/bin/env bash
set -Eeuo pipefail

# Run the closed-road Simple Lane Change Avoidance acceptance cases. Each case
# gets a fresh planning_simulator and ROS domain so the tracker cannot reuse a
# UUID or a stale module state from a previous scenario.

ROOT="/home/nvidia/autoware"
DRIVER="${ROOT}/src/byd/obstacle_avoidance_limit_test/scripts/simple_avoidance_distance_test.py"
SUMMARY="${ROOT}/src/byd/obstacle_avoidance_limit_test/scripts/summarize_simple_lc_acceptance.py"
MAP_PATH="${MAP_PATH:-/home/nvidia/autoware_map/3_test/}"
LANELET2_MAP_FILE="${LANELET2_MAP_FILE:-0727_lanelet2_map.osm}"
POINTCLOUD_MAP_FILE="${POINTCLOUD_MAP_FILE:-pointcloud_map.pcd}"
OUT="${OUT:-${ROOT}/log/$(date +%Y%m%d)/simple_lc_avoidance_acceptance_$(date +%H%M%S)}"
BASE_DOMAIN="${BASE_DOMAIN:-110}"
STARTUP_TIMEOUT_SEC="${STARTUP_TIMEOUT_SEC:-180}"
SETTLE_SEC="${SETTLE_SEC:-3}"
SAMPLE_SEC="${SAMPLE_SEC:-25}"
CASE_FILTER="${CASE_FILTER:-}"
EGO_X="${LC_ACCEPTANCE_EGO_X:-}"
EGO_Y="${LC_ACCEPTANCE_EGO_Y:-}"
EGO_YAW="${LC_ACCEPTANCE_EGO_YAW:-}"
GOAL_X="${LC_ACCEPTANCE_GOAL_X:-}"
GOAL_Y="${LC_ACCEPTANCE_GOAL_Y:-}"
GOAL_YAW="${LC_ACCEPTANCE_GOAL_YAW:-}"

LAUNCH_PID=""
CASE_DIR=""

cleanup_case() {
  set +e
  if [[ -n "${LAUNCH_PID}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill -INT -- "-${LAUNCH_PID}" 2>/dev/null || kill -INT "${LAUNCH_PID}" 2>/dev/null || true
    for _ in $(seq 1 20); do
      kill -0 "${LAUNCH_PID}" 2>/dev/null || break
      sleep 1
    done
    kill -TERM -- "-${LAUNCH_PID}" 2>/dev/null || kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
  LAUNCH_PID=""
}

trap cleanup_case EXIT INT TERM

set +u
source /opt/ros/humble/setup.bash
source "${ROOT}/install/setup.bash"
set -u
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION="${LC_ACCEPTANCE_RMW:-rmw_fastrtps_cpp}"

mkdir -p "${OUT}"
cd "${ROOT}"

run_case() {
  local name="$1"
  local speed="$2"
  local distance="$3"
  local intrusion="$4"
  local shoulder="$5"
  local scenario="$6"
  local trailer_types="$7"
  local speed_limit="$8"
  local case_index="$9"

  CASE_DIR="${OUT}/scenario_${name}"
  local ros_log_dir="${CASE_DIR}/roslog"
  local launch_stdout="${CASE_DIR}/launch_stdout.log"
  local result_dir="${CASE_DIR}/result"
  local domain=$((BASE_DOMAIN + case_index))
  mkdir -p "${CASE_DIR}" "${ros_log_dir}"

  export ROS_DOMAIN_ID="${domain}"
  export ROS_LOG_DIR="${ros_log_dir}"
  echo "=== ${name}: speed=${speed} distance=${distance} scenario=${scenario} domain=${domain} ==="

  setsid ros2 launch autoware_launch planning_simulator.launch.xml \
    map_path:="${MAP_PATH}" \
    vehicle_model:=byd_vehicle sensor_model:=byd_sensor_kit \
    lanelet2_map_file:="${LANELET2_MAP_FILE}" \
    pointcloud_map_file:="${POINTCLOUD_MAP_FILE}" \
    launch_didrive_perception:=false scenario_simulation:=false rviz:=false \
    initial_engage_state:=true \
    launch_simple_avoidance:=false launch_simple_lc_avoidance:=true \
    launch_obstacle_stop_module:=false launch_dynamic_obstacle_stop_module:=false \
    >"${launch_stdout}" 2>&1 &
  LAUNCH_PID=$!

  local ready=0
  local deadline=$((SECONDS + STARTUP_TIMEOUT_SEC))
  while (( SECONDS < deadline )); do
    # Loading the behavior-planner container is not sufficient: the route
    # handler still has no map during the first several seconds.  Starting
    # the driver at that point races the map/route services and creates a
    # false INVALID result before the module is ever evaluated.
    if grep -q "Loaded node '/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner'" "${launch_stdout}" 2>/dev/null && \
       grep -q "Succeeded to load lanelet2_map" "${launch_stdout}" 2>/dev/null; then
      ready=1
      break
    fi
    if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
      break
    fi
    sleep 1
  done
  if [[ "${ready}" -ne 1 ]]; then
    echo "INVALID startup timeout or planner load failure" | tee "${CASE_DIR}/status.txt"
    cleanup_case
    return 0
  fi
  sleep 5

  local launch_log
  launch_log="$(find "${ros_log_dir}" -type f -name launch.log -print -quit 2>/dev/null || true)"
  local launch_arg=()
  if [[ -n "${launch_log}" ]]; then
    launch_arg=(--launch-log "${launch_log}")
  fi
  local trailer_arg=()
  if [[ -n "${trailer_types}" ]]; then
    trailer_arg=(--trailer-types "${trailer_types}")
  fi
  local speed_limit_arg=()
  if [[ -n "${speed_limit}" ]]; then
    speed_limit_arg=(--speed-limit "${speed_limit}")
  fi
  local pose_arg=()
  if [[ -n "${EGO_X}" ]]; then pose_arg+=(--ego-x "${EGO_X}"); fi
  if [[ -n "${EGO_Y}" ]]; then pose_arg+=(--ego-y "${EGO_Y}"); fi
  if [[ -n "${EGO_YAW}" ]]; then pose_arg+=(--ego-yaw "${EGO_YAW}"); fi
  if [[ -n "${GOAL_X}" ]]; then pose_arg+=(--goal-x "${GOAL_X}"); fi
  if [[ -n "${GOAL_Y}" ]]; then pose_arg+=(--goal-y "${GOAL_Y}"); fi
  if [[ -n "${GOAL_YAW}" ]]; then pose_arg+=(--goal-yaw "${GOAL_YAW}"); fi

  set +e
  python3 "${DRIVER}" \
    --mode simple_lc_avoidance \
    --case-name "${name}" \
    --scenario "${scenario}" \
    --speed "${speed}" --distance "${distance}" --intrusion "${intrusion}" --shoulder "${shoulder}" \
    --settle-sec "${SETTLE_SEC}" --sample-sec "${SAMPLE_SEC}" \
    "${launch_arg[@]}" "${trailer_arg[@]}" "${speed_limit_arg[@]}" "${pose_arg[@]}" \
    --output-dir "${result_dir}" --record-bag
  local driver_rc=$?
  set -e
  printf '%s\n' "driver_exit_code=${driver_rc}" >"${CASE_DIR}/status.txt"
  if [[ -n "${launch_log}" ]]; then
    cp "${launch_log}" "${CASE_DIR}/launch.log" 2>/dev/null || true
  fi
  cleanup_case
}

run_selected() {
  if [[ -z "${CASE_FILTER}" || "${CASE_FILTER}" == "$1" ]]; then
    run_case "$@"
  fi
}

# Fast gate from the plan. The 3/5 m cases are expected SAFE_STOP; all other
# feasible cases are expected SUCCESS unless the scenario deliberately blocks
# or invalidates the articulated footprint.
run_selected success_right 2.0 20.0 0.5 right normal "" "" 0
run_selected success_left 2.0 20.0 0.5 left normal "" "" 1
run_selected safe_stop_3m 2.0 3.0 0.5 right normal "" "" 2
run_selected safe_stop_5m 2.0 5.0 0.5 right normal "" "" 3
run_selected safe_stop_occupied 2.0 20.0 0.5 right adjacent_lane_occupied "" "" 4
run_selected loss_recovery 2.0 20.0 0.5 right target_loss_recovery "" "" 5
run_selected loss_stop 2.0 20.0 0.5 right target_loss_stop "" "" 6
run_selected passed_loss 2.0 20.0 0.5 right target_passed_loss "" "" 7
run_selected trailer_normal 2.0 25.0 0.5 right normal default "" 8
run_selected trailer_wide 2.0 25.0 0.5 right normal wide "" 9
run_selected trailer_articulation 2.0 25.0 0.5 right normal articulation "" 10
run_selected low_speed_success 0.3 15.0 0.5 right normal "" 0.3 11

set +e
python3 "${SUMMARY}" --root "${OUT}" --map-file "${MAP_PATH%/}/${LANELET2_MAP_FILE}"
summary_rc=$?
set -e
echo "acceptance output: ${OUT}"
exit "${summary_rc}"
