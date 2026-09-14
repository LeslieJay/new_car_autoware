#!/usr/bin/env bash
set -e

# Run one isolated Simple Avoidance case per simulator process.  The process
# restart is deliberate: the multi-object tracker otherwise retains a target
# UUID from the previous distance and can corrupt the first-lock measurement.

ROOT="/home/nvidia/autoware"
DRIVER="${ROOT}/src/byd/obstacle_avoidance_limit_test/scripts/simple_avoidance_distance_test.py"
DATE_DIR="${ROOT}/log/$(date +%Y%m%d)"
LOCK_MODE="${1:-start_lock}"
OUT="${2:-${DATE_DIR}/simple_avoidance_min_lock_distance/${LOCK_MODE}}"
BASE_DOMAIN="${ROS_DOMAIN_ID:-70}"
# Keep the test map configurable and aligned with the current
# planning_simulator default.  Override these three variables when switching
# maps instead of silently testing a stale lanelet file.
MAP_PATH="${MAP_PATH:-/home/nvidia/autoware_map/3_test/}"
LANELET2_MAP_FILE="${LANELET2_MAP_FILE:-0727_lanelet2_map.osm}"
POINTCLOUD_MAP_FILE="${POINTCLOUD_MAP_FILE:-pointcloud_map.pcd}"
# Match the manual planning_simulator workflow: after pose + route, switching
# to autonomous is sufficient for the dummy vehicle to run.  Tests that need
# a deliberately stopped simulator can still override this to false.
INITIAL_ENGAGE_STATE="${INITIAL_ENGAGE_STATE:-true}"
SAMPLE_SEC="${SAMPLE_SEC:-25}"
SETTLE_SEC="${SETTLE_SEC:-3}"
RECORD_ARGS=()
if [[ "${RECORD_BAG:-0}" == "1" ]]; then
  RECORD_ARGS+=(--record-bag)
fi

if [[ "${LOCK_MODE}" != "start_lock" && "${LOCK_MODE}" != "steady_speed_lock" ]]; then
  echo "usage: $0 [start_lock|steady_speed_lock] [output_dir]" >&2
  exit 2
fi

case "${LOCK_MODE}" in
  start_lock)
    DISTANCES=(8 10 12 14 16 18 20)
    ;;
  steady_speed_lock)
    DISTANCES=(12 14 16 18 20)
    ;;
esac

if [[ -n "${DISTANCES_CSV:-}" ]]; then
  IFS=',' read -r -a DISTANCES <<< "${DISTANCES_CSV}"
fi

mkdir -p "${OUT}"
cd "${ROOT}"
source install/setup.bash
set -u

for index in "${!DISTANCES[@]}"; do
  distance="${DISTANCES[${index}]}"
  domain=$((BASE_DOMAIN + index))
  case_dir="${OUT}/distance_${distance}m"
  ros_log_dir="${case_dir}/roslog"
  stdout_log="${case_dir}/launch_stdout.log"
  result_dir="${case_dir}/result"
  mkdir -p "${case_dir}" "${ros_log_dir}"

  export ROS_LOCALHOST_ONLY=1
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export ROS_DOMAIN_ID="${domain}"
  export ROS_LOG_DIR="${ros_log_dir}"

  setsid ros2 launch autoware_launch planning_simulator.launch.xml \
    map_path:="${MAP_PATH}" \
    vehicle_model:=byd_vehicle sensor_model:=byd_sensor_kit \
    lanelet2_map_file:="${LANELET2_MAP_FILE}" \
    pointcloud_map_file:="${POINTCLOUD_MAP_FILE}" \
    launch_didrive_perception:=false scenario_simulation:=false rviz:=false \
    initial_engage_state:=${INITIAL_ENGAGE_STATE} \
    launch_simple_avoidance:=true launch_simple_lc_avoidance:=false \
    launch_obstacle_stop_module:=false launch_dynamic_obstacle_stop_module:=false \
    >"${stdout_log}" 2>&1 &
  launch_pid=$!

  cleanup() {
    kill -INT -- "-${launch_pid}" 2>/dev/null || true
    for _ in $(seq 1 10); do
      kill -0 "${launch_pid}" 2>/dev/null || return 0
      sleep 1
    done
    kill -TERM -- "-${launch_pid}" 2>/dev/null || true
    wait "${launch_pid}" 2>/dev/null || true
  }
  trap cleanup EXIT

  ready=0
  for _ in $(seq 1 150); do
    # The simulator service is created before DDS discovery is complete.  A
    # short `ros2 service list` probe can therefore report a false negative
    # on a large launch.  The driver waits for each service itself; use the
    # planner component load as the non-blocking startup gate here.
    if grep -q "Loaded node '/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner'" "${stdout_log}"; then
      ready=1
      break
    fi
    sleep 1
  done
  if [[ "${ready}" -ne 1 ]]; then
    echo "INVALID startup timeout for distance ${distance} m" | tee "${case_dir}/status.txt"
    cleanup
    trap - EXIT
    continue
  fi
  sleep 5

  launch_log="$(find "${ros_log_dir}" -mindepth 2 -maxdepth 2 -name launch.log -print | head -1)"
  if [[ -z "${launch_log}" ]]; then
    echo "INVALID missing launch.log for distance ${distance} m" | tee "${case_dir}/status.txt"
    cleanup
    trap - EXIT
    continue
  fi

  python3 "${DRIVER}" \
    --mode simple_avoidance \
    --lock-mode "${LOCK_MODE}" \
    --speed 2.0 --distance "${distance}" --intrusion 0.5 --shoulder right \
    --sample-sec "${SAMPLE_SEC}" --settle-sec "${SETTLE_SEC}" \
    --launch-log "${launch_log}" \
    --output-dir "${result_dir}" "${RECORD_ARGS[@]}"

  # ros2 launch removes its log directory on shutdown in some deployments;
  # preserve the exact launch log beside the result before cleanup.
  cp "${launch_log}" "${case_dir}/launch.log" 2>/dev/null || true
  cleanup
  trap - EXIT
done

echo "results written to ${OUT}"
