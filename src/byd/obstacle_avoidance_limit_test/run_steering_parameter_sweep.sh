#!/usr/bin/env bash

set -Eeuo pipefail

# Re-run the recorded obstacle window with only the Simple Avoidance shaping
# parameters changed.  This is a diagnostic experiment: it explicitly disables
# lane-change/stop modules and enables the tagged diagnostic logs.

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
BAG_PATH="${ROOT}/log/20260914/110528_bag"
MAP_PATH="${MAP_PATH:-/home/nvidia/autoware_map/3_test/}"
LANELET_MAP_FILE="${LANELET_MAP_FILE:-new_two_lanelet2_map_with_latlon.osm}"
POINTCLOUD_MAP_FILE="${POINTCLOUD_MAP_FILE:-pointcloud_map.pcd}"
OUTPUT_ROOT="${OUTPUT_ROOT:-/tmp/simple_avoidance_steering_sweep_$(date +%Y%m%d_%H%M%S)}"
START_OFFSET="${START_OFFSET:-675}"
PLAYBACK_SECONDS="${PLAYBACK_SECONDS:-20}"
PLAYBACK_RATE="${PLAYBACK_RATE:-1.0}"
BASE_DOMAIN="${BASE_DOMAIN:-210}"
# The captured route is published before the obstacle window.  Replay a short
# route/state pre-roll first so a planner launched for a late window is not left
# waiting forever for a non-latched route topic.
ROUTE_PREROLL_OFFSET="${ROUTE_PREROLL_OFFSET:-0}"
ROUTE_PREROLL_SECONDS="${ROUTE_PREROLL_SECONDS:-30}"
DRY_RUN=0
MAX_CASES=""
JERKS_CSV="0.8,1.2,1.6,2.4"
DISTANCES_CSV="5,4,3"

usage() {
  cat <<'EOF'
Run a diagnostic parameter grid for Simple Avoidance.

Usage:
  run_steering_parameter_sweep.sh [options]

Options:
  --bag PATH              Input rosbag2 directory
  --map-path PATH         Map directory
  --lanelet-map FILE      Lanelet2 map filename
  --pointcloud-map FILE   Pointcloud map filename
  --output-root PATH      Sweep output directory
  --start-offset SEC      Input bag offset to start replaying (default: 675)
  --playback-seconds SEC  Wall-clock replay limit per case (default: 20)
  --rate RATE             ros2 bag play rate (default: 1.0)
  --route-preroll-offset SEC  Route/state pre-roll offset (default: 0)
  --route-preroll-seconds SEC Wall-clock pre-roll duration (default: 30)
  --base-domain ID        First ROS domain ID (default: 210)
  --jerks CSV             Lateral jerk values (default: 0.8,1.2,1.6,2.4)
  --distances CSV         Minimum shifting distances (default: 5,4,3)
  --max-cases N           Stop after N cases, useful for a smoke run
  --dry-run               Print the matrix without launching ROS
  -h, --help              Show this help
EOF
}

die() {
  echo "ERROR: $*" >&2
  exit 1
}

is_number() {
  [[ "$1" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]]
}

as_double() {
  # ros2 param infers an integer for lexical values such as "5" and then exits
  # successfully even though a declared double parameter rejected the update.
  printf '%.6f' "$1"
}

set_and_verify_double_parameter() {
  local node="$1"
  local parameter="$2"
  local requested="$3"
  local value
  value="$(as_double "${requested}")"

  local set_output
  set_output="$(ros2 param set --no-daemon --spin-time 10.0 "${node}" "${parameter}" "${value}" 2>&1)"
  printf '%s\n' "${set_output}"
  [[ "${set_output}" == *"Set parameter successful"* ]] || return 1
}

while (($# > 0)); do
  case "$1" in
    --bag) BAG_PATH="$2"; shift 2 ;;
    --map-path) MAP_PATH="$2"; shift 2 ;;
    --lanelet-map) LANELET_MAP_FILE="$2"; shift 2 ;;
    --pointcloud-map) POINTCLOUD_MAP_FILE="$2"; shift 2 ;;
    --output-root) OUTPUT_ROOT="$2"; shift 2 ;;
    --start-offset) START_OFFSET="$2"; shift 2 ;;
    --playback-seconds) PLAYBACK_SECONDS="$2"; shift 2 ;;
    --rate) PLAYBACK_RATE="$2"; shift 2 ;;
    --route-preroll-offset) ROUTE_PREROLL_OFFSET="$2"; shift 2 ;;
    --route-preroll-seconds) ROUTE_PREROLL_SECONDS="$2"; shift 2 ;;
    --base-domain) BASE_DOMAIN="$2"; shift 2 ;;
    --jerks) JERKS_CSV="$2"; shift 2 ;;
    --distances) DISTANCES_CSV="$2"; shift 2 ;;
    --max-cases) MAX_CASES="$2"; shift 2 ;;
    --dry-run) DRY_RUN=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) die "unknown option: $1" ;;
  esac
done

[[ -d "${BAG_PATH}" && -f "${BAG_PATH}/metadata.yaml" ]] || die "bag not found: ${BAG_PATH}"
[[ -d "${MAP_PATH}" ]] || die "map directory not found: ${MAP_PATH}"
[[ -f "${MAP_PATH%/}/${LANELET_MAP_FILE}" ]] || die "lanelet map not found"
[[ -f "${MAP_PATH%/}/${POINTCLOUD_MAP_FILE}" ]] || die "pointcloud map not found"
is_number "${START_OFFSET}" || die "--start-offset must be numeric"
is_number "${PLAYBACK_SECONDS}" || die "--playback-seconds must be numeric"
is_number "${PLAYBACK_RATE}" || die "--rate must be numeric and positive"
is_number "${ROUTE_PREROLL_OFFSET}" || die "--route-preroll-offset must be numeric"
is_number "${ROUTE_PREROLL_SECONDS}" || die "--route-preroll-seconds must be numeric"

IFS=',' read -r -a JERKS <<< "${JERKS_CSV}"
IFS=',' read -r -a DISTANCES <<< "${DISTANCES_CSV}"
(( ${#JERKS[@]} > 0 && ${#DISTANCES[@]} > 0 )) || die "the parameter lists cannot be empty"

case_count=0
for jerk in "${JERKS[@]}"; do
  for distance in "${DISTANCES[@]}"; do
    is_number "${jerk}" || die "invalid jerk: ${jerk}"
    is_number "${distance}" || die "invalid minimum shifting distance: ${distance}"
    ((case_count += 1))
    if [[ -n "${MAX_CASES}" && "${case_count}" -ge "${MAX_CASES}" ]]; then
      break 2
    fi
  done
done

mkdir -p "${OUTPUT_ROOT}"
printf 'case,jerk,min_shifting_distance\n' >"${OUTPUT_ROOT}/matrix.csv"

if ((DRY_RUN)); then
  dry_case_index=0
  for jerk in "${JERKS[@]}"; do
    for distance in "${DISTANCES[@]}"; do
      ((dry_case_index += 1))
      if [[ -n "${MAX_CASES}" && "${dry_case_index}" -gt "${MAX_CASES}" ]]; then
        break 2
      fi
      case_name="jerk_${jerk}_distance_${distance}"
      printf '%s,%.6g,%.6g\n' "${case_name}" "${jerk}" "${distance}"
      printf '%s,%.6g,%.6g\n' "${case_name}" "${jerk}" "${distance}" >>"${OUTPUT_ROOT}/matrix.csv"
    done
  done
  echo "dry-run output root: ${OUTPUT_ROOT}"
  exit 0
fi

set +u
source /opt/ros/humble/setup.bash
source "${ROOT}/install/setup.bash"
set -u

command -v ros2 >/dev/null || die "ros2 is unavailable after sourcing the workspace"
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION="${SWEEP_RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

NODE="/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner"
INPUT_TOPICS=(
  /tf /tf_static
  /localization/kinematic_state /localization/acceleration
  /perception/object_recognition/objects
  /perception/object_recognition/tracking/objects
  /perception/occupancy_grid_map/map
  /planning/mission_planning/route /planning/mission_planning/state
  /planning/route /planning/route_state
  /vehicle/status/velocity_status /vehicle/status/steering_status
  /vehicle/status/gear_status /vehicle/status/control_mode
  /system/operation_mode/state
)
OUTPUT_TOPICS=(
  /planning/path_candidate/simple_avoidance
  /planning/path_reference/simple_avoidance
  /planning/planning_factors/simple_avoidance
  /planning/scenario_planning/lane_driving/behavior_planning/path
  /planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id
  /planning/scenario_planning/lane_driving/motion_planning/elastic_band_smoother/output/traj
  /planning/scenario_planning/scenario_selector/trajectory
  /planning/trajectory
)

LAUNCH_PID=""
RECORD_PID=""
case_dir=""

stop_group() {
  local pid="$1"
  local signal="${2:-INT}"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
    kill "-${signal}" -- "-${pid}" 2>/dev/null || kill "-${signal}" "${pid}" 2>/dev/null || true
  fi
}

cleanup_case() {
  set +e
  stop_group "${RECORD_PID}" INT
  [[ -z "${RECORD_PID}" ]] || wait "${RECORD_PID}" 2>/dev/null || true
  stop_group "${LAUNCH_PID}" INT
  [[ -z "${LAUNCH_PID}" ]] || wait "${LAUNCH_PID}" 2>/dev/null || true
  RECORD_PID=""
  LAUNCH_PID=""
}

trap cleanup_case EXIT INT TERM

case_index=0
for jerk in "${JERKS[@]}"; do
  for distance in "${DISTANCES[@]}"; do
    ((case_index += 1))
    if [[ -n "${MAX_CASES}" && "${case_index}" -gt "${MAX_CASES}" ]]; then
      break 2
    fi

    case_name="jerk_${jerk}_distance_${distance}"
    case_dir="${OUTPUT_ROOT}/${case_name}"
    ros_log_dir="${case_dir}/ros_logs"
    launch_log="${case_dir}/launch_stdout.log"
    result_bag="${case_dir}/result_bag"
    mkdir -p "${case_dir}" "${ros_log_dir}"
    export ROS_DOMAIN_ID=$((BASE_DOMAIN + case_index))
    export ROS_LOG_DIR="${ros_log_dir}"
    printf '%s,%.6g,%.6g\n' "${case_name}" "${jerk}" "${distance}" >>"${OUTPUT_ROOT}/matrix.csv"

    echo "=== ${case_name} (domain ${ROS_DOMAIN_ID}) ==="
    setsid ros2 launch autoware_launch logging_simulator.launch.xml \
      map_path:="${MAP_PATH}" \
      lanelet2_map_file:="${LANELET_MAP_FILE}" \
      pointcloud_map_file:="${POINTCLOUD_MAP_FILE}" \
      vehicle:=false system:=false sensing:=false localization:=false \
      perception:=false didrive_perception:=false planning:=true control:=false rviz:=false \
      launch_simple_avoidance:=true launch_simple_lc_avoidance:=false \
      launch_obstacle_stop_module:=false launch_dynamic_obstacle_stop_module:=false \
      >"${launch_log}" 2>&1 &
    LAUNCH_PID=$!

    ready=0
    deadline=$((SECONDS + 180))
    while ((SECONDS < deadline)); do
      if grep -q "Loaded node '/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner'" \
        "${launch_log}" 2>/dev/null; then
        ready=1
        break
      fi
      kill -0 "${LAUNCH_PID}" 2>/dev/null || break
      sleep 1
    done
    if ((ready == 0)); then
      echo "INVALID: planner startup timeout" | tee "${case_dir}/status.txt"
      cleanup_case
      continue
    fi

    parameter_log="${case_dir}/parameter.log"
    : >"${parameter_log}"
    if ! set_and_verify_double_parameter "${NODE}" \
      simple_avoidance.shifting_lateral_jerk "${jerk}" >>"${parameter_log}" 2>&1 ||
      ! set_and_verify_double_parameter "${NODE}" \
      simple_avoidance.min_shifting_distance "${distance}" >>"${parameter_log}" 2>&1; then
      echo "INVALID: failed to set diagnostic parameters" | tee "${case_dir}/status.txt"
      cleanup_case
      continue
    fi
    diagnostic_output="$(ros2 param set --no-daemon --spin-time 10.0 "${NODE}" \
      simple_avoidance.publish_steering_diagnostics true 2>&1)"
    printf '%s\n' "${diagnostic_output}" >>"${parameter_log}"
    if [[ "${diagnostic_output}" != *"Set parameter successful"* ]]; then
      echo "INVALID: failed to enable steering diagnostics" | tee "${case_dir}/status.txt"
      cleanup_case
      continue
    fi

    setsid ros2 bag record -o "${result_bag}" /clock "${INPUT_TOPICS[@]}" "${OUTPUT_TOPICS[@]}" \
      >"${case_dir}/record.log" 2>&1 &
    RECORD_PID=$!
    sleep 2

    # Feed the route and route state from the captured pre-roll.  Keep this
    # separate from the obstacle-window playback so every case still receives
    # exactly the same late-window inputs and timing.
    route_preroll_log="${case_dir}/route_preroll.log"
    timeout --signal INT --kill-after 5 "${ROUTE_PREROLL_SECONDS}s" \
      ros2 bag play "${BAG_PATH}" --clock 100 --rate 1.0 \
      --start-offset "${ROUTE_PREROLL_OFFSET}" \
      --topics /planning/mission_planning/route /planning/route \
      /planning/mission_planning/state /planning/route_state \
      /system/operation_mode/state \
      >"${route_preroll_log}" 2>&1 || preroll_status=$?
    printf 'preroll_status=%s\n' "${preroll_status:-0}" >"${case_dir}/route_preroll_status.txt"
    unset preroll_status
    sleep 1

    timeout --signal INT --kill-after 10 "${PLAYBACK_SECONDS}s" \
      ros2 bag play "${BAG_PATH}" --clock 100 --rate "${PLAYBACK_RATE}" \
      --start-offset "${START_OFFSET}" --topics "${INPUT_TOPICS[@]}" \
      >"${case_dir}/playback.log" 2>&1 || playback_status=$?
    playback_status="${playback_status:-0}"
    printf 'playback_status=%s\n' "${playback_status}" >"${case_dir}/status.txt"
    cleanup_case
    unset playback_status
  done
done

python3 "${ROOT}/src/byd/obstacle_avoidance_limit_test/scripts/summarize_avoidance_steering_sweep.py" \
  "${OUTPUT_ROOT}" --output "${OUTPUT_ROOT}/summary.csv" --record-selection first
echo "Sweep artifacts: ${OUTPUT_ROOT}"
