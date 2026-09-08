#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/../../.." && pwd)"

BAG_PATH="/home/nvidia/autoware/log/0803/1136_bag"
MAP_PATH="/home/nvidia/autoware_map/3_test/"
LANELET_MAP_FILE="0727_lanelet2_map.osm"
POINTCLOUD_MAP_FILE="pointcloud_map.pcd"
PLAYBACK_RATE="0.5"
START_OFFSET="0.0"
ROUTE_PRIME_OFFSET=""
RVIZ="true"
OUTPUT_ROOT="/tmp/simple_avoidance_replay"
WAIT_TIMEOUT_SEC=120
REPLAY_DOMAIN_ID=91

usage() {
  cat <<'EOF'
Replay recorded localization/perception inputs through the current planning parameters.

Usage:
  replay_simple_avoidance_bag.sh [options]

Options:
  --bag PATH                 Rosbag directory (default: 1136_bag)
  --map-path PATH            Directory containing the map files
  --lanelet-map FILE         Lanelet2 map filename
  --pointcloud-map FILE      Pointcloud map filename
  --rate RATE                Playback rate (default: 0.5)
  --start-offset SEC         Skip this many seconds from bag start
  --route-prime-offset SEC   Publish route topics around this offset before replay
  --output-root PATH         Parent directory for logs/results
  --wait-timeout SEC         Planner startup timeout (default: 120)
  --no-rviz                  Do not start RViz
  -h, --help                 Show this help

Example:
  ./src/byd/replay_simple_avoidance_bag.sh \
    --bag /media/byd/MyDisk1/weicanming/0803/1136_bag \
    --map-path /home/nvidia/autoware_map/3_test/ \
    --rate 0.5
EOF
}

die() {
  echo "ERROR: $*" >&2
  exit 1
}

is_positive_number() {
  [[ "$1" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]] &&
    awk -v value="$1" 'BEGIN { exit !(value > 0) }'
}

while (($# > 0)); do
  case "$1" in
    --bag)
      (($# >= 2)) || die "--bag requires a value"
      BAG_PATH="$2"
      shift 2
      ;;
    --map-path)
      (($# >= 2)) || die "--map-path requires a value"
      MAP_PATH="$2"
      shift 2
      ;;
    --lanelet-map)
      (($# >= 2)) || die "--lanelet-map requires a value"
      LANELET_MAP_FILE="$2"
      shift 2
      ;;
    --pointcloud-map)
      (($# >= 2)) || die "--pointcloud-map requires a value"
      POINTCLOUD_MAP_FILE="$2"
      shift 2
      ;;
    --rate)
      (($# >= 2)) || die "--rate requires a value"
      PLAYBACK_RATE="$2"
      shift 2
      ;;
    --start-offset)
      (($# >= 2)) || die "--start-offset requires a value"
      START_OFFSET="$2"
      shift 2
      ;;
    --route-prime-offset)
      (($# >= 2)) || die "--route-prime-offset requires a value"
      ROUTE_PRIME_OFFSET="$2"
      shift 2
      ;;
    --output-root)
      (($# >= 2)) || die "--output-root requires a value"
      OUTPUT_ROOT="$2"
      shift 2
      ;;
    --wait-timeout)
      (($# >= 2)) || die "--wait-timeout requires a value"
      WAIT_TIMEOUT_SEC="$2"
      shift 2
      ;;
    --no-rviz)
      RVIZ="false"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "unknown option: $1"
      ;;
  esac
done

[[ -d "${BAG_PATH}" ]] || die "bag directory not found: ${BAG_PATH}"
[[ -f "${BAG_PATH}/metadata.yaml" ]] || die "metadata.yaml not found in: ${BAG_PATH}"
[[ -d "${MAP_PATH}" ]] || die "map directory not found: ${MAP_PATH}"
[[ -f "${MAP_PATH%/}/${LANELET_MAP_FILE}" ]] ||
  die "lanelet map not found: ${MAP_PATH%/}/${LANELET_MAP_FILE}"
[[ -f "${MAP_PATH%/}/${POINTCLOUD_MAP_FILE}" ]] ||
  die "pointcloud map not found: ${MAP_PATH%/}/${POINTCLOUD_MAP_FILE}"
[[ -f "${WORKSPACE_DIR}/install/setup.bash" ]] ||
  die "workspace is not built: ${WORKSPACE_DIR}/install/setup.bash is missing"
is_positive_number "${PLAYBACK_RATE}" || die "--rate must be greater than zero"
[[ "${START_OFFSET}" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]] ||
  die "--start-offset must be zero or greater"
if [[ -n "${ROUTE_PRIME_OFFSET}" ]]; then
  [[ "${ROUTE_PRIME_OFFSET}" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]] ||
    die "--route-prime-offset must be zero or greater"
fi
[[ "${WAIT_TIMEOUT_SEC}" =~ ^[1-9][0-9]*$ ]] ||
  die "--wait-timeout must be a positive integer"

# ROS setup scripts may reference unset variables, so temporarily disable nounset.
set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

command -v ros2 >/dev/null || die "ros2 command is unavailable after sourcing the workspace"

RUN_ID="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="${OUTPUT_ROOT%/}/${RUN_ID}"
LAUNCH_LOG="${RUN_DIR}/logging_simulator.log"
NEW_BAG_PATH="${RUN_DIR}/new_planning_result"
mkdir -p "${RUN_DIR}"
export ROS_LOG_DIR="${RUN_DIR}/ros_logs"
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID="${REPLAY_DOMAIN_ID}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
mkdir -p "${ROS_LOG_DIR}"

LAUNCH_PID=""
RECORD_PID=""
MONITOR_PID=""

stop_process_group() {
  local pid="$1"
  local signal="${2:-INT}"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
    kill "-${signal}" -- "-${pid}" 2>/dev/null || true
  fi
}

cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM

  stop_process_group "${RECORD_PID}" INT
  if [[ -n "${RECORD_PID}" ]]; then
    wait "${RECORD_PID}" 2>/dev/null || true
  fi
  stop_process_group "${MONITOR_PID}" TERM
  stop_process_group "${LAUNCH_PID}" INT
  if [[ -n "${LAUNCH_PID}" ]]; then
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi

  echo
  echo "Artifacts: ${RUN_DIR}"
  exit "${exit_code}"
}
trap cleanup EXIT INT TERM

echo "Run directory : ${RUN_DIR}"
echo "Bag           : ${BAG_PATH}"
echo "Map           : ${MAP_PATH}"
echo "Playback rate : ${PLAYBACK_RATE}"
echo
echo "Starting logging_simulator (planning only)..."

setsid ros2 launch autoware_launch logging_simulator.launch.xml \
  map_path:="${MAP_PATH}" \
  lanelet2_map_file:="${LANELET_MAP_FILE}" \
  pointcloud_map_file:="${POINTCLOUD_MAP_FILE}" \
  vehicle:=false \
  system:=false \
  sensing:=false \
  localization:=false \
  perception:=false \
  didrive_perception:=false \
  planning:=true \
  control:=false \
  rviz:="${RVIZ}" \
  >"${LAUNCH_LOG}" 2>&1 &
LAUNCH_PID=$!

# Show only decision-relevant messages while retaining the complete launch log on disk.
setsid bash -c \
  'tail -n 0 -F "$1" | stdbuf -oL grep --line-buffered -E "SIMPLE_(LC_)?AVOIDANCE|infeasible_distance|avoidance path generated|outside drivable area|validateNonEmpty|sudden shift|Invalid Trajectory|surrounding hazard|ERROR|process has died"' \
  _ "${LAUNCH_LOG}" &
MONITOR_PID=$!

PLANNER_NODE="/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner"
echo "Waiting up to ${WAIT_TIMEOUT_SEC}s for ${PLANNER_NODE}..."
deadline=$((SECONDS + WAIT_TIMEOUT_SEC))
until ros2 node list --no-daemon 2>/dev/null | grep -Fxq "${PLANNER_NODE}"; do
  kill -0 "${LAUNCH_PID}" 2>/dev/null ||
    die "logging_simulator exited during startup; inspect ${LAUNCH_LOG}"
  ((SECONDS < deadline)) ||
    die "planner did not start within ${WAIT_TIMEOUT_SEC}s; inspect ${LAUNCH_LOG}"
  sleep 1
done

echo
echo "Loaded simple_avoidance parameters:"
for parameter in \
  simple_avoidance.min_prepare_distance \
  simple_avoidance.min_shifting_distance \
  simple_avoidance.shifting_lateral_jerk; do
  ros2 param get "${PLANNER_NODE}" "${parameter}" ||
    echo "WARN: could not read ${parameter}" >&2
done

echo
echo "Starting result recording: ${NEW_BAG_PATH}"

INPUT_TOPICS=(
  /tf
  /tf_static
  /localization/kinematic_state
  /localization/acceleration
  /perception/object_recognition/objects
  /perception/object_recognition/tracking/objects
  /perception/occupancy_grid_map/map
  /perception/obstacle_segmentation/pointcloud
  /planning/mission_planning/route
  /planning/mission_planning/state
  /planning/route
  /planning/route_state
  /vehicle/status/velocity_status
  /vehicle/status/steering_status
  /vehicle/status/gear_status
  /vehicle/status/control_mode
  /system/operation_mode/state
)

setsid ros2 bag record \
  -o "${NEW_BAG_PATH}" \
  /clock \
  "${INPUT_TOPICS[@]}" \
  /planning/trajectory \
  /planning/planning_validator/validation_status \
  /planning/path_candidate/simple_avoidance \
  /planning/path_reference/simple_avoidance \
  /planning/planning_factors/simple_avoidance \
  /planning/path_candidate/simple_lane_change_avoidance \
  /planning/path_reference/simple_lane_change_avoidance \
  /planning/planning_factors/simple_lane_change_avoidance \
  /planning/planning_factors/obstacle_stop \
  /planning/planning_factors/dynamic_obstacle_stop \
  /planning/planning_factors/surround_obstacle_checker \
  /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/simple_avoidance \
  /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/info/simple_avoidance \
  /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/simple_lane_change_avoidance \
  /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/info/simple_lane_change_avoidance \
  /planning/scenario_planning/lane_driving/motion_planning/path_smoother/path \
  /planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory \
  /planning/scenario_planning/lane_driving/motion_planning/path_optimizer/virtual_wall \
  >"${RUN_DIR}/bag_record.log" 2>&1 &
RECORD_PID=$!
sleep 2
kill -0 "${RECORD_PID}" 2>/dev/null || die "ros2 bag record failed; inspect ${RUN_DIR}/bag_record.log"

if [[ -n "${ROUTE_PRIME_OFFSET}" ]]; then
  echo
  echo "Priming route topics from bag offset ${ROUTE_PRIME_OFFSET}s..."
  ROUTE_PRIME_BAG="${RUN_DIR}/route_prime"
  python3 "${SCRIPT_DIR}/create_route_prime_bag.py" \
    "${BAG_PATH}" "${ROUTE_PRIME_BAG}" "${ROUTE_PRIME_OFFSET}"
  ros2 bag play "${ROUTE_PRIME_BAG}" --clock 100 --delay 1
fi

echo
echo "Replaying planning inputs. Complete launch output: ${LAUNCH_LOG}"
ros2 bag play "${BAG_PATH}" \
  --clock 100 \
  --rate "${PLAYBACK_RATE}" \
  --start-offset "${START_OFFSET}" \
  --delay 2 \
  --topics "${INPUT_TOPICS[@]}"

echo
echo "Playback complete; finalizing result bag..."
stop_process_group "${RECORD_PID}" INT
wait "${RECORD_PID}" 2>/dev/null || true
RECORD_PID=""

generated_count="$(grep -c 'avoidance path generated' "${LAUNCH_LOG}" || true)"
lane_change_generated_count="$(grep -c 'SIMPLE_LC_AVOIDANCE.*lane change path generated' "${LAUNCH_LOG}" || true)"
infeasible_count="$(grep -c 'pass-through reason=infeasible_distance' "${LAUNCH_LOG}" || true)"
realigned_count="$(grep -c 'realigned stale planned base offset' "${LAUNCH_LOG}" || true)"
sudden_shift_count="$(grep -c 'planning trajectory had sudden shift' "${LAUNCH_LOG}" || true)"
invalid_trajectory_count="$(grep -c 'Invalid Trajectory detected' "${LAUNCH_LOG}" || true)"
empty_path_exception_count="$(grep -c 'validateNonEmpty(): Points is empty' "${LAUNCH_LOG}" || true)"
behavior_planner_death_count="$(grep -c 'behavior_planning_container.*process has died' "${LAUNCH_LOG}" || true)"

echo
echo "Replay summary"
echo "  avoidance path generated : ${generated_count}"
echo "  lane-change path generated: ${lane_change_generated_count}"
echo "  infeasible_distance      : ${infeasible_count}"
echo "  stale base realigned     : ${realigned_count}"
echo "  sudden trajectory shift  : ${sudden_shift_count}"
echo "  invalid trajectory       : ${invalid_trajectory_count}"
echo "  empty-path exception     : ${empty_path_exception_count}"
echo "  behavior planner deaths  : ${behavior_planner_death_count}"
echo "  launch log               : ${LAUNCH_LOG}"
echo "  result bag               : ${NEW_BAG_PATH}"

if ((generated_count == 0 && lane_change_generated_count == 0)); then
  echo "RESULT: neither avoidance module generated a path; inspect infeasible reasons above."
  exit 2
fi

if ((lane_change_generated_count == 0)); then
  echo "RESULT: simple_lane_change_avoidance did not generate a path."
  exit 4
fi

if ((empty_path_exception_count > 0 || behavior_planner_death_count > 0)); then
  echo "RESULT: behavior planning crashed during replay."
  exit 5
fi

if ((sudden_shift_count > 0 || invalid_trajectory_count > 0)); then
  if awk -v offset="${START_OFFSET}" 'BEGIN { exit !(offset == 0) }'; then
    echo "RESULT: planning_validator rejected at least one trajectory."
    exit 3
  fi
  echo "WARN: ignored planning_validator transients caused by starting replay at a nonzero offset."
fi

python3 "${SCRIPT_DIR}/validate_simple_avoidance_replay.py" \
  "${NEW_BAG_PATH}" \
  --source-bag "${BAG_PATH}" \
  --analyze-after-seconds 300 \
  --max-outside-drivable-duration 5

echo "RESULT: lane-change avoidance completed without the abnormal-stop failure chain."
