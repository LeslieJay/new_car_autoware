#!/usr/bin/env bash

# Open-loop planning + control replay. It never replays recorded planning/control outputs.
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/../../.." && pwd)"

BAG_PATH="${WORKSPACE_DIR}/log/20260918/103357_bag"
SOURCE_LOG="${WORKSPACE_DIR}/log/20260918/20260918103349.log"
MAP_PATH="/home/nvidia/autoware_map/3_test/"
LANELET_MAP_FILE="turn_lanelet2_map_with_latlon.osm"
POINTCLOUD_MAP_FILE="pointcloud_map.pcd"
START_OFFSET="450.0"
ROUTE_PRIME_OFFSET="445.0"
PLAYBACK_RATE="1.0"
PLAYBACK_DURATION="140"
OUTPUT_ROOT="/tmp/planning_control_replay"
WAIT_TIMEOUT_SEC=150
REPLAY_DOMAIN_ID=92
RVIZ=false
DRY_RUN=false

usage() {
  sed -n '2,5p' "$0"
  echo "Usage: $0 [--bag PATH] [--source-log PATH] [--map-path PATH]"
  echo "          [--lanelet-map FILE] [--pointcloud-map FILE] [--start-offset SEC]"
  echo "          [--route-prime-offset SEC] [--rate RATE] [--duration SEC] [--output-root PATH]"
  echo "          [--wait-timeout SEC] [--rviz] [--dry-run]"
}

die() { echo "ERROR: $*" >&2; exit 2; }
is_number() { [[ "$1" =~ ^[0-9]+([.][0-9]+)?$ ]]; }

while (($#)); do
  case "$1" in
    --bag) BAG_PATH="$2"; shift 2 ;;
    --source-log) SOURCE_LOG="$2"; shift 2 ;;
    --map-path) MAP_PATH="$2"; shift 2 ;;
    --lanelet-map) LANELET_MAP_FILE="$2"; shift 2 ;;
    --pointcloud-map) POINTCLOUD_MAP_FILE="$2"; shift 2 ;;
    --start-offset) START_OFFSET="$2"; shift 2 ;;
    --route-prime-offset) ROUTE_PRIME_OFFSET="$2"; shift 2 ;;
    --rate) PLAYBACK_RATE="$2"; shift 2 ;;
    --duration) PLAYBACK_DURATION="$2"; shift 2 ;;
    --output-root) OUTPUT_ROOT="$2"; shift 2 ;;
    --wait-timeout) WAIT_TIMEOUT_SEC="$2"; shift 2 ;;
    --rviz) RVIZ=true; shift ;;
    --dry-run) DRY_RUN=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) die "unknown option: $1" ;;
  esac
done

if [[ -d "${BAG_PATH}" && ! -f "${BAG_PATH}/metadata.yaml" ]]; then
  mapfile -t db3_files < <(find "${BAG_PATH}" -maxdepth 1 -type f -name '*.db3' | sort)
  ((${#db3_files[@]} == 1)) || die "bag directory has no metadata.yaml and not exactly one .db3 file: ${BAG_PATH}"
  BAG_URI="${db3_files[0]}"
else
  BAG_URI="${BAG_PATH}"
fi
[[ -d "${BAG_URI}" || -f "${BAG_URI}" ]] || die "bag not found: ${BAG_URI}"
[[ -f "${SOURCE_LOG}" ]] || die "source log not found: ${SOURCE_LOG}"
[[ -f "${MAP_PATH%/}/${LANELET_MAP_FILE}" ]] || die "lanelet map not found"
[[ -f "${MAP_PATH%/}/${POINTCLOUD_MAP_FILE}" ]] || die "pointcloud map not found"
[[ -f "${WORKSPACE_DIR}/install/setup.bash" ]] || die "workspace is not built"
is_number "${START_OFFSET}" || die "--start-offset must be non-negative"
is_number "${ROUTE_PRIME_OFFSET}" || die "--route-prime-offset must be non-negative"
is_number "${PLAYBACK_RATE}" || die "--rate must be positive"
is_number "${PLAYBACK_DURATION}" || die "--duration must be positive"
[[ "${WAIT_TIMEOUT_SEC}" =~ ^[1-9][0-9]*$ ]] || die "--wait-timeout must be a positive integer"

echo "Bag URI          : ${BAG_URI}"
echo "Map              : ${MAP_PATH%/}/${LANELET_MAP_FILE}"
echo "Start offset     : ${START_OFFSET}s"
echo "Route prime      : ${ROUTE_PRIME_OFFSET}s"
echo "Playback rate    : ${PLAYBACK_RATE}"
echo "Playback duration: ${PLAYBACK_DURATION}s wall time"
echo "Mode             : open-loop planning + control"
if ${DRY_RUN}; then
  echo "DRY RUN: inputs validated; no ROS processes started."
  exit 0
fi

RUN_ID="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="${OUTPUT_ROOT%/}/${RUN_ID}"
RESULT_BAG="${RUN_DIR}/planning_control_result"
LAUNCH_LOG="${RUN_DIR}/launch.log"
PARAM_DIR="${RUN_DIR}/effective_parameters"
mkdir -p "${RUN_DIR}" "${PARAM_DIR}" "${RUN_DIR}/ros_logs"

export ROS_LOG_DIR="${RUN_DIR}/ros_logs"
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID="${REPLAY_DOMAIN_ID}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

LAUNCH_PID=""
RECORD_PID=""
cleanup() {
  code=$?
  trap - EXIT INT TERM
  for pid in "${RECORD_PID}" "${LAUNCH_PID}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill -INT -- "-${pid}" 2>/dev/null || true
      wait "${pid}" 2>/dev/null || true
    fi
  done
  echo "Artifacts: ${RUN_DIR}"
  exit "${code}"
}
trap cleanup EXIT INT TERM

setsid ros2 launch autoware_launch logging_simulator.launch.xml \
  map_path:="${MAP_PATH}" lanelet2_map_file:="${LANELET_MAP_FILE}" \
  pointcloud_map_file:="${POINTCLOUD_MAP_FILE}" vehicle:=false system:=false \
  sensing:=false localization:=false perception:=false didrive_perception:=false \
  planning:=true control:=true use_sim_time:=true rviz:="${RVIZ}" \
  >"${LAUNCH_LOG}" 2>&1 &
LAUNCH_PID=$!

required_nodes=(
  /planning/planning_validator
  /control/trajectory_follower/controller_node_exe
  /control/control_validator
  /control/vehicle_cmd_gate
)
deadline=$((SECONDS + WAIT_TIMEOUT_SEC))
while true; do
  nodes="$(ros2 node list --no-daemon 2>/dev/null || true)"
  missing=()
  for node in "${required_nodes[@]}"; do
    grep -Fxq "${node}" <<<"${nodes}" || missing+=("${node}")
  done
  ((${#missing[@]} == 0)) && break
  kill -0 "${LAUNCH_PID}" 2>/dev/null || die "launch exited; inspect ${LAUNCH_LOG}"
  ((SECONDS < deadline)) || die "nodes unavailable after ${WAIT_TIMEOUT_SEC}s: ${missing[*]}"
  sleep 1
done

printf '%s\n' "${nodes}" | sort -u >"${RUN_DIR}/nodes.txt"
while IFS= read -r node; do
  [[ "${node}" == /planning/* || "${node}" == /control/* ]] || continue
  [[ "${node}" != *container* && "${node}" != *transform_listener* && "${node}" != *glog* ]] || continue
  safe_name="${node#/}"
  safe_name="${safe_name//\//__}"
  timeout 8s ros2 param dump "${node}" >"${PARAM_DIR}/${safe_name}.yaml" 2>"${PARAM_DIR}/${safe_name}.stderr" || \
    echo "WARN: parameter dump failed for ${node}" >&2
done <"${RUN_DIR}/nodes.txt"

{
  timeout 8s ros2 param get /planning/planning_validator trajectory_checker.min_lon_accel.threshold
  timeout 8s ros2 param get /control/trajectory_follower/controller_node_exe min_acc
  timeout 8s ros2 param get /control/trajectory_follower/controller_node_exe delay_compensation_time
  timeout 8s ros2 param get /control/control_validator thresholds.assumed_delay_time
  timeout 8s ros2 param get /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner simple_lane_change_avoidance.min_forward_distance
  timeout 8s ros2 param get /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner simple_lane_change_avoidance.stop_margin_before_object
} >"${RUN_DIR}/critical_parameters.txt" 2>&1

OUTPUT_TOPICS=(
  /planning/scenario_planning/trajectory
  /planning/trajectory
  /planning/planning_validator/validation_status
  /planning/planning_validator/virtual_wall
  /planning/planning_factors/simple_avoidance
  /planning/planning_factors/simple_lane_change_avoidance
  /planning/planning_factors/obstacle_stop
  /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/virtual_wall/simple_lane_change_avoidance
  /control/trajectory_follower/control_cmd
  /control/command/control_cmd
  /control/control_validator/validation_status
  /control/control_validator/virtual_wall
  /localization/kinematic_state
  /vehicle/status/velocity_status
  /diagnostics
)
setsid ros2 bag record -o "${RESULT_BAG}" /clock "${OUTPUT_TOPICS[@]}" \
  >"${RUN_DIR}/record.log" 2>&1 &
RECORD_PID=$!
sleep 2
kill -0 "${RECORD_PID}" 2>/dev/null || die "bag recorder failed"

ROUTE_PRIME_BAG="${RUN_DIR}/route_prime"
python3 "${SCRIPT_DIR}/create_route_prime_bag.py" "${BAG_URI}" "${ROUTE_PRIME_BAG}" "${ROUTE_PRIME_OFFSET}"
ros2 bag play "${ROUTE_PRIME_BAG}" --clock 100 --delay 1 --disable-keyboard-controls

INPUT_TOPICS=(
  /tf /tf_static
  /localization/kinematic_state /localization/acceleration
  /perception/object_recognition/objects /perception/object_recognition/tracking/objects
  /perception/occupancy_grid_map/map /perception/obstacle_segmentation/pointcloud
  /planning/mission_planning/route /planning/mission_planning/state /planning/route /planning/route_state
  /vehicle/status/velocity_status /vehicle/status/steering_status /vehicle/status/gear_status
  /vehicle/status/control_mode /vehicle/status/actuation_status
  /system/operation_mode/state /autoware/state
)
set +e
timeout --signal=INT --kill-after=10s "${PLAYBACK_DURATION}s" \
  ros2 bag play "${BAG_URI}" --storage sqlite3 --clock 100 --rate "${PLAYBACK_RATE}" \
    --start-offset "${START_OFFSET}" --delay 2 --disable-keyboard-controls \
    --topics "${INPUT_TOPICS[@]}"
play_rc=$?
set -e
if ((play_rc != 0 && play_rc != 124 && play_rc != 130)); then
  die "rosbag playback failed with exit ${play_rc}"
fi

kill -INT -- "-${RECORD_PID}" 2>/dev/null || true
wait "${RECORD_PID}" 2>/dev/null || true
RECORD_PID=""

python3 "${SCRIPT_DIR}/analyze_planning_control_timeline.py" "${RESULT_BAG}" \
  --log "${LAUNCH_LOG}" --output "${RUN_DIR}/timeline.md" --json "${RUN_DIR}/timeline.json" \
  --require-runtime-chain
python3 "${SCRIPT_DIR}/audit_planning_control_params.py" \
  --workspace "${WORKSPACE_DIR}" --markdown "${RUN_DIR}/static_audit.md" --json "${RUN_DIR}/static_audit.json" || audit_rc=$?
audit_rc="${audit_rc:-0}"
if ((audit_rc == 2)); then
  die "static audit tool failed"
fi

echo "Replay complete. Static audit exit=${audit_rc} (1 means expected P0/P1 findings)."
echo "Timeline: ${RUN_DIR}/timeline.md"
