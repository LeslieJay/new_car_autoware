#!/usr/bin/env bash
set -Eeuo pipefail

# Run isolated planning_simulator cases for longitudinal obstacle distances.

WORKSPACE="/home/nvidia/autoware"
LOG_ROOT="${LOG_ROOT:-${WORKSPACE}/log/20260911/longitudinal_sweep}"
DISTANCES=(6 8 10 12 15 20)
MAX_OBSERVE_SEC="${MAX_OBSERVE_SEC:-180}"
DOMAIN_ID="${LONGITUDINAL_SWEEP_DOMAIN_ID:-77}"
# The desktop shell may export CycloneDDS for other Autoware sessions. Keep
# this sweep isolated on Fast DDS unless the caller explicitly overrides it.
RMW="${LONGITUDINAL_SWEEP_RMW:-rmw_fastrtps_cpp}"

LAUNCH_PID=""
BAG_PID=""
OBSTACLE_PID=""
CURRENT_CASE_DIR=""

cleanup_case() {
  set +e
  if [[ -n "${OBSTACLE_PID}" ]] && kill -0 "${OBSTACLE_PID}" 2>/dev/null; then
    kill -INT "${OBSTACLE_PID}" 2>/dev/null
    wait "${OBSTACLE_PID}" 2>/dev/null
  fi
  if [[ -n "${BAG_PID}" ]] && kill -0 "${BAG_PID}" 2>/dev/null; then
    kill -INT "${BAG_PID}" 2>/dev/null
    sleep 2
    kill -TERM "${BAG_PID}" 2>/dev/null
    wait "${BAG_PID}" 2>/dev/null
  fi
  if [[ -n "${LAUNCH_PID}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill -INT "${LAUNCH_PID}" 2>/dev/null
    for _ in {1..20}; do
      kill -0 "${LAUNCH_PID}" 2>/dev/null || break
      sleep 1
    done
    kill -TERM "${LAUNCH_PID}" 2>/dev/null
    wait "${LAUNCH_PID}" 2>/dev/null
  fi
  LAUNCH_PID=""
  BAG_PID=""
  OBSTACLE_PID=""
}

trap cleanup_case EXIT INT TERM

# ROS setup scripts reference optional variables before declaring them. Load both
# environments before enabling nounset for the test runner itself.
set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${DOMAIN_ID}"
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION="${RMW}"

mkdir -p "${LOG_ROOT}"

for distance in "${DISTANCES[@]}"; do
  CURRENT_CASE_DIR="${LOG_ROOT}/${distance}m"
  mkdir -p "${CURRENT_CASE_DIR}/ros"
  BAG_PATH="${CURRENT_CASE_DIR}/bag"
  if [[ -e "${BAG_PATH}" ]]; then
    BAG_PATH="${CURRENT_CASE_DIR}/bag_$(date +%H%M%S)"
  fi

  export ROS_LOG_DIR="${CURRENT_CASE_DIR}/ros"
  echo "=== longitudinal=${distance}m ==="
  echo "case_dir=${CURRENT_CASE_DIR}"

  ros2 launch autoware_launch planning_simulator.launch.xml \
    launch_obstacle_stop_module:=false \
    launch_dynamic_obstacle_stop_module:=false \
    rviz:=false \
    >"${CURRENT_CASE_DIR}/launch.log" 2>&1 &
  LAUNCH_PID=$!

  MODULE_OUTPUT=""
  MODULE_CHECK_DEADLINE=$((SECONDS + 90))
  while (( SECONDS < MODULE_CHECK_DEADLINE )); do
    MODULE_OUTPUT="$(ros2 param get --no-daemon --include-hidden-nodes --spin-time 3 \
      /planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner \
      launch_modules 2>/dev/null || true)"
    if [[ "${MODULE_OUTPUT}" == *"String values are:"* ]]; then
      break
    fi
    sleep 1
  done
  printf '%s\n' "${MODULE_OUTPUT}" >"${CURRENT_CASE_DIR}/launch_modules.txt"
  if [[ "${MODULE_OUTPUT}" != *"String values are:"* ]]; then
    echo "motion_velocity_planner launch_modules was not available for ${distance}m" >&2
    cleanup_case
    exit 1
  fi
  if [[ "${MODULE_OUTPUT}" == *"ObstacleStopModule"* ]]; then
    echo "obstacle stop module was loaded; aborting case" >&2
    cleanup_case
    exit 1
  fi

  ros2 bag record -o "${BAG_PATH}" \
    /clock /tf /tf_static \
    /simulation/dummy_perception_publisher/object_info \
    /simulation/dummy_perception_publisher/output/debug/ground_truth_objects \
    /perception/object_recognition/objects \
    /perception/object_recognition/tracking/objects \
    /localization/kinematic_state \
    /vehicle/status/velocity_status \
    /vehicle/status/gear_status \
    /vehicle/status/control_mode \
    /control/current_gate_mode \
    /planning/trajectory \
    /planning/planning_factors/simple_avoidance \
    /planning/planning_factors/obstacle_stop \
    /planning/planning_factors/dynamic_obstacle_stop \
    /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/simple_avoidance \
    >"${CURRENT_CASE_DIR}/bag_record.log" 2>&1 &
  BAG_PID=$!
  sleep 3
  if ! kill -0 "${BAG_PID}" 2>/dev/null; then
    echo "rosbag record failed for ${distance}m" >&2
    cleanup_case
    exit 1
  fi

  python3 "${WORKSPACE}/src/byd/scripts/sim_init_and_set_goal.py" \
    >"${CURRENT_CASE_DIR}/sim_init_and_set_goal.log" 2>&1

  python3 "${WORKSPACE}/src/byd/scripts/place_route_obstacle.py" \
    --clear-first \
    --longitudinal "${distance}" \
    --intrusion 0.5 \
    --shoulder right \
    --label unknown \
    --hold 0 \
    >"${CURRENT_CASE_DIR}/place_route_obstacle.log" 2>&1 &
  OBSTACLE_PID=$!
  sleep 3

  bash "${WORKSPACE}/src/byd/scripts/autonomous_mode_run.sh" \
    >"${CURRENT_CASE_DIR}/autonomous_mode_run.log" 2>&1

  echo "observing for ${MAX_OBSERVE_SEC}s"
  sleep "${MAX_OBSERVE_SEC}"

  cleanup_case

  python3 "${WORKSPACE}/src/byd/scripts/analyze_longitudinal_obstacle_case.py" \
    --bag "${BAG_PATH}" \
    --distance "${distance}" \
    --out "${CURRENT_CASE_DIR}/result.json" \
    --log "${CURRENT_CASE_DIR}/launch.log" \
    --log "${CURRENT_CASE_DIR}/autonomous_mode_run.log" \
    >>"${CURRENT_CASE_DIR}/analysis.log" 2>&1 || true
done

python3 - "${LOG_ROOT}" <<'PY'
import csv
import json
import sys
from pathlib import Path

root = Path(sys.argv[1])
rows = []
for result_path in sorted(root.glob("*m/result.json")):
    rows.append(json.loads(result_path.read_text(encoding="utf-8")))

fieldnames = [
    "longitudinal_m",
    "result",
    "object_count",
    "has_simple_avoidance",
    "has_obstacle_stop",
    "has_dynamic_obstacle_stop",
    "vehicle_passed",
    "pre_obstacle_motion",
    "duplicate_ground_truth_uuid",
    "invalid_trajectory_count",
    "mrm_operation_count",
    "bag_path",
]
with (root / "summary.csv").open("w", newline="", encoding="utf-8") as stream:
    writer = csv.DictWriter(stream, fieldnames=fieldnames)
    writer.writeheader()
    for row in rows:
        writer.writerow({key: row.get(key) for key in fieldnames})
print(f"summary={root / 'summary.csv'}")
PY

echo "summary=${LOG_ROOT}/summary.csv"
