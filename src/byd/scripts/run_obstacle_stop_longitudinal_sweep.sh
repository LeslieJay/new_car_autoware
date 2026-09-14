#!/usr/bin/env bash
set -Eeuo pipefail

# Run the Obstacle Stop longitudinal acceptance cases directly in the host ROS
# environment.  Every case follows the requested order:
# launch -> init/goal -> autonomous -> stable speed -> obstacle -> observe -> cleanup.

WORKSPACE="/home/nvidia/autoware"
DISTANCE_LIST="${OBSTACLE_STOP_LONGITUDINAL_DISTANCES:-8 10 12 14 16 18 20}"
read -r -a DISTANCES <<<"${DISTANCE_LIST}"
LOG_ROOT="${LOG_ROOT:-${WORKSPACE}/log/$(date +%Y%m%d)/obstacle_stop_longitudinal_sweep_$(date +%H%M%S)}"
MAX_OBSERVE_SEC="${MAX_OBSERVE_SEC:-180}"
BLOCKED_SEC="${BLOCKED_SEC:-15}"
SPEED_TARGET="${SPEED_TARGET:-2.0}"
SPEED_TOLERANCE="${SPEED_TOLERANCE:-0.1}"
SPEED_STABLE_SEC="${SPEED_STABLE_SEC:-3.0}"
PLACE_OBSTACLE="${PLACE_OBSTACLE:-true}"
CONTROL_LABEL="${CONTROL_LABEL:-original_protocol}"
DISABLE_START_PLANNER_FREESPACE="${DISABLE_START_PLANNER_FREESPACE:-0}"
LAUNCH_OBSTACLE_STOP_MODULE="${LAUNCH_OBSTACLE_STOP_MODULE:-true}"
LAUNCH_DYNAMIC_OBSTACLE_STOP_MODULE="${LAUNCH_DYNAMIC_OBSTACLE_STOP_MODULE:-false}"
TARGET_LOST_TIME_THRESHOLD="${TARGET_LOST_TIME_THRESHOLD:-}"
COMMITMENT_DISTANCE_BEFORE_SHIFT_START="${COMMITMENT_DISTANCE_BEFORE_SHIFT_START:-}"

LAUNCH_PID=""
LAUNCH_PGID=""
BAG_PID=""
OBSTACLE_PID=""
MONITOR_PID=""

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
  # Do not block indefinitely in wait(2): ros2 launch can remain alive while
  # a composed child is unwinding.  Poll briefly, then terminate the explicit
  # test PID; the caller separately handles its launch process group.
  for _ in $(seq 1 50); do
    kill -0 "${pid}" 2>/dev/null || { wait "${pid}" 2>/dev/null || true; return 0; }
    sleep 0.1
  done
  kill -KILL "${pid}" 2>/dev/null || true
  wait "${pid}" 2>/dev/null || true
}

stop_process_group() {
  local pgid="${1:-}"
  local signal="${2:-INT}"
  local wait_seconds="${3:-20}"
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
  for _ in $(seq 1 100); do
    if ! ps -eo pgid= | awk -v pgid="${pgid}" '$1 == pgid {found=1} END {exit !found}'; then
      return 0
    fi
    sleep 0.1
  done
  # The PGID is the explicit setsid group created for this one test case.
  # A final group-scoped KILL prevents a half-dead launch from contaminating
  # the next independently launched distance.
  kill -KILL -- "-${pgid}" 2>/dev/null || true
}

cleanup_case() {
  set +e
  stop_pid "${BAG_PID}" INT 10
  stop_pid "${OBSTACLE_PID}" INT 15
  stop_pid "${MONITOR_PID}" INT 10
  # ros2 launch owns the component containers and normally tears them down
  # when its parent receives SIGINT.  Use the direct parent first; the process
  # group is only a fallback for launch variants that leave children behind.
  stop_pid "${LAUNCH_PID}" INT 30
  stop_process_group "${LAUNCH_PGID}" INT 20
  BAG_PID=""
  OBSTACLE_PID=""
  MONITOR_PID=""
  LAUNCH_PID=""
  LAUNCH_PGID=""
}

trap cleanup_case EXIT
trap 'exit 130' INT TERM

set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE}/install/setup.bash"
set -u
# Deliberately inherit ROS_DOMAIN_ID, RMW_IMPLEMENTATION and
# ROS_LOCALHOST_ONLY from the host terminal.  This is a host-mode test, not a
# DDS-isolated run.
export PYTHONUNBUFFERED=1

mkdir -p "${LOG_ROOT}"

echo "preflight: checking test scripts"
bash -n "${WORKSPACE}/src/byd/scripts/autonomous_mode_run.sh"
python3 -m py_compile \
  "${WORKSPACE}/src/byd/scripts/sim_init_and_set_goal.py" \
  "${WORKSPACE}/src/byd/scripts/place_route_obstacle.py" \
  "${WORKSPACE}/src/byd/scripts/monitor_longitudinal_obstacle_case.py" \
  "${WORKSPACE}/src/byd/scripts/wait_vehicle_speed_stable.py" \
  "${WORKSPACE}/src/byd/scripts/wait_trajectory_stable.py" \
  "${WORKSPACE}/src/byd/scripts/analyze_longitudinal_obstacle_case.py" \
  "${WORKSPACE}/src/byd/scripts/check_motion_velocity_planner_modules.py" \
  "${WORKSPACE}/src/byd/scripts/summarize_obstacle_stop_longitudinal_sweep.py"

for distance in "${DISTANCES[@]}"; do
  case_dir="${LOG_ROOT}/${distance}m"
  mkdir -p "${case_dir}/ros"
  export ROS_LOG_DIR="${case_dir}/ros"
  LAUNCH_PID=""
  LAUNCH_PGID=""
  BAG_PID=""
  OBSTACLE_PID=""
  MONITOR_PID=""
  clear_confirmed=0
  case_failed=0
  motion_start_epoch=""
  autonomous_rc=0
  echo "=== obstacle_stop longitudinal=${distance}m ==="
  echo "case_dir=${case_dir}"
  printf '%s\n' \
    "control_label=${CONTROL_LABEL}" \
    "place_obstacle=${PLACE_OBSTACLE}" \
    "launch_obstacle_stop_module=${LAUNCH_OBSTACLE_STOP_MODULE}" \
    "launch_dynamic_obstacle_stop_module=${LAUNCH_DYNAMIC_OBSTACLE_STOP_MODULE}" \
    "disable_start_planner_freespace=${DISABLE_START_PLANNER_FREESPACE}" \
    "target_lost_time_threshold=${TARGET_LOST_TIME_THRESHOLD:-default}" \
    "commitment_distance_before_shift_start=${COMMITMENT_DISTANCE_BEFORE_SHIFT_START:-default}" \
      >"${case_dir}/case_config.txt"

  setsid --wait ros2 launch autoware_launch planning_simulator.launch.xml \
    rviz:=false \
    launch_simple_avoidance:=true \
    launch_obstacle_stop_module:="${LAUNCH_OBSTACLE_STOP_MODULE}" \
    launch_dynamic_obstacle_stop_module:="${LAUNCH_DYNAMIC_OBSTACLE_STOP_MODULE}" \
    >"${case_dir}/launch.log" 2>&1 &
  LAUNCH_PID=$!
  LAUNCH_PGID="${LAUNCH_PID}"

  module_output=""
  module_ready=0
  # Query the parameter via rclpy so a stale ros2 daemon cache cannot make a
  # false-positive or hide a live composed node.
  if module_output="$(python3 -u \
  "${WORKSPACE}/src/byd/scripts/check_motion_velocity_planner_modules.py" \
      --timeout 90 2>"${case_dir}/launch_modules_check.log")"; then
    module_ready=1
  fi
  printf '%s\n' "${module_output}" >"${case_dir}/launch_modules.txt"

  if [[ "${module_ready}" != 1 ]]; then
    printf '%s\n' "launch_modules 未就绪" >"${case_dir}/failure.txt"
    case_failed=1
  elif { [[ "${LAUNCH_OBSTACLE_STOP_MODULE}" == true ]] &&
         ! rg -q 'autoware::motion_velocity_planner::ObstacleStopModule' \
           "${case_dir}/launch_modules.txt"; } ||
       { [[ "${LAUNCH_DYNAMIC_OBSTACLE_STOP_MODULE}" == false ]] &&
         rg -q 'autoware::motion_velocity_planner::DynamicObstacleStopModule' \
           "${case_dir}/launch_modules.txt"; }; then
    printf '%s\n' "模块配置不符合要求" >"${case_dir}/failure.txt"
    case_failed=1
  fi

  if [[ "${case_failed}" == 0 ]]; then
    simple_avoidance_parameter_confirmed=0
    : >"${case_dir}/simple_avoidance_parameter.log"
    for _ in $(seq 1 60); do
      if timeout 3s ros2 param get \
        /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner \
        simple_avoidance.avoidance_start_distance_before_object_front \
        >"${case_dir}/simple_avoidance_parameter.txt" 2>>"${case_dir}/simple_avoidance_parameter.log" && \
        rg -q 'Double value is: (5(\.0+)?)$' "${case_dir}/simple_avoidance_parameter.txt"; then
        simple_avoidance_parameter_confirmed=1
        break
      fi
      sleep 1
    done
    if [[ "${simple_avoidance_parameter_confirmed}" != 1 ]]; then
      printf '%s\n' "Simple Avoidance 起始距离参数未能回读为 5.0" >"${case_dir}/failure.txt"
      case_failed=1
    fi
  fi

  if [[ "${case_failed}" == 0 ]]; then
    : >"${case_dir}/runtime_parameter.log"
    parameter_failed=0
    if [[ -n "${TARGET_LOST_TIME_THRESHOLD}" ]]; then
      if ! timeout 5s ros2 param set \
        /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner \
        simple_avoidance.target_lost_time_threshold "${TARGET_LOST_TIME_THRESHOLD}" \
        >>"${case_dir}/runtime_parameter.log" 2>&1; then parameter_failed=1; fi
    fi
    if [[ -n "${COMMITMENT_DISTANCE_BEFORE_SHIFT_START}" ]]; then
      if ! timeout 5s ros2 param set \
        /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner \
        simple_avoidance.commitment_distance_before_shift_start "${COMMITMENT_DISTANCE_BEFORE_SHIFT_START}" \
        >>"${case_dir}/runtime_parameter.log" 2>&1; then parameter_failed=1; fi
    fi
    expected_target_lost="${TARGET_LOST_TIME_THRESHOLD:-2.0}"
    expected_commitment="${COMMITMENT_DISTANCE_BEFORE_SHIFT_START:-2.0}"
    parameter_confirmed=0
    for _ in $(seq 1 60); do
      target_ok=0
      commitment_ok=0
      if timeout 5s ros2 param get \
          /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner \
          simple_avoidance.target_lost_time_threshold \
          >"${case_dir}/target_lost_time_threshold.txt" 2>>"${case_dir}/runtime_parameter.log" && \
          rg -q "Double value is: ${expected_target_lost}(\.0+)?$" \
          "${case_dir}/target_lost_time_threshold.txt"; then
        target_ok=1
      fi
      if timeout 5s ros2 param get \
          /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner \
          simple_avoidance.commitment_distance_before_shift_start \
          >"${case_dir}/commitment_distance_before_shift_start.txt" 2>>"${case_dir}/runtime_parameter.log" && \
          rg -q "Double value is: ${expected_commitment}(\.0+)?$" \
          "${case_dir}/commitment_distance_before_shift_start.txt"; then
        commitment_ok=1
      fi
      if [[ "${target_ok}" == 1 && "${commitment_ok}" == 1 ]]; then
        parameter_confirmed=1
        break
      fi
      sleep 1
    done
    {
      printf 'expected_target_lost_time_threshold=%s\n' "${expected_target_lost}"
      cat "${case_dir}/target_lost_time_threshold.txt" 2>/dev/null || true
      printf 'expected_commitment_distance_before_shift_start=%s\n' "${expected_commitment}"
      cat "${case_dir}/commitment_distance_before_shift_start.txt" 2>/dev/null || true
    } >>"${case_dir}/runtime_parameter.log"
    if [[ "${parameter_confirmed}" != 1 ]]; then
      parameter_failed=1
    fi
    if [[ "${parameter_failed}" != 0 ]]; then
      printf '%s\n' "运行时 Simple Avoidance 参数设置失败" >"${case_dir}/failure.txt"
      case_failed=1
    fi
  fi

  if [[ "${case_failed}" == 0 ]]; then
    if [[ "${DISABLE_START_PLANNER_FREESPACE}" == 1 ]]; then
      : >"${case_dir}/disable_start_planner_freespace.log"
      parameter_confirmed=0
      # Motion Velocity Planner may be ready before the composed Behavior Path
      # Planner exposes its parameter service.  Retry the complete set/get
      # transaction instead of treating that startup race as a planner result.
      for _ in $(seq 1 60); do
        if timeout 3s ros2 param set \
          /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner \
          start_planner.freespace_planner.enable_freespace_planner false \
          >>"${case_dir}/disable_start_planner_freespace.log" 2>&1 && \
          timeout 3s ros2 param get \
          /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner \
          start_planner.freespace_planner.enable_freespace_planner \
          >"${case_dir}/start_planner_freespace_parameter.txt" 2>&1 && \
          rg -q 'Boolean value is: False' \
          "${case_dir}/start_planner_freespace_parameter.txt"; then
          parameter_confirmed=1
          break
        fi
        sleep 1
      done
      if [[ "${parameter_confirmed}" != 1 ]]; then
        printf '%s\n' "Start Planner freespace 参数未能设置并回读为 False" \
          >"${case_dir}/failure.txt"
        case_failed=1
      fi
    fi
  fi

  if [[ "${case_failed}" == 0 ]]; then
    if ! timeout 120s ros2 service call /control/vehicle_cmd_gate/set_pause \
      tier4_control_msgs/srv/SetPause "{pause: true}" \
      >"${case_dir}/pre_init_pause.log" 2>&1; then
      printf '%s\n' "初始化前无法暂停车辆" >"${case_dir}/failure.txt"
      case_failed=1
    fi
  fi

  if [[ "${case_failed}" == 0 ]]; then
    ros2 bag record -o "${case_dir}/rosbag" \
      /clock /tf /tf_static \
      /simulation/dummy_perception_publisher/object_info \
      /simulation/dummy_perception_publisher/output/debug/ground_truth_objects \
      /simulation/debug/ground_truth_objects \
      /perception/object_recognition/objects \
      /perception/object_recognition/tracking/objects \
      /localization/kinematic_state \
      /vehicle/status/velocity_status /vehicle/status/gear_status \
      /vehicle/status/control_mode /control/current_gate_mode \
      /api/routing/state \
      /planning/trajectory \
      /planning/planning_factors/simple_avoidance \
      /planning/planning_factors/obstacle_stop \
      /planning/planning_factors/dynamic_obstacle_stop \
      /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/simple_avoidance \
      /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/output/path \
      /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/processing_time_detail_ms/start_planner \
      /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/processing_time_detail_ms/simple_avoidance \
      /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/processing_time_detail_ms/behavior_path_planner \
      /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/avoidance_debug_message_array \
      /autoware/state \
      /api/external/get/engage \
      >"${case_dir}/bag_record.log" 2>&1 &
    BAG_PID=$!
    sleep 3
    if ! kill -0 "${BAG_PID}" 2>/dev/null; then
      printf '%s\n' "rosbag 启动失败" >"${case_dir}/failure.txt"
      case_failed=1
    fi
  fi

  if [[ "${case_failed}" == 0 ]]; then
    if ! python3 -u "${WORKSPACE}/src/byd/scripts/sim_init_and_set_goal.py" \
      >"${case_dir}/sim_init_and_set_goal.log" 2>&1; then
      printf '%s\n' "起点/终点设置失败" >"${case_dir}/failure.txt"
      case_failed=1
    else
      motion_start_epoch="$(date +%s.%N)"
      if ! timeout 30s ros2 topic echo /planning/trajectory --once \
          >"${case_dir}/trajectory_ready.log" 2>&1; then
        printf '%s\n' "规划轨迹未就绪" >"${case_dir}/failure.txt"
        case_failed=1
      elif ! python3 -u "${WORKSPACE}/src/byd/scripts/wait_trajectory_stable.py" \
        --stable-sec 2.0 --min-samples 5 --max-gap-sec 0.5 --timeout-sec 30 \
        --out "${case_dir}/trajectory_stable.json" \
        >"${case_dir}/trajectory_stable.log" 2>&1; then
        printf '%s\n' "规划轨迹未连续刷新" >"${case_dir}/failure.txt"
        case_failed=1
      fi
    fi
  fi

  if [[ "${case_failed}" == 0 && "${PLACE_OBSTACLE}" == true ]]; then
    placement_json="${case_dir}/placement.json"
    ready_file="${case_dir}/obstacle_ready.flag"
    rm -f "${placement_json}" "${ready_file}"
    bash "${WORKSPACE}/src/byd/scripts/autonomous_mode_run.sh" \
      >"${case_dir}/autonomous_mode_run.log" 2>&1 || autonomous_rc=$?
    if [[ "${autonomous_rc}" != 0 ]]; then
      printf '%s\n' "车辆启动失败" >"${case_dir}/failure.txt"
      case_failed=1
    elif ! python3 -u "${WORKSPACE}/src/byd/scripts/wait_vehicle_speed_stable.py" \
      --target-speed "${SPEED_TARGET}" --tolerance "${SPEED_TOLERANCE}" \
      --stable-sec "${SPEED_STABLE_SEC}" --timeout-sec 60 \
      --out "${case_dir}/speed_stable.json" \
      >"${case_dir}/speed_stable.log" 2>&1; then
      printf '%s\n' "车辆未能稳定达到 2.0 m/s" >"${case_dir}/failure.txt"
      case_failed=1
    fi
  fi

  if [[ "${case_failed}" == 0 && "${PLACE_OBSTACLE}" == true ]]; then
    python3 -u "${WORKSPACE}/src/byd/scripts/monitor_longitudinal_obstacle_case.py" \
      --distance "${distance}" \
      --max-observe-sec "${MAX_OBSERVE_SEC}" \
      --blocked-sec "${BLOCKED_SEC}" \
      --obstacle-ready-file "${ready_file}" \
      --obstacle-metadata-file "${placement_json}" \
      --out "${case_dir}/monitor.json" \
      >"${case_dir}/monitor.log" 2>&1 &
    MONITOR_PID=$!

    python3 -u "${WORKSPACE}/src/byd/scripts/place_route_obstacle.py" \
      --clear-first \
      --ahead-of-ego "${distance}" \
      --placement-json "${placement_json}" \
      --intrusion 0.5 \
      --shoulder right \
      --label unknown \
      --hold 0 \
      >"${case_dir}/place_route_obstacle.log" 2>&1 &
    OBSTACLE_PID=$!
    obstacle_ready=0
    for _ in $(seq 1 150); do
      if rg -q '障碍物生效确认成功' "${case_dir}/place_route_obstacle.log" 2>/dev/null; then
        obstacle_ready=1
        touch "${ready_file}"
        break
      fi
      kill -0 "${OBSTACLE_PID}" 2>/dev/null || break
      sleep 0.1
    done
    if [[ "${obstacle_ready}" != 1 ]]; then
      printf '%s\n' "障碍物未在三条感知链路中确认生效" >"${case_dir}/failure.txt"
      case_failed=1
      stop_pid "${MONITOR_PID}" INT 10
      MONITOR_PID=""
    fi
    if [[ -s "${placement_json}" ]]; then
      motion_start_epoch="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["add_epoch"])' "${placement_json}")"
    fi
    set +e
    if [[ -n "${MONITOR_PID}" ]]; then
      wait "${MONITOR_PID}"
      monitor_rc=$?
    else
      monitor_rc=4
    fi
    set -e
    MONITOR_PID=""
    printf 'monitor_exit=%s\nautonomous_exit=%s\n' "${monitor_rc}" "${autonomous_rc}" \
      >"${case_dir}/execution_status.txt"
  elif [[ "${case_failed}" == 0 ]]; then
    timeout "${MAX_OBSERVE_SEC}s" bash "${WORKSPACE}/src/byd/scripts/autonomous_mode_run.sh" \
      >"${case_dir}/autonomous_mode_run.log" 2>&1 || autonomous_rc=$?
    printf 'baseline_without_obstacle=1\nautonomous_exit=%s\n' "${autonomous_rc}" \
      >"${case_dir}/execution_status.txt"
  fi

  # Stop recording first, then interrupt the persistent obstacle publisher so
  # its finally block sends DELETEALL and verifies all three perception chains.
  if [[ -n "${BAG_PID}" ]]; then
    stop_pid "${BAG_PID}" INT 10
    BAG_PID=""
  fi
  if [[ -n "${OBSTACLE_PID}" ]]; then
    stop_pid "${OBSTACLE_PID}" INT 15
    if rg -q '障碍物清除确认成功' "${case_dir}/place_route_obstacle.log" 2>/dev/null; then
      clear_confirmed=1
    fi
    OBSTACLE_PID=""
  fi

  if [[ "${case_failed}" != 0 ]]; then
    failure_reason="$(tr '\n' ' ' <"${case_dir}/failure.txt" 2>/dev/null || printf '%s' 'case setup failed')"
    python3 - "${case_dir}/result.json" "${distance}" "${failure_reason}" "${clear_confirmed}" "${CONTROL_LABEL}" <<'PY'
import json
import sys
from pathlib import Path

path = Path(sys.argv[1])
path.write_text(
    json.dumps(
        {
            "longitudinal_m": float(sys.argv[2]),
            "result": "INCOMPLETE",
            "early_stop_reason": sys.argv[3],
            "clear_confirmed": bool(int(sys.argv[4])),
            "bag_path": str(path.parent / "rosbag"),
            "control_label": sys.argv[5],
        },
        ensure_ascii=False,
        indent=2,
    )
    + "\n",
    encoding="utf-8",
)
PY
  else
    if [[ "${PLACE_OBSTACLE}" != true && "${case_failed}" == 0 ]]; then
      python3 - "${case_dir}/result.json" "${distance}" "${autonomous_rc}" "${CONTROL_LABEL}" <<'PY'
import json
import sys
from pathlib import Path

path = Path(sys.argv[1])
path.write_text(json.dumps({
    "longitudinal_m": float(sys.argv[2]),
    "result": "PASS" if int(sys.argv[3]) in (0, 124) else "FAIL",
    "control_label": sys.argv[4],
    "baseline_without_obstacle": True,
    "autonomous_exit": int(sys.argv[3]),
    "bag_path": str(path.parent / "rosbag"),
}, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
PY
      python3 -u "${WORKSPACE}/src/byd/scripts/diagnose_obstacle_stop_timeline.py" \
        --case-dir "${case_dir}" --out "${case_dir}/timeline.json" \
        >"${case_dir}/timeline.log" 2>&1 || true
      cleanup_case
      continue
    fi
    set +e
    python3 -u "${WORKSPACE}/src/byd/scripts/analyze_longitudinal_obstacle_case.py" \
      --bag "${case_dir}/rosbag" \
      --distance "${distance}" \
      --out "${case_dir}/result.json" \
      --monitor "${case_dir}/monitor.json" \
      --motion-start-epoch "${motion_start_epoch}" \
      --clear-confirmed \
      --allow-obstacle-stop \
      --launch-modules "${case_dir}/launch_modules.txt" \
      --log "${case_dir}/launch.log" \
      --log "${case_dir}/place_route_obstacle.log" \
      --log "${case_dir}/autonomous_mode_run.log" \
      >"${case_dir}/analysis.log" 2>&1
    set -e
    python3 - "${case_dir}/result.json" "${clear_confirmed}" "${autonomous_rc}" "${CONTROL_LABEL}" <<'PY'
import json
import sys
from pathlib import Path

path = Path(sys.argv[1])
data = json.loads(path.read_text(encoding="utf-8"))
data["clear_confirmed"] = bool(int(sys.argv[2]))
data["control_label"] = sys.argv[4]
if int(sys.argv[3]) != 0:
    data["result"] = "FAIL"
    data["early_stop_reason"] = "AUTONOMOUS_MODE_RUN_FAILED"
if not data["clear_confirmed"]:
    data["result"] = "FAIL"
    data["early_stop_reason"] = "OBSTACLE_CLEAR_NOT_CONFIRMED"
path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
PY
    python3 -u "${WORKSPACE}/src/byd/scripts/diagnose_obstacle_stop_timeline.py" \
      --case-dir "${case_dir}" \
      --out "${case_dir}/timeline.json" \
      >"${case_dir}/timeline.log" 2>&1 || true
  fi

  cleanup_case
done

set +e
python3 -u "${WORKSPACE}/src/byd/scripts/summarize_obstacle_stop_longitudinal_sweep.py" \
  "${LOG_ROOT}" --distances "${DISTANCES[@]}"
summary_rc=$?
set -e
echo "sweep_root=${LOG_ROOT}"
exit "${summary_rc}"
