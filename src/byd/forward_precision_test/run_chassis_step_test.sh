#!/usr/bin/env bash
# Low-speed chassis step-response test through vehicle_cmd_gate.

set -Eeuo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WORKSPACE_DIR=$(cd "${SCRIPT_DIR}/../../.." && pwd)
TARGET_SPEED="0.10"
DRIVE_SECONDS="5"
REPETITIONS="3"
OUTPUT_ROOT="${WORKSPACE_DIR}/log/chassis_calibration"
ARMED=false
BAG_PID=""

usage() {
  cat <<'EOF'
Usage:
  run_chassis_step_test.sh --arm [options]

Required:
  --arm                 Confirm that the closed test area and physical E-stop are ready.

Options:
  --speed MPS           Target speed, (0, 0.30], default: 0.10
  --drive-seconds SEC   Duration of each nonzero step, [1, 10], default: 5
  --repetitions N       Number of steps, [1, 10], default: 3
  --output-root DIR     Rosbag parent directory
  -h, --help            Show this help

The script ends in STOP mode and pause=true. It never restores AUTONOMOUS automatically.
EOF
}

while (($#)); do
  case "$1" in
    --arm) ARMED=true; shift ;;
    --speed) TARGET_SPEED=${2:?missing value for --speed}; shift 2 ;;
    --drive-seconds) DRIVE_SECONDS=${2:?missing value for --drive-seconds}; shift 2 ;;
    --repetitions) REPETITIONS=${2:?missing value for --repetitions}; shift 2 ;;
    --output-root) OUTPUT_ROOT=${2:?missing value for --output-root}; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ "${ARMED}" != true ]]; then
  echo "REFUSED: use --arm only after clearing the test area and checking the physical E-stop." >&2
  exit 2
fi

python3 - "${TARGET_SPEED}" "${DRIVE_SECONDS}" "${REPETITIONS}" <<'PY'
import sys
speed, duration = map(float, sys.argv[1:3])
try:
    repetitions = int(sys.argv[3])
except ValueError:
    raise SystemExit("--repetitions must be an integer") from None
if not 0.0 < speed <= 0.30:
    raise SystemExit("--speed must be in (0, 0.30] m/s")
if not 1.0 <= duration <= 10.0:
    raise SystemExit("--drive-seconds must be in [1, 10]")
if not 1 <= repetitions <= 10:
    raise SystemExit("--repetitions must be an integer in [1, 10]")
PY

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [[ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  echo "Missing ${WORKSPACE_DIR}/install/setup.bash; build the workspace first." >&2
  exit 1
fi
# shellcheck disable=SC1091
source "${WORKSPACE_DIR}/install/setup.bash"

control_message() {
  local velocity=$1
  printf '{lateral: {steering_tire_angle: 0.0, steering_tire_rotation_rate: 0.0, is_defined_steering_tire_rotation_rate: false}, longitudinal: {velocity: %s, acceleration: 0.0, jerk: 0.0, is_defined_acceleration: false, is_defined_jerk: false}}' "${velocity}"
}

publish_for() {
  local velocity=$1
  local seconds=$2
  timeout --signal=INT "${seconds}" ros2 topic pub -r 20 \
    /external/selected/control_cmd autoware_control_msgs/msg/Control \
    "$(control_message "${velocity}")" >/dev/null 2>&1 || true
}

stop_and_hold() {
  echo "[safety] publishing zero velocity for 3 s"
  publish_for 0.0 3
}

cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM
  stop_and_hold || true
  ros2 service call /control/vehicle_cmd_gate/set_pause \
    tier4_control_msgs/srv/SetPause '{pause: true}' >/dev/null 2>&1 || true
  ros2 service call /api/operation_mode/change_to_stop \
    autoware_adapi_v1_msgs/srv/ChangeOperationMode '{}' >/dev/null 2>&1 || true
  if [[ -n "${BAG_PID}" ]] && kill -0 "${BAG_PID}" 2>/dev/null; then
    kill -INT "${BAG_PID}" 2>/dev/null || true
    wait "${BAG_PID}" 2>/dev/null || true
  fi
  echo "[safety] test ended in zero command, pause=true, operation mode STOP"
  exit "${exit_code}"
}
trap cleanup EXIT INT TERM

required_services=(
  /api/operation_mode/change_to_local
  /api/operation_mode/enable_autoware_control
  /api/operation_mode/change_to_stop
  /api/autoware/set/engage
  /control/vehicle_cmd_gate/set_pause
)
available_services=$(ros2 service list)
for service in "${required_services[@]}"; do
  if ! grep -qxF "${service}" <<<"${available_services}"; then
    echo "Required service is unavailable: ${service}" >&2
    exit 1
  fi
done

if ! ros2 topic list | grep -qxF /vehicle/status/velocity_status; then
  echo "Missing /vehicle/status/velocity_status; refusing an unobservable motion test." >&2
  exit 1
fi

echo "Test configuration: speed=${TARGET_SPEED}m/s duration=${DRIVE_SECONDS}s repetitions=${REPETITIONS}"
read -r -p "Type RUN to move the vehicle: " confirmation
if [[ "${confirmation}" != RUN ]]; then
  echo "Cancelled."
  exit 2
fi

echo "[mode] changing to LOCAL external control"
ros2 service call /api/operation_mode/change_to_local \
  autoware_adapi_v1_msgs/srv/ChangeOperationMode '{}' >/dev/null
ros2 service call /api/operation_mode/enable_autoware_control \
  autoware_adapi_v1_msgs/srv/ChangeOperationMode '{}' >/dev/null
ros2 service call /api/autoware/set/engage \
  tier4_external_api_msgs/srv/Engage '{engage: true}' >/dev/null
ros2 service call /control/vehicle_cmd_gate/set_pause \
  tier4_control_msgs/srv/SetPause '{pause: false}' >/dev/null

ros2 topic pub --once /external/selected/gear_cmd \
  autoware_vehicle_msgs/msg/GearCommand \
  '{stamp: {sec: 0, nanosec: 0}, command: 2}' >/dev/null

sleep 1
gate_mode=$(timeout 5 ros2 topic echo /control/current_gate_mode --once 2>/dev/null | awk '/data:/ {print $2; exit}')
if [[ "${gate_mode}" != 1 ]]; then
  echo "Expected EXTERNAL gate_mode=1, got '${gate_mode:-missing}'." >&2
  exit 1
fi
gear_command=$(timeout 5 ros2 topic echo /control/command/gear_cmd --once 2>/dev/null | awk '/command:/ {print $2; exit}')
if [[ "${gear_command}" != 2 ]]; then
  echo "Expected DRIVE gear command=2, got '${gear_command:-missing}'." >&2
  exit 1
fi

mkdir -p "${OUTPUT_ROOT}"
bag_path="${OUTPUT_ROOT}/step_${TARGET_SPEED}mps_$(date +%Y%m%d_%H%M%S)"
echo "[record] ${bag_path}"
ros2 bag record -o "${bag_path}" \
  /external/selected/control_cmd \
  /external/selected/gear_cmd \
  /control/command/control_cmd \
  /control/command/gear_cmd \
  /can_driver/debug/control_cmd_rx \
  /can_driver/debug/control_cmd_can \
  /vehicle/status/velocity_status \
  /vehicle/status/steering_status \
  /vehicle/status/control_mode \
  /vehicle/status/gear_status \
  /localization/kinematic_state \
  /localization/acceleration \
  >"${bag_path}.record.log" 2>&1 &
BAG_PID=$!
sleep 2
if ! kill -0 "${BAG_PID}" 2>/dev/null; then
  echo "ros2 bag record failed; see ${bag_path}.record.log" >&2
  exit 1
fi

stop_and_hold
for ((run = 1; run <= REPETITIONS; run++)); do
  echo "[run ${run}/${REPETITIONS}] velocity=${TARGET_SPEED}m/s for ${DRIVE_SECONDS}s"
  publish_for "${TARGET_SPEED}" "${DRIVE_SECONDS}"
  stop_and_hold
  echo "[run ${run}/${REPETITIONS}] settle for 5 s"
  sleep 5
done

echo "Completed. Bag: ${bag_path}"
