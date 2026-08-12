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
STEPS=()

usage() {
  cat <<'EOF'
Usage:
  run_chassis_step_test.sh --arm [options]

Required:
  --arm                 Confirm that the closed test area and physical E-stop are ready.

Options:
  --speed MPS           Target speed, (0, 4), default: 0.10
  --drive-seconds SEC   Duration of each nonzero step, (0, 60), default: 5
  --step MPS:SEC        Add a speed/duration group; may be specified multiple times
  --repetitions N       Number of steps, [1, 10], default: 3
  --output-root DIR     Rosbag parent directory
  -h, --help            Show this help

If --step is used, --speed and --drive-seconds are ignored. Each group gets its own bag.
EOF
}

while (($#)); do
  case "$1" in
    --arm) ARMED=true; shift ;;
    --speed) TARGET_SPEED=${2:?missing value for --speed}; shift 2 ;;
    --drive-seconds) DRIVE_SECONDS=${2:?missing value for --drive-seconds}; shift 2 ;;
    --step) STEPS+=("${2:?missing value for --step}"); shift 2 ;;
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

if ((${#STEPS[@]} == 0)); then
  STEPS+=("${TARGET_SPEED}:${DRIVE_SECONDS}")
fi

python3 - "${REPETITIONS}" "${STEPS[@]}" <<'PY'
import sys
try:
    repetitions = int(sys.argv[1])
except ValueError:
    raise SystemExit("--repetitions must be an integer") from None
if not 1 <= repetitions <= 10:
    raise SystemExit("--repetitions must be an integer in [1, 10]")
for step in sys.argv[2:]:
    try:
        speed, duration = map(float, step.split(":"))
    except ValueError:
        raise SystemExit(f"invalid step '{step}'; expected MPS:SEC") from None
    if not 0.0 < speed < 4.0:
        raise SystemExit(f"speed must be in (0, 4) m/s: {speed}")
    if not 0.0 < duration < 60.0:
        raise SystemExit(f"drive-seconds must be in (0, 60): {duration}")
PY

# ROS 2/colcon setup scripts probe optional environment variables and are not nounset-safe.
# Temporarily disable `set -u` only while sourcing them, then restore strict mode.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [[ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  set -u
  echo "Missing ${WORKSPACE_DIR}/install/setup.bash; build the workspace first." >&2
  exit 1
fi
# shellcheck disable=SC1091
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

control_message() {
  local velocity=$1
  printf '{lateral: {steering_tire_angle: 0.0, steering_tire_rotation_rate: 0.0, is_defined_steering_tire_rotation_rate: false}, longitudinal: {velocity: %s, acceleration: 0.0, jerk: 0.0, is_defined_acceleration: false, is_defined_jerk: false}}' "${velocity}"
}

publish_for() {
  local velocity=$1
  local seconds=$2
  timeout --signal=INT "${seconds}" ros2 topic pub -r 50 \
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
  if [[ -n "${BAG_PID}" ]] && kill -0 "${BAG_PID}" 2>/dev/null; then
    kill -INT "${BAG_PID}" 2>/dev/null || true
    wait "${BAG_PID}" 2>/dev/null || true
  fi
  echo "[safety] test ended with a zero-velocity command"
  exit "${exit_code}"
}
trap cleanup EXIT INT TERM

if ! ros2 topic list | grep -qxF /vehicle/status/velocity_status; then
  echo "Missing /vehicle/status/velocity_status; refusing an unobservable motion test." >&2
  exit 1
fi

ros2 topic pub --once /external/selected/gear_cmd \
  autoware_vehicle_msgs/msg/GearCommand \
  '{stamp: {sec: 0, nanosec: 0}, command: 2}' >/dev/null

sleep 1
gear_command=$(timeout 5 ros2 topic echo /control/command/gear_cmd --once 2>/dev/null | awk '/command:/ {print $2; exit}')
if [[ "${gear_command}" != 2 ]]; then
  echo "Expected DRIVE gear command=2, got '${gear_command:-missing}'." >&2
  exit 1
fi

mkdir -p "${OUTPUT_ROOT}"
for ((group = 0; group < ${#STEPS[@]}; group++)); do
  IFS=: read -r TARGET_SPEED DRIVE_SECONDS <<<"${STEPS[group]}"
  bag_path="${OUTPUT_ROOT}/step_$((group + 1))_${TARGET_SPEED}mps_${DRIVE_SECONDS}s_$(date +%Y%m%d_%H%M%S)"
  echo "[group $((group + 1))/${#STEPS[@]}] speed=${TARGET_SPEED}m/s duration=${DRIVE_SECONDS}s repetitions=${REPETITIONS}"
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
    >/dev/null 2>&1 &
  BAG_PID=$!
  sleep 2
  if ! kill -0 "${BAG_PID}" 2>/dev/null; then
    echo "ros2 bag record failed for ${bag_path}" >&2
    exit 1
  fi

  stop_and_hold
  for ((run = 1; run <= REPETITIONS; run++)); do
    echo "[group $((group + 1)) run ${run}/${REPETITIONS}] velocity=${TARGET_SPEED}m/s for ${DRIVE_SECONDS}s"
    publish_for "${TARGET_SPEED}" "${DRIVE_SECONDS}"
    stop_and_hold
    echo "[group $((group + 1)) run ${run}/${REPETITIONS}] settle for 5 s"
    sleep 5
  done

  kill -INT "${BAG_PID}" 2>/dev/null || true
  wait "${BAG_PID}" 2>/dev/null || true
  BAG_PID=""
  echo "[completed] ${bag_path}"
done

echo "Completed ${#STEPS[@]} group(s)."
