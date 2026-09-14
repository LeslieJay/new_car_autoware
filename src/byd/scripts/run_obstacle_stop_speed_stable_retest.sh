#!/usr/bin/env bash
set -Eeuo pipefail

# Host-side causal retest.  The underlying runner owns one fresh Autoware
# process per distance; this wrapper adds repetitions and the two one-variable
# controls from the diagnosis plan.
WORKSPACE="/home/nvidia/autoware"
DISTANCES="10,12"
REPEATS=5
TARGET_SPEED=2.0
SPEED_TOLERANCE=0.1
STABLE_SEC=3.0
OUTPUT_ROOT="${WORKSPACE}/log/$(date +%Y%m%d)/obstacle_stop_speed_stable_10_12_$(date +%H%M%S)"
MAX_OBSERVE_SEC=180
BLOCKED_SEC=15
RUN_CONTROLS=1

usage() {
  cat <<'EOF'
Usage: run_obstacle_stop_speed_stable_retest.sh [options]
  --distances 10,12
  --repeats N
  --target-speed MPS
  --speed-tolerance MPS
  --stable-sec SEC
  --output-root PATH
  --max-observe-sec SEC
  --blocked-sec SEC
  --skip-controls
EOF
}

while (($#)); do
  case "$1" in
    --distances) DISTANCES="${2:?missing value}"; shift 2 ;;
    --repeats) REPEATS="${2:?missing value}"; shift 2 ;;
    --target-speed) TARGET_SPEED="${2:?missing value}"; shift 2 ;;
    --speed-tolerance) SPEED_TOLERANCE="${2:?missing value}"; shift 2 ;;
    --stable-sec) STABLE_SEC="${2:?missing value}"; shift 2 ;;
    --output-root) OUTPUT_ROOT="${2:?missing value}"; shift 2 ;;
    --max-observe-sec) MAX_OBSERVE_SEC="${2:?missing value}"; shift 2 ;;
    --blocked-sec) BLOCKED_SEC="${2:?missing value}"; shift 2 ;;
    --skip-controls) RUN_CONTROLS=0; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if ! [[ "${REPEATS}" =~ ^[1-9][0-9]*$ ]]; then
  echo "--repeats must be a positive integer" >&2
  exit 2
fi
if ! [[ "${DISTANCES}" =~ ^[0-9]+([.][0-9]+)?,[0-9]+([.][0-9]+)?$ ]]; then
  echo "--distances must contain exactly two comma-separated numbers" >&2
  exit 2
fi
DISTANCE_LIST="${DISTANCES//,/ }"
RUNNER="${WORKSPACE}/src/byd/scripts/run_obstacle_stop_longitudinal_sweep.sh"
SUMMARY="${WORKSPACE}/src/byd/scripts/summarize_obstacle_stop_longitudinal_sweep.py"

mkdir -p "${OUTPUT_ROOT}"
bash -n "${RUNNER}" "${WORKSPACE}/src/byd/scripts/run_obstacle_stop_speed_stable_retest.sh"
python3 -m py_compile \
  "${WORKSPACE}/src/byd/scripts/place_route_obstacle.py" \
  "${WORKSPACE}/src/byd/scripts/monitor_longitudinal_obstacle_case.py" \
  "${WORKSPACE}/src/byd/scripts/wait_vehicle_speed_stable.py" \
  "${WORKSPACE}/src/byd/scripts/wait_trajectory_stable.py" \
  "${WORKSPACE}/src/byd/scripts/diagnose_obstacle_stop_timeline.py"

run_matrix() {
  local label="$1" root="$2" lost="$3" commitment="$4" freespace="$5" distance_list="$6"
  env \
    OBSTACLE_STOP_LONGITUDINAL_DISTANCES="${distance_list}" \
    LOG_ROOT="${root}" \
    MAX_OBSERVE_SEC="${MAX_OBSERVE_SEC}" \
    BLOCKED_SEC="${BLOCKED_SEC}" \
    CONTROL_LABEL="${label}" \
    TARGET_LOST_TIME_THRESHOLD="${lost}" \
    COMMITMENT_DISTANCE_BEFORE_SHIFT_START="${commitment}" \
    DISABLE_START_PLANNER_FREESPACE="${freespace}" \
    SPEED_TARGET="${TARGET_SPEED}" \
    SPEED_TOLERANCE="${SPEED_TOLERANCE}" \
    SPEED_STABLE_SEC="${STABLE_SEC}" \
    "${RUNNER}" || true
}

for repeat in $(seq 1 "${REPEATS}"); do
  run_matrix "stable_speed_default" "${OUTPUT_ROOT}/repetition_${repeat}" "" "" 0 "${DISTANCE_LIST}"
done

if [[ "${RUN_CONTROLS}" == 1 ]]; then
  # Controls are deliberately separate from the formal default matrix.
  for control_repeat in 1 2 3; do
    run_matrix "stable_speed_target_hold_8s" "${OUTPUT_ROOT}/control_target_hold_8s/repetition_${control_repeat}" 8.0 "" 0 12
    run_matrix "stable_speed_commitment_5m" "${OUTPUT_ROOT}/control_commitment_5m/repetition_${control_repeat}" "" 5.0 0 12
  done
fi

python3 -u "${WORKSPACE}/src/byd/scripts/summarize_obstacle_stop_speed_stable_retest.py" \
  "${OUTPUT_ROOT}" --distances "${DISTANCES}"
echo "retest_root=${OUTPUT_ROOT}"
