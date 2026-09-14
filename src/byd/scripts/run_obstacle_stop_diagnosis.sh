#!/usr/bin/env bash
set -Eeuo pipefail

# Host-side diagnosis matrix. This orchestrates the existing independent-case
# runner and deliberately inherits all ROS/DDS variables from the host shell.

WORKSPACE="/home/nvidia/autoware"
DISTANCES="8 10 12 14 16 18 20"
TRANSITION_REPEATS=2
OUTPUT_ROOT="${WORKSPACE}/log/$(date +%Y%m%d)/obstacle_stop_diagnosis_$(date +%H%M%S)"
MAX_OBSERVE_SEC=180
BLOCKED_SEC=15
SKIP_CONTROLS=0

usage() {
  cat <<'EOF'
Usage: run_obstacle_stop_diagnosis.sh [options]
  --distances "8 10 12 14 16 18 20"   Distances for the original matrix
  --transition-repeats N                Additional runs for 10/12/14/16/20 m
  --output-root PATH                    Root for all diagnosis artifacts
  --max-observe-sec N                   Per-case observation limit (default 180)
  --blocked-sec N                       No-progress threshold (default 15)
  --skip-controls                       Skip no-obstacle/module-isolation controls
EOF
}

while (($#)); do
  case "$1" in
    --distances) DISTANCES="${2:?missing value}"; shift 2 ;;
    --transition-repeats) TRANSITION_REPEATS="${2:?missing value}"; shift 2 ;;
    --output-root) OUTPUT_ROOT="${2:?missing value}"; shift 2 ;;
    --max-observe-sec) MAX_OBSERVE_SEC="${2:?missing value}"; shift 2 ;;
    --blocked-sec) BLOCKED_SEC="${2:?missing value}"; shift 2 ;;
    --skip-controls) SKIP_CONTROLS=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

RUNNER="${WORKSPACE}/src/byd/scripts/run_obstacle_stop_longitudinal_sweep.sh"
ANALYZER="${WORKSPACE}/src/byd/scripts/diagnose_obstacle_stop_timeline.py"
SUMMARY="${WORKSPACE}/src/byd/scripts/summarize_obstacle_stop_diagnosis.py"
mkdir -p "${OUTPUT_ROOT}"

echo "host ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>}"
echo "host RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<unset>}"
echo "host ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-<unset>}"
echo "diagnosis_root=${OUTPUT_ROOT}"

bash -n "${RUNNER}"
python3 -m py_compile "${ANALYZER}" "${SUMMARY}"

run_matrix() {
  local label="$1" distances="$2" root="$3"
  shift 3
  mkdir -p "${root}"
  echo "=== matrix ${label}: ${distances} ==="
  env \
    OBSTACLE_STOP_LONGITUDINAL_DISTANCES="${distances}" \
    LOG_ROOT="${root}" \
    MAX_OBSERVE_SEC="${MAX_OBSERVE_SEC}" \
    BLOCKED_SEC="${BLOCKED_SEC}" \
    CONTROL_LABEL="${label}" \
    "$@" "${RUNNER}" || true
}

# Analyze the previous seven-case run before starting new host processes.
existing_root="$(find "${WORKSPACE}/log" -type d -name 'obstacle_stop_longitudinal_sweep_*' -print 2>/dev/null | sort | tail -1 || true)"
if [[ -n "${existing_root}" && "${existing_root}" != "${OUTPUT_ROOT}" ]]; then
  baseline_root="${OUTPUT_ROOT}/baseline_existing"
  for distance in ${DISTANCES}; do
    mkdir -p "${baseline_root}/${distance}m"
    python3 -u "${ANALYZER}" \
      --case-dir "${existing_root}/${distance}m" \
      --out "${baseline_root}/${distance}m/timeline.json" \
      >"${baseline_root}/${distance}m/timeline.log" 2>&1 || true
  done
else
  echo "no prior obstacle_stop_longitudinal_sweep_* root found; skipping baseline"
fi

# Original requested protocol, one complete 8..20 m sweep.
run_matrix original_protocol "${DISTANCES}" "${OUTPUT_ROOT}/original_protocol/repetition_0"

# Repeat the transition neighborhoods to test determinism.
transition_distances="10 12 14 16 20"
for repeat in $(seq 1 "${TRANSITION_REPEATS}"); do
  run_matrix "original_protocol_repeat_${repeat}" "${transition_distances}" \
    "${OUTPUT_ROOT}/original_protocol/repetition_${repeat}"
done

if [[ "${SKIP_CONTROLS}" == 0 ]]; then
  # No-obstacle baseline: isolates Start Planner/MRM/trajectory health.
  run_matrix baseline_no_obstacle "0" "${OUTPUT_ROOT}/baseline_no_obstacle" \
    env PLACE_OBSTACLE=false MAX_OBSERVE_SEC=30

  # Stop-module isolation: only 16/20 m are needed.
  for distance in 16 20; do
    run_matrix obstacle_stop_off "${distance}" \
      "${OUTPUT_ROOT}/obstacle_stop_off" \
      env LAUNCH_OBSTACLE_STOP_MODULE=false LAUNCH_DYNAMIC_OBSTACLE_STOP_MODULE=false
  done

  # Start Planner isolation: runtime parameter change is limited to this control.
  for distance in 12 16 20; do
    run_matrix start_planner_freespace_off "${distance}" \
      "${OUTPUT_ROOT}/start_planner_freespace_off" \
      env DISABLE_START_PLANNER_FREESPACE=1
  done
fi

python3 -u "${SUMMARY}" "${OUTPUT_ROOT}"
echo "diagnosis_summary=${OUTPUT_ROOT}/diagnosis_summary.md"
