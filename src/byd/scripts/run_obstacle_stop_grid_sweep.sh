#!/usr/bin/env bash
set -Eeuo pipefail

# Execute the fixed-position Obstacle Stop matrix.  The single-case runner owns
# all ROS lifecycle, bag recording, object cleanup and analysis; this wrapper
# only expands the distance x intrusion x repetition matrix.

WORKSPACE="/home/nvidia/autoware"
DISTANCE_LIST="${OBSTACLE_STOP_LONGITUDINAL_DISTANCES:-8 12 16 20}"
INTRUSION_LIST="${OBSTACLE_STOP_INTRUSIONS:-0.5 1.0 1.5}"
SHOULDER="${OBSTACLE_STOP_SHOULDER:-right}"
REPETITIONS="${OBSTACLE_STOP_REPETITIONS:-3}"
LOG_ROOT="${LOG_ROOT:-${WORKSPACE}/log/$(date +%Y%m%d)/obstacle_stop_grid_sweep_$(date +%H%M%S)}"
MAX_OBSERVE_SEC="${MAX_OBSERVE_SEC:-180}"
BLOCKED_SEC="${BLOCKED_SEC:-15}"
RESUME_COMPLETED="${RESUME_COMPLETED:-true}"
DRY_RUN="${DRY_RUN:-false}"

read -r -a DISTANCES <<<"${DISTANCE_LIST}"
read -r -a INTRUSIONS <<<"${INTRUSION_LIST}"

if [[ ! "${REPETITIONS}" =~ ^[1-9][0-9]*$ ]]; then
  echo "OBSTACLE_STOP_REPETITIONS must be a positive integer" >&2
  exit 2
fi
if [[ "${SHOULDER}" != right && "${SHOULDER}" != left ]]; then
  echo "OBSTACLE_STOP_SHOULDER must be right or left" >&2
  exit 2
fi
if (( ${#DISTANCES[@]} == 0 || ${#INTRUSIONS[@]} == 0 )); then
  echo "distance and intrusion lists must not be empty" >&2
  exit 2
fi
for value in "${DISTANCES[@]}"; do
  if ! [[ "${value}" =~ ^[0-9]+([.][0-9]+)?$ ]] || [[ "${value}" == 0 || "${value}" == 0.* ]]; then
    echo "invalid longitudinal distance: ${value}" >&2
    exit 2
  fi
done
for value in "${INTRUSIONS[@]}"; do
  if ! [[ "${value}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
    echo "invalid intrusion: ${value}" >&2
    exit 2
  fi
done

RUNNER="${WORKSPACE}/src/byd/scripts/run_obstacle_stop_longitudinal_sweep.sh"
SUMMARY="${WORKSPACE}/src/byd/scripts/summarize_obstacle_stop_longitudinal_sweep.py"
mkdir -p "${LOG_ROOT}"

bash -n "${RUNNER}" "${WORKSPACE}/src/byd/scripts/autonomous_mode_run.sh"
python3 -m py_compile \
  "${WORKSPACE}/src/byd/scripts/analyze_longitudinal_obstacle_case.py" \
  "${WORKSPACE}/src/byd/scripts/monitor_longitudinal_obstacle_case.py" \
  "${SUMMARY}"

case_dir_for() {
  local distance="$1" intrusion="$2" repetition="$3"
  local distance_tag intrusion_tag
  distance_tag="$(printf '%g' "${distance}")"
  intrusion_tag="$(printf '%g' "${intrusion}")"
  printf '%s/d%sm_i%sm_%s_r%02d' \
    "${LOG_ROOT}" "${distance_tag}" "${intrusion_tag}" "${SHOULDER}" "${repetition}"
}

has_complete_result() {
  local result_path="$1"
  [[ -s "${result_path}" ]] || return 1
  python3 - "${result_path}" <<'PY'
import json
import sys
from pathlib import Path

try:
    result = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
except (OSError, json.JSONDecodeError):
    raise SystemExit(1)
raise SystemExit(0 if result.get("result") not in (None, "SETUP_INVALID") else 1)
PY
}

run_case() {
  local distance="$1" intrusion="$2" repetition="$3" case_dir="$4"
  if [[ "${DRY_RUN}" == true ]]; then
    echo "dry-run: distance=${distance} intrusion=${intrusion} shoulder=${SHOULDER} repetition=${repetition} case_dir=${case_dir}"
    return 0
  fi
  if [[ "${RESUME_COMPLETED}" == true ]] && has_complete_result "${case_dir}/result.json"; then
    echo "resume: ${case_dir}"
    return 0
  fi
  mkdir -p "${case_dir}"
  echo "=== distance=${distance}m intrusion=${intrusion}m shoulder=${SHOULDER} repetition=${repetition} ==="
  distance_tag="$(printf '%g' "${distance}")"
  intrusion_tag="$(printf '%g' "${intrusion}")"
  env \
    OBSTACLE_STOP_GRID_CHILD=1 \
    OBSTACLE_STOP_PROTOCOL=fixed \
    OBSTACLE_STOP_INTRUSION="${intrusion}" \
    OBSTACLE_STOP_SHOULDER="${SHOULDER}" \
    OBSTACLE_STOP_REPETITION="${repetition}" \
    OBSTACLE_STOP_CASE_ID="d${distance_tag}m_i${intrusion_tag}m_${SHOULDER}_r$(printf '%02d' "${repetition}")" \
    OBSTACLE_STOP_LONGITUDINAL_DISTANCES="${distance}" \
    LOG_ROOT="${case_dir}" \
    MAX_OBSERVE_SEC="${MAX_OBSERVE_SEC}" \
    BLOCKED_SEC="${BLOCKED_SEC}" \
    CONTROL_LABEL="fixed_position_grid" \
    "${RUNNER}" || true
}

for repetition in $(seq 1 "${REPETITIONS}"); do
  for intrusion in "${INTRUSIONS[@]}"; do
    for distance in "${DISTANCES[@]}"; do
      run_case "${distance}" "${intrusion}" "${repetition}" \
        "$(case_dir_for "${distance}" "${intrusion}" "${repetition}")"
    done
  done
done

# Three no-obstacle baselines use the same independent launch lifecycle.  They
# are separate from the obstacle matrix and are reported as controls.
for repetition in $(seq 1 "${REPETITIONS}"); do
  baseline_dir="${LOG_ROOT}/baseline_r$(printf '%02d' "${repetition}")"
  if [[ "${DRY_RUN}" == true ]]; then
    echo "dry-run: baseline repetition=${repetition} case_dir=${baseline_dir}"
    continue
  fi
  if [[ "${RESUME_COMPLETED}" == true ]] && has_complete_result "${baseline_dir}/result.json"; then
    continue
  fi
  mkdir -p "${baseline_dir}"
  env \
    OBSTACLE_STOP_GRID_CHILD=1 \
    OBSTACLE_STOP_PROTOCOL=fixed \
    PLACE_OBSTACLE=false \
    OBSTACLE_STOP_REPETITION="${repetition}" \
    OBSTACLE_STOP_CASE_ID="baseline_r$(printf '%02d' "${repetition}")" \
    OBSTACLE_STOP_LONGITUDINAL_DISTANCES="0" \
    LOG_ROOT="${baseline_dir}" \
    MAX_OBSERVE_SEC=30 \
    CONTROL_LABEL="no_obstacle_baseline" \
  "${RUNNER}" || true
done

if [[ "${DRY_RUN}" == true ]]; then
  echo "dry-run: matrix expansion complete"
  exit 0
fi

python3 -u "${SUMMARY}" "${LOG_ROOT}" \
  --distances "${DISTANCES[@]}" \
  --intrusions "${INTRUSIONS[@]}" \
  --shoulder "${SHOULDER}" \
  --repetitions "${REPETITIONS}"
echo "grid_root=${LOG_ROOT}"
