#!/usr/bin/env bash
# 最小冒烟检查：确认统一倒车节点对外接口存在
set -euo pipefail

missing=0

check_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "[FAIL] command not found: $1"
    missing=1
  fi
}

check_cmd ros2

if ! ros2 service list 2>/dev/null | grep -q '/reverse_parking_planner/set_goal_pose'; then
  echo "[FAIL] service /reverse_parking_planner/set_goal_pose not found (is node running?)"
  missing=1
else
  echo "[OK] set_goal_pose service"
fi

if ! ros2 service list 2>/dev/null | grep -q '/reverse_parking_planner/trigger_planning'; then
  echo "[FAIL] service /reverse_parking_planner/trigger_planning not found"
  missing=1
else
  echo "[OK] trigger_planning service"
fi

for topic in \
  /external/selected/control_cmd \
  /external/selected/gear_cmd \
  /planning/scenario_planning/parking/trajectory; do
  if ros2 topic list 2>/dev/null | grep -qx "$topic"; then
    echo "[OK] topic $topic"
  else
    echo "[WARN] topic $topic not found (may appear after first plan)"
  fi
done

if [[ "$missing" -ne 0 ]]; then
  exit 1
fi

echo "Smoke check passed."
