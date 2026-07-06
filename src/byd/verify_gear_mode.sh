#!/bin/bash
# 验证 LOCAL→AUTO 档位链路（需在 Autoware 运行时执行）

set -eo pipefail

source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source /home/nvidia/autoware/install/setup.bash

check_topic() {
  local topic=$1
  if ros2 topic list 2>/dev/null | grep -qF "${topic}"; then
    echo "OK  topic exists: ${topic}"
    return 0
  fi
  echo "FAIL topic missing: ${topic}"
  return 1
}

echo "=== 档位链路验证 ==="
fail=0
check_topic "/vehicle/status/gear_status" || fail=1
check_topic "/control/shift_decider/gear_cmd" || fail=1
check_topic "/control/command/gear_cmd" || fail=1

if [ "${fail}" -eq 0 ]; then
  echo ""
  echo "gear_status sample:"
  timeout 3 ros2 topic echo /vehicle/status/gear_status --once 2>/dev/null || echo "  (no message within 3s - publish gear_cmd first)"
  echo ""
  echo "shift_decider gear_cmd hz (3s):"
  timeout 3 ros2 topic hz /control/shift_decider/gear_cmd 2>/dev/null || echo "  (not publishing - check autoware state + trajectory_follower + gear_status)"
fi

exit "${fail}"
