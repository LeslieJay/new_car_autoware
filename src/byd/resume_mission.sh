#!/bin/bash

set -eo pipefail

source /opt/ros/humble/setup.bash
if [ -f /home/nvidia/autoware/install/setup.bash ]; then
  # shellcheck disable=SC1091
  source /home/nvidia/autoware/install/setup.bash
fi

SERVICE_NAME="/byd/mission/resume"
SERVICE_TYPE="std_srvs/srv/Trigger"
WAIT_TIMEOUT_SECONDS=5
CALL_TIMEOUT_SECONDS=8

echo "=== 恢复任务（继续原路线）==="
if ! timeout "${WAIT_TIMEOUT_SECONDS}s" bash -c \
  "until services=\$(ros2 service list) && printf '%s\n' \"\$services\" | rg -qx '$SERVICE_NAME'; do sleep 0.2; done"
then
  echo "✗ 恢复失败：服务 $SERVICE_NAME 未就绪（等待 ${WAIT_TIMEOUT_SECONDS}s 超时）"
  echo "  请确认 mission_pause_node 已启动，并且话题/命名空间配置正确"
  exit 1
fi

if timeout "${CALL_TIMEOUT_SECONDS}s" ros2 service call "$SERVICE_NAME" "$SERVICE_TYPE" "{}"; then
  echo "✓ 任务已恢复"
else
  echo "✗ 恢复失败：调用 $SERVICE_NAME 超时或失败"
  exit 1
fi
