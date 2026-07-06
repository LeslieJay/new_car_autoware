#!/bin/bash

# ROS 2 Autoware 服务调用脚本
# 从 LOCAL 模式切回 AUTONOMOUS (AUTO)，并复位档位为 DRIVE

set -eo pipefail

echo "=== 开始切回 AUTONOMOUS 模式 ==="

source /opt/ros/humble/setup.bash
if [ -f /home/nvidia/autoware/install/setup.bash ]; then
  # shellcheck disable=SC1091
  source /home/nvidia/autoware/install/setup.bash
fi

# 1. 切回 Autonomous（gate_mode 将变为 AUTO）
echo "[1/5] 切换到 Autonomous 操作模式..."
if ros2 service call /api/operation_mode/change_to_autonomous \
  autoware_adapi_v1_msgs/srv/ChangeOperationMode "{}"; then
  echo "✓ change_to_autonomous 成功"
else
  echo "✗ change_to_autonomous 失败"
  exit 1
fi

sleep 1

# 2. 启用 Autoware 控制
echo "[2/5] 启用 Autoware 控制..."
if ros2 service call /api/operation_mode/enable_autoware_control \
  autoware_adapi_v1_msgs/srv/ChangeOperationMode "{}"; then
  echo "✓ enable_autoware_control 成功"
else
  echo "✗ enable_autoware_control 失败"
  exit 1
fi

sleep 1

# 3. Engage
echo "[3/5] 设置 Engage=true..."
if ros2 service call /api/autoware/set/engage \
  tier4_external_api_msgs/srv/Engage "{engage: true}"; then
  echo "✓ Engage 成功"
else
  echo "✗ Engage 失败"
  exit 1
fi

sleep 1

# 4. 取消 vehicle_cmd_gate 暂停
echo "[4/5] 取消 vehicle_cmd_gate 暂停..."
if ros2 service call /control/vehicle_cmd_gate/set_pause \
  tier4_control_msgs/srv/SetPause "{pause: false}"; then
  echo "✓ 取消暂停成功"
else
  echo "✗ 取消暂停失败"
  exit 1
fi

sleep 1

# 5. 显式复位档位为 DRIVE（辅助 shift_decider / gate 脱离 LOCAL 时的 R 锁存）
echo "[5/5] 复位档位为 DRIVE..."
ros2 topic pub --once /control/shift_decider/gear_cmd \
  autoware_vehicle_msgs/msg/GearCommand "{stamp: {sec: 0, nanosec: 0}, command: 2}" >/dev/null

sleep 0.5

gate_mode=$(ros2 topic echo /control/current_gate_mode --once 2>/dev/null | awk '/data:/ {print $2; exit}')
gear_cmd=$(ros2 topic echo /control/command/gear_cmd --once 2>/dev/null | awk '/command:/ {print $2; exit}')
shift_gear=$(ros2 topic echo /control/shift_decider/gear_cmd --once 2>/dev/null | awk '/command:/ {print $2; exit}' || true)

echo "=== 切回完成 ==="
echo "  gate_mode (0=AUTO): ${gate_mode:-unknown}"
echo "  /control/command/gear_cmd command (2=DRIVE, 20=REVERSE): ${gear_cmd:-unknown}"
echo "  /control/shift_decider/gear_cmd command: ${shift_gear:-not publishing yet}"

if [ "${gear_cmd}" = "20" ]; then
  echo "WARNING: gear_cmd 仍为 REVERSE(20)，请确认 /vehicle/status/gear_status 已发布且 shift_decider 运行正常" >&2
  exit 1
fi
