#!/bin/bash

# ROS 2 Autoware 服务调用脚本
# 用于依次执行四个服务调用

echo "=== 开始执行ROS 2服务调用 ==="

# 1. 切换到本地操作模式
echo "[1/4] 切换到本地操作模式..."
if ros2 service call /api/operation_mode/change_to_local autoware_adapi_v1_msgs/srv/ChangeOperationMode; then
    echo "✓ 切换到本地操作模式成功"
else
    echo "✗ 切换到本地操作模式失败"
    exit 1
fi

# 等待服务响应
sleep 1

# 2. 启用Autoware控制
echo "[2/4] 启用Autoware控制..."
if ros2 service call /api/operation_mode/enable_autoware_control autoware_adapi_v1_msgs/srv/ChangeOperationMode "{}"; then
    echo "✓ 启用Autoware控制成功"
else
    echo "✗ 启用Autoware控制失败"
    exit 1
fi

# 等待服务响应
sleep 1

# 3. 设置Engage为true
echo "[3/4] 设置Engage为true..."
if ros2 service call /api/autoware/set/engage tier4_external_api_msgs/srv/Engage "{engage: true}"; then
    echo "✓ 设置Engage成功"
else
    echo "✗ 设置Engage失败"
    exit 1
fi

# 等待服务响应
sleep 1

# 4. 取消车辆命令暂停
echo "[4/4] 取消车辆命令暂停..."
if ros2 service call /control/vehicle_cmd_gate/set_pause tier4_control_msgs/srv/SetPause "{pause: false}"; then
    echo "✓ 取消车辆命令暂停成功"
else
    echo "✗ 取消车辆命令暂停失败"
    exit 1
fi

echo "=== 所有服务调用执行完成！ ==="