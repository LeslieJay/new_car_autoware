#!/bin/bash

# 通过 bringup.launch.py 统一启动（can_driver 会在独立终端拉起）
cd /home/nvidia/autoware
source install/setup.bash
ros2 launch byd_launch bringup.launch.py

# 切自动模式:
# ros2 service call /api/operation_mode/change_to_autonomous autoware_adapi_v1_msgs/srv/ChangeOperationMode
