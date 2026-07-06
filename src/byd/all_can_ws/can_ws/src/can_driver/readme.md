<!--
 * @Author: du.xiaoying1
 * @Date: 2025-08-12 11:43:01
 * @LastEditors: dxy
 * @LastEditTime: 2025-08-19 16:49:09
 * @FilePath: /qr_agv_0627_r/src/can_driver/readme.md
 * @Description: 
 * 
 * Copyright (c) 2025 by du.xiaoying1 , All Rights Reserved. 
-->


##准备
1.安装库和功能包
sudo apt update
sudo apt install build-essential
sudo apt install can-utils  # 安装 can-utils 工具包
sudo apt install libsocketcan-dev  # 如果需要安装 SocketCAN 开发库

2.赋予代码中指令免密权限

注：每台设备只需执行一次即可，shell脚本更新除外
# 1. 添加执行权限
chmod +x add-can-sudo.sh

# 2. 运行（无需手动加 sudo）
./add-can-sudo.sh

##编译顺序
1.
colcon build --packages-select agv_interfaces

2.
colcon build --packages-select hins_laser_interfaces

3.
source install/setup.bash

4.
colcon build
or
colcon build --packages-select can_driver

若是先进行步骤4,可能报错找不到agv_interfaces ,hins_laser_interfaces

5.运行
ros2 launch can_driver can_launch.py



