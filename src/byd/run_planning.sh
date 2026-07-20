#!/bin/bash

# 在当前终端窗口打开新的标签页来运行命令

# 获取当前终端类型
CURRENT_TERMINAL=$(ps -p $PPID -o comm=)
gnome-terminal --tab -- bash -c "cd /home/nvidia/autoware && source install/setup.bash && ros2 run agv_to_rcs autoware_pose_to_rcs_pose; exec bash"
gnome-terminal --tab -- bash -c "cd /home/nvidia/autoware && source install/setup.bash && ros2 run agv_to_rcs autoware_auto_server; exec bash"
gnome-terminal --tab -- bash -c "cd /home/nvidia/autoware && source install/setup.bash && ros2 launch autoware_launch planning_simulator.launch.xml; exec bash"
gnome-terminal --tab -- bash -c "cd /home/nvidia/autoware && source install/setup.bash && ros2 run agv_to_rcs main; exec bash"
gnome-terminal --tab -- bash -c "cd /home/nvidia/autoware && source install/setup.bash && ros2 run can_driver can_node; exec bash"
gnome-terminal --tab -- bash -c "cd /home/nvidia/autoware && source install/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:=0.0.0.0 port:=9090"
# ros2 service call /api/operation_mode/change_to_autonomous autoware_adapi_v1_msgs/srv/ChangeOperationMod
# ros2 service call /api/operation_mode/change_to_autonomous
