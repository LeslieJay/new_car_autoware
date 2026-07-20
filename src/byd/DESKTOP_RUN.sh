# 获取当前终端类型
CURRENT_TERMINAL=$(ps -p $PPID -o comm=)
gnome-terminal --tab -- bash -c "ros2 run can_six_driver can_rtk_node; exec bash"
gnome-terminal --tab -- bash -c "ros2 run can_driver can_node; exec bash"
gnome-terminal --tab -- bash -c "ros2 launch rslidar_sdk start_3.py; exec bash"
gnome-terminal --tab -- bash -c "ros2 run agv_to_rcs autoware_pose_to_rcs_pose; exec bash"
gnome-terminal --tab -- bash -c "cd /home/nvidia/autoware && source install/setup.bash && ros2 run agv_to_rcs autoware_auto_server; exec bash"
gnome-terminal --tab -- bash -c "cd /home/nvidia/autoware && source install/setup.bash && ros2 launch autoware_launch autoware.launch.xml; exec bash"
gnome-terminal --tab -- bash -c "cd /home/nvidia/autoware && source install/setup.bash && ros2 run agv_to_rcs main; exec bash"
gnome-terminal --tab -- bash -c "ros2 run byd_auto_engage auto_engage_node; exec bash"
#gnome-terminal --tab -- bash -c "ros2 launch reverse_parking_planner reverse_parking_planner.launch.py; exec bash"
gnome-terminal --tab -- bash -c "cd /home/nvidia/autoware && source install/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:=0.0.0.0 port:=9090"
