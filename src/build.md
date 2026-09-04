colcon build --symlink-install --packages-select \
    byd_launch \
    autoware_launch \
    autoware_vehicle_cmd_gate \
    autoware_surround_obstacle_checker \
    autoware_behavior_path_simple_avoidance_module \
    autoware_behavior_path_simple_lane_change_avoidance_module \
    didrive_front_collision_warning \
    mission_loop
## 测试

  --cmake-clean-cache

stat -L /home/nvidia/autoware/install/autoware_behavior_path_simple_avoidance_module/lib/libautoware_behavior_path_simple_avoidance_module.so

stat "$(readlink -f /home/nvidia/autoware/install/autoware_behavior_path_simple_avoidance_module/lib/libautoware_behavior_path_simple_avoidance_module.so)"


cd /home/nvidia/autoware
source /opt/ros/humble/setup.bash
colcon build \
  --packages-select \
  byd_event_rosbag_recorder \
  byd_system_event_monitor \
  byd_launch \
  autoware_launch \
  autoware_multi_object_tracker \
  --symlink-install \
  --event-handlers console_direct+