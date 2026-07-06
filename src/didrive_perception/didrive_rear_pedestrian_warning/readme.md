# 发布图像
ros2 run image_publisher image_publisher_node /home/nvidia/autoware/src/didrive_perception/didrive_rear_pedestrian_warning/images/bus.jpg --ros-args -r /image_raw:=/sensing/camera/camera0/image_rect_color

# 启动yolox检测行人
ros2 launch autoware_tensorrt_yolox yolox_s_plus_opt.launch.xml 

# 启动didrive_rear_pedestrian_warning
ros2 launch didrive_rear_pedestrian_warning rear_pedestrian_warning.launch.xml

# 输出话题
