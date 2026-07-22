开机自启动测试

# 告诉系统我刚修改了某个配置文件，你刷新一下（这个过程不需要重启系统）
sudo systemctl daemon-reload 

# 开启服务 
sudo systemctl start cameras_startup.service

# 查看服务状态
sudo systemctl status cameras_startup.service

# 关闭服务
sudo systemctl stop cameras_startup.service
video_recorder.service

# 重启摄像头（先关后开）
sudo systemctl restart ros2_cameras.service

# 设置开机自动启动：
sudo systemctl enable cameras_startup.service
sudo systemctl disable video_recorder.service

sudo vim /etc/systemd/system/cameras_startup.service

'''
[Unit]
Description=ROS2 4-Channel Camera Startup Service
# 确保在网络和图形界面（如果需要）准备好后再启动
After=network.target nvargus-daemon.service

[Service]
Type=simple
# 以 root 用户运行，这样 sysctl 才有权限修改，且拥有访问硬件的权限
User=root
# 你的脚本绝对路径
ExecStart=/home/nvidia/autoware/start_cameras.sh
# 异常退出时自动重启
Restart=on-failure
RestartSec=5
# 传递必要的环境变量（有些 Jetson 驱动需要）
Environment=USER=nvidia HOME=/home/nvidia

[Install]
WantedBy=multi-user.target
'''

sudo vim /etc/systemd/system/video_recorder.service

'''
[Unit]
Description=ROS2 4-Channel Video Recorder Service
# 强依赖：必须和摄像头服务一起启动，且在摄像头服务启动之后再执行
Requires=cameras_startup.service
After=network.target cameras_startup.service

[Service]
Type=simple
User=root
WorkingDirectory=/home/nvidia/autoware/src/driving_recorder

# 环境变量（根据你的 ROS2 版本自行调整 humble/foxy）
Environment="PYTHONUNBUFFERED=1"
Environment="PATH=/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
Environment="LD_LIBRARY_PATH=/home/nvidia/autoware/install/autoware_vehicle_msgs/lib:/home/nvidia/autoware/install/lib:/opt/ros/humble/lib:/usr/local/cuda/lib64"
Environment="PYTHONPATH=/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages"
Environment="PYTHONPATH=/home/nvidia/autoware/install/autoware_vehicle_msgs/local/lib/python3.10/dist-packages:/home/nvidia/autoware/install/local/lib/python3.10/dist-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages"

ExecStart=/usr/bin/python3 /home/nvidia/autoware/src/driving_recorder/recorder_node.py

# 优雅退出与重启
KillSignal=SIGINT
TimeoutStopSec=5
Restart=on-failure
RestartSec=5

[Install]
# 注意：这里可以不写 WantedBy，或者保持 multi-user.target
# 因为它已经被 cameras_startup 依赖链带起来了，为了保险也可以留着
WantedBy=multi-user.target
'''

