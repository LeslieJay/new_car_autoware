### Usage

step1.使用相机驱动点亮相机
    cd /home/byd/SG8A_ORIN_GMSL2-F_V2_AGX_Orin_YUV_JP6.2_L4TR36.4.3
    ./quick_start.sh

step2.使用ros2 launch启动录制程序
    cd vehicle_video_recorder
    source install/setup.bash
    ros2 launch vehicle_video_recorder recorder_launch.xml