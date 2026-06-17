#进入节点目录
cd lane_detection_node

#####################当前位于lane_detection_node目录下#####################

#onnx转为tr
/usr/src/tensorrt/bin/trtexec --onnx=tmp_own.onnx --fp16 --saveEngine==tmp_own_fp16.trt

#ROS环境配置
source /opt/ros/humble/setup.bash

# 编译
colcon build --packages-select ufld_ros --cmake-args -DCMAKE_BUILD_TYPE=Release

# 加载环境
source install/setup.bash

#publish test image
python test_publisher.py

# 运行节点
ros2 launch ufld_ros ufld.launch.py

#可以修改yaml文件调整参数，若相机安装位置调整，需要重新确认以下参数：
重要参数一：camera_intrinsics
重要参数二：camera_height
重要参数三：pitch_angle，可以通过python find_best_pitch_angle.py确认，需要提前准备好x-anylabeling/labelme/labelimg（提前精细标注好两条平行的车道线）生成的车道线标注json文件，作为json输入路径


#车道线类型id与名称映射
label_mapping = {
     'unknown':0,'white-dash': 1, 'white-solid': 2, 'double-white-dash': 3,
     'double-white-solid': 4, 'white-ldash-rsolid': 5, 'white-solid-rdash': 6,
     'yellow-dash': 7, 'yellow-solid': 8, 'double-yellow-dash': 9,
     'double-yellow-solid': 10, 'yellow-ldash-rsolid': 11, 'yellow-lsolid-rdash': 12,
     'left-curbside': 13, 'right-curbside': 14
}

#若需要计算车道线误差指标并进行2D/3D可视化
1、准备好gt和predict文件夹，里面的txt格式例如：
1713.0 1350 1574.5 1417 1418.0 1492 1262.0 1567 1105.0 1642 962.5 1710 805.0 1785 646.5 1860 488.0 1935 346.0 2002 186.0 2077 47.5 2152
1883.5 1350 1847.0 1417 1807.5 1492 1769.0 1567 1730.5 1642 1695.5 1710 1657.5 1785 1618.5 1860 1578.5 1935 1542.0 2002
参考格式：
x1 y1 x2 y2 x3 y3...
x1 y1 x2 y2 x3 y3 x4 y4...
2、准备好图片文件夹
3、运行脚本cal_lane_metric.py
