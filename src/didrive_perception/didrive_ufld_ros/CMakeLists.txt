cmake_minimum_required(VERSION 3.16)
project(didrive_ufld_ros)
# 默认使用C++17标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 根据ROS2版本设置兼容性
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
message(WARNING "rclcpp found: ${rclcpp_FOUND}")
message(WARNING "tf2_ros found: ${tf2_ros_FOUND}")
# 首先查找CUDA
find_package(CUDAToolkit REQUIRED)

# 设置CUDA包含路径
message(STATUS "CUDA Toolkit version: ${CUDAToolkit_VERSION}")
message(STATUS "CUDA Include path: ${CUDAToolkit_INCLUDE_DIRS}")

# 查找TensorRT
find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  PATHS 
    /usr/include/aarch64-linux-gnu 
    /usr/include/x86_64-linux-gnu
    /usr/local/cuda/targets/aarch64-linux/include
  REQUIRED
)

find_library(TENSORRT_LIBRARY nvinfer
  PATHS 
    /usr/lib/aarch64-linux-gnu 
    /usr/lib/x86_64-linux-gnu
    /usr/local/cuda/targets/aarch64-linux/lib
  REQUIRED
)

# 查找TensorRT插件库（如果存在）
find_library(TENSORRT_PLUGIN_LIBRARY nvinfer_plugin
  PATHS 
    /usr/lib/aarch64-linux-gnu 
    /usr/lib/x86_64-linux-gnu
    /usr/local/cuda/targets/aarch64-linux/lib
)

# 查找Eigen3
find_package(Eigen3 REQUIRED)

# 检查TensorRT是否找到
if(NOT TENSORRT_INCLUDE_DIR)
  message(FATAL_ERROR "TensorRT include directory not found")
endif()

if(NOT TENSORRT_LIBRARY)
  message(FATAL_ERROR "TensorRT library not found")
endif()

message(STATUS "TensorRT include: ${TENSORRT_INCLUDE_DIR}")
message(STATUS "TensorRT library: ${TENSORRT_LIBRARY}")

# 包含目录
include_directories(
  include
  ${TENSORRT_INCLUDE_DIR}
  ${CUDAToolkit_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# 创建可执行文件
add_executable(ufld_ros_node
  src/ufld_detector.cpp
  src/ufld_ros_node.cpp
)

# 目标链接库
target_link_libraries(ufld_ros_node
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  cv_bridge::cv_bridge
  yaml-cpp
  CUDA::cudart 
  cuda
  ${OpenCV_LIBRARIES}
  ${yaml-cpp_LIBRARIES}
  tf2::tf2
  tf2_ros::tf2_ros
  ${TENSORRT_LIBRARY}
  ${CUDAToolkit_LIBRARIES}
)

# 添加包含目录
target_include_directories(ufld_ros_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${TENSORRT_INCLUDE_DIR}
  ${CUDAToolkit_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

# 添加编译定义
target_compile_definitions(ufld_ros_node PRIVATE
  EIGEN_DONT_ALIGN_STATICALLY=1
)

# 添加编译选项
target_compile_options(ufld_ros_node PRIVATE
  $<$<COMPILE_LANGUAGE:CXX>:-std=c++17>
)

# 安装目标
install(TARGETS
  ufld_ros_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装启动文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装配置文件
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# 安装模型文件
install(DIRECTORY model
  DESTINATION share/${PROJECT_NAME}
)

# 导出依赖
ament_export_dependencies(
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  yaml-cpp
  tf2
  tf2_ros
)
# 声明此包为ament包
ament_export_include_directories(include)
ament_package()