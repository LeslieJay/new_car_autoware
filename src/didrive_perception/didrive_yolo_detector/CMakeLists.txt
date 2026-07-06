cmake_minimum_required(VERSION 3.8)
project(didrive_yolo_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(autoware_perception_msgs REQUIRED)
find_package(image_transport REQUIRED)
# 显式寻找 CUDA 和 TensorRT 库
find_package(CUDA REQUIRED)
find_library(NVINFER_LIB nvinfer PATHS /usr/lib/aarch64-linux-gnu /usr/local/cuda/lib64 REQUIRED)
find_package(tier4_perception_msgs REQUIRED)
# 3. 添加并编译可执行节点
add_executable(yolo_detector_node src/detector_node.cpp)


# 配置头文件包含路径
target_include_directories(yolo_detector_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${OpenCV_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  /usr/include/aarch64-linux-gnu # TensorRT 默认头文件路径
)

# 链接第三方库
target_link_libraries(yolo_detector_node
  ${OpenCV_LIBS}
  ${NVINFER_LIB}
  ${CUDA_LIBRARIES}
)

# 链接 ROS 2 依赖
ament_target_dependencies(yolo_detector_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  autoware_perception_msgs
  tier4_perception_msgs
)

# 4. 安装规则 (安装至 install/ 目录归档，方便 Launch 寻找)
install(TARGETS yolo_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
