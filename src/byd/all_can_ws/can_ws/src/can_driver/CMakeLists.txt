cmake_minimum_required(VERSION 3.5)
project(can_driver)

# 找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED) 
find_package(geometry_msgs REQUIRED)
find_package(agv_interfaces REQUIRED)
find_package(autoware_control_msgs REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
find_package(tier4_external_api_msgs REQUIRED)
find_package(vda5050_interfaces REQUIRED)
find_package(ref_slam_interface REQUIRED)
find_package(autoware_system_msgs REQUIRED)


# ✅ 添加 include 目录到编译路径
include_directories(include)

# 添加可执行文件
add_executable(can_node 
src/can_node.cpp 
src/can_receiver.cpp 
src/can_send.cpp 
src/hook_action_server.cpp
src/main.cpp)

# 链接依赖和库
ament_target_dependencies(can_node 
  agv_interfaces
  rclcpp
  autoware_system_msgs
  rclcpp_action
  geometry_msgs
  autoware_control_msgs
  autoware_vehicle_msgs
  tier4_external_api_msgs
  vda5050_interfaces
  ref_slam_interface)
target_link_libraries(can_node 
  pthread)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

# ✅ 安装 config 目录 (只保留这一条！)
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install the node executable
install(
  TARGETS can_node
  DESTINATION lib/${PROJECT_NAME}
)

# ✅ 安装头文件
install(DIRECTORY include/
  DESTINATION include)

ament_package()

set_target_properties(can_node PROPERTIES
    INSTALL_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/lib"
    BUILD_RPATH   "${CMAKE_CURRENT_SOURCE_DIR}/lib"
)
