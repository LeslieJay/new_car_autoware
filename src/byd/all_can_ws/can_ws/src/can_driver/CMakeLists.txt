cmake_minimum_required(VERSION 3.5)
project(can_driver)

# 找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED) 
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(agv_interfaces REQUIRED)
find_package(autoware_control_msgs REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
find_package(tier4_external_api_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)                # 关键：查找 sensor_msgs
find_package(autoware_sensing_msgs REQUIRED)      # 查找 autoware_sensing_msgs
find_package(tf2 REQUIRED)
find_package(vda5050_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(ref_slam_interface REQUIRED)


# ✅ 添加 include 目录到编译路径
include_directories(include)

# 添加可执行文件
add_executable(can_node 
src/can_node.cpp 
src/can_receiver.cpp 
src/can_send.cpp 
src/usbcan_battery.cpp 
src/usbcan_parser.cpp 
src/hook_action_server.cpp
src/main.cpp 
src/usbcan_utils.cpp
${agv_interfaces_SRCS})

# 链接依赖和库
ament_target_dependencies(can_node 
 agv_interfaces
  rclcpp
  rclcpp_action  
  std_msgs
  geometry_msgs
  autoware_control_msgs
  autoware_vehicle_msgs
  tier4_external_api_msgs
  sensor_msgs       
  autoware_sensing_msgs
  vda5050_interfaces
  ref_slam_interface
  tf2
  tf2_msgs
  tf2_ros
  yaml-cpp)
target_link_libraries(can_node 
  pthread
  ${rclcpp_action_LIBRARIES} 
  ${agv_interfaces_TARGETS}
  yaml-cpp
  agv_interfaces::agv_interfaces__rosidl_generator_c
  agv_interfaces::agv_interfaces__rosidl_typesupport_c)

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
