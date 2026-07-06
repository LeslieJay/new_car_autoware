cmake_minimum_required(VERSION 3.5)
project(can_rtk_driver)

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

# ✅ 添加 include 目录到编译路径
include_directories(include)

# 添加可执行文件
add_executable(can_rtk_node 
src/can_node.cpp 
src/can_receiver.cpp 
src/can_send.cpp 
src/main.cpp 
src/can_battery.cpp
${agv_interfaces_SRCS})

# 链接依赖和库
ament_target_dependencies(can_rtk_node 
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
  tf2
  yaml-cpp
  rosidl_typesupport_cpp)
target_link_libraries(can_rtk_node 
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
  TARGETS can_rtk_node
  DESTINATION lib/${PROJECT_NAME}
)

# ✅ 安装头文件
install(DIRECTORY include/
  DESTINATION include)

ament_package()
