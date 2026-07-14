cmake_minimum_required(VERSION 3.8)
project(byd_mission_pause_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tier4_control_msgs REQUIRED)
find_package(autoware_adapi_v1_msgs REQUIRED)

include_directories(include)

add_executable(mission_pause_node src/mission_pause_node.cpp)
ament_target_dependencies(mission_pause_node
  rclcpp
  std_srvs
  tier4_control_msgs
  autoware_adapi_v1_msgs
)

install(TARGETS mission_pause_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
