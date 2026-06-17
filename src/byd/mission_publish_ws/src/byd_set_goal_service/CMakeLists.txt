cmake_minimum_required(VERSION 3.8)
project(byd_set_goal_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(autoware_adapi_v1_msgs REQUIRED)

include_directories(include)

add_executable(set_goal_node src/set_goal_node.cpp)
ament_target_dependencies(set_goal_node
  rclcpp
  geometry_msgs
  std_msgs
  autoware_adapi_v1_msgs
)

install(TARGETS set_goal_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
