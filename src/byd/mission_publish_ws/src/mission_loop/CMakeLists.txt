cmake_minimum_required(VERSION 3.8)
project(mission_loop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

# -------- node --------
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/mission_loop_node.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  SYSTEM
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  rclcpp_components
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::mission_loop::MissionLoopNode"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR SingleThreadedExecutor
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# -------- install runtime files --------
ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
