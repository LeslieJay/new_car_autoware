cmake_minimum_required(VERSION 3.14)
project(agv_simple_reverse_planner)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(${PROJECT_NAME}_node_exe
  src/agv_simple_reverse_planner_node.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
