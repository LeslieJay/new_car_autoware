cmake_minimum_required(VERSION 3.14)
project(didrive_front_collision_warning)

find_package(autoware_cmake REQUIRED)
if(BUILD_TESTING)
  # Cppcheck 2.7 cannot parse consecutive GoogleTest TEST macros.
  set(ament_cmake_cppcheck_ADDITIONAL_EXCLUDE test)
endif()
autoware_package()

find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/risk_evaluator.cpp
)

ament_auto_add_executable(front_collision_warning_node
  src/front_collision_warning_node.cpp
)
target_link_libraries(front_collision_warning_node
  ${PROJECT_NAME}
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_risk_evaluator test/test_risk_evaluator.cpp)
  target_include_directories(test_risk_evaluator PRIVATE include)
  target_link_libraries(test_risk_evaluator ${PROJECT_NAME})
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
