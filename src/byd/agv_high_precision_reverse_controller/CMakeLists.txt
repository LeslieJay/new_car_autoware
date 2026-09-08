cmake_minimum_required(VERSION 3.14)
project(agv_high_precision_reverse_controller)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(precision_reverse_controller SHARED
  src/precision_reverse_controller.cpp
)

ament_auto_add_executable(${PROJECT_NAME}_node_exe
  src/high_precision_reverse_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node_exe precision_reverse_controller)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_precision_reverse_controller
    test/test_precision_reverse_controller.cpp
  )
  target_link_libraries(test_precision_reverse_controller precision_reverse_controller)

  ament_add_pytest_test(test_automatic_mode_handoff
    test/test_automatic_mode_handoff.py
    TIMEOUT 30
  )

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
  USE_SCOPED_HEADER_INSTALL_DIR
)
