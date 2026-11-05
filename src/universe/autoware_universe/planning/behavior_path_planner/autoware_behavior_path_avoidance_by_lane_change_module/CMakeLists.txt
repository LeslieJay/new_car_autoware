cmake_minimum_required(VERSION 3.14)
project(autoware_behavior_path_avoidance_by_lane_change_module)

find_package(autoware_cmake REQUIRED)
autoware_package()
pluginlib_export_plugin_description_file(autoware_behavior_path_planner plugins.xml)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/scene.cpp
  src/manager.cpp
  src/interface.cpp
  src/opposite_lane.cpp
)

if(BUILD_TESTING)
  add_executable(test_borrowing_math test/test_borrowing_math.cpp)
  target_include_directories(test_borrowing_math PRIVATE src)
  target_compile_features(test_borrowing_math PRIVATE cxx_std_17)
  # Assertions remain active in RelWithDebInfo/Release builds.
  target_compile_options(test_borrowing_math PRIVATE -UNDEBUG)
  add_test(NAME test_borrowing_math COMMAND test_borrowing_math)

  ament_add_ros_isolated_gmock(test_opposite_lane test/test_opposite_lane.cpp)
  target_link_libraries(test_opposite_lane ${PROJECT_NAME})
  target_include_directories(test_opposite_lane PRIVATE src)

  ament_add_ros_isolated_gmock(test_${PROJECT_NAME}
    test/test_behavior_path_planner_node_interface.cpp
  )

  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}
  )

  target_include_directories(test_${PROJECT_NAME} PRIVATE src)
endif()

ament_auto_package(INSTALL_TO_SHARE config)
