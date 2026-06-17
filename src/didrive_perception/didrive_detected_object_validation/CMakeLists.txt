cmake_minimum_required(VERSION 3.14)
project(didrive_detected_object_validation)

find_package(autoware_cmake REQUIRED)
autoware_package()

### Find OpenCV Dependencies
find_package(OpenCV REQUIRED)

### Find PCL Dependencies
find_package(PCL REQUIRED QUIET COMPONENTS common search filters segmentation)

### Find Eigen Dependencies
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

### Find Boost Dependencies
find_package(Boost REQUIRED)

include_directories(
  include
  SYSTEM
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_COMMON_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/lanelet_filter/debug.cpp
  src/lanelet_filter/lanelet_filter.cpp
  lib/utils/utils.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "didrive::detected_object_validation::lanelet_filter::ObjectLaneletFilterNode"
  EXECUTABLE object_lanelet_filter_node
)
if(BUILD_TESTING)

  ament_auto_add_gtest(object_lanelet_filter_tests
    test/test_utils.cpp
    test/lanelet_filter/test_lanelet_filter.cpp
  )
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
