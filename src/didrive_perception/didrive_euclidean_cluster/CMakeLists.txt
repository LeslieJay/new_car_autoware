cmake_minimum_required(VERSION 3.14)
project(didrive_euclidean_cluster)
add_compile_options(-O3 -DNDEBUG)

set (BUILD_TESTING false)
find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
if(OpenMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
include_directories(
  include
  SYSTEM
  ${PCL_COMMON_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

ament_auto_add_library(${PROJECT_NAME}_lib SHARED
  lib/euclidean_cluster.cpp
  lib/voxel_grid_based_euclidean_cluster.cpp
  lib/utils.cpp
)

target_link_libraries(${PROJECT_NAME}_lib
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_auto_add_library(${PROJECT_NAME}_node_core SHARED
  src/euclidean_cluster_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node_core
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${PROJECT_NAME}_lib
)

rclcpp_components_register_node(${PROJECT_NAME}_node_core
  PLUGIN "didrive::euclidean_cluster::EuclideanClusterNode"
  EXECUTABLE euclidean_cluster_node
)

ament_auto_add_library(${PROJECT_NAME}_voxel_grid_node_core SHARED
  src/voxel_grid_based_euclidean_cluster_node.cpp
)
target_link_libraries(${PROJECT_NAME}_voxel_grid_node_core
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${PROJECT_NAME}_lib
)

rclcpp_components_register_node(${PROJECT_NAME}_voxel_grid_node_core
  PLUGIN "didrive::euclidean_cluster::VoxelGridBasedEuclideanClusterNode"
  EXECUTABLE voxel_grid_based_euclidean_cluster_node
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_auto_add_gtest(test_voxel_grid_based_euclidean_cluster_fusion
    test/test_voxel_grid_based_euclidean_cluster.cpp
  )
endif()

ament_auto_package(INSTALL_TO_SHARE
    launch
    config
    test
)
