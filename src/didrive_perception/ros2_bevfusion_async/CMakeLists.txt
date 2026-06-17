cmake_minimum_required(VERSION 3.8)
project(didrive_ros2_bevfusion_async)

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_BUILD_TYPE "Debug")
set(arch ${CMAKE_HOST_SYSTEM_PROCESSOR})

find_package(CUDA REQUIRED)
find_package(Protobuf REQUIRED)
message(WARNING "Protobuf_INCLUDE_DIRS = ${Protobuf_INCLUDE_DIRS}")
message(WARNING "Protobuf_LIBRARIES = ${Protobuf_LIBRARIES}")

# set(spconv_root /media/byd-asd/BEV/work/Lidar_AI_Solution/libraries/3DSparseConvolution/libspconv_cuda12)
# set(spconv_include ${spconv_root}/include)
# set(spconv_lib ${spconv_root}/lib/${arch})
# set(spconv_include ../libraries/3DSparseConvolution/libspconv/include)
# set(spconv_lib ../libraries/3DSparseConvolution/libspconv/lib/${arch}_cuda$ENV{SPCONV_CUDA_VERSION})
# message(STATUS "spconv_lib = ${spconv_lib}")

if("$ENV{SPCONV_CUDA_VERSION}" EQUAL "11.4")
  # For cuda 11.x
  set(CMAKE_CXX_FLAGS_RELEASE "-std=c++14 -Wextra -Wall -Wno-missing-field-initializers -Wno-deprecated-declarations -O3 -DENABLE_TEXT_BACKEND_STB -Wno-unused-parameter")
  set(CMAKE_CXX_FLAGS_DEBUG   "-std=c++14 -O0 -g -DENABLE_TEXT_BACKEND_STB")
else()
  # For cuda 12.x
  set(CMAKE_CXX_FLAGS_RELEASE "-std=c++17 -Wextra -Wall -Wno-missing-field-initializers -Wno-deprecated-declarations -O3 -DENABLE_TEXT_BACKEND_STB -Wno-unused-parameter -Wno-dev")
  set(CMAKE_CXX_FLAGS_DEBUG   "-std=c++17 -O0 -g -DENABLE_TEXT_BACKEND_STB")
endif()

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} 
  -gencode arch=compute_$ENV{CUDASM},code=compute_$ENV{CUDASM}
)

include_directories(
  ${Protobuf_INCLUDE_DIRS}
  $ENV{CUDA_Inc}
  $ENV{TensorRT_Inc}
  $ENV{Python_Inc}
  include/
  include/bevfusion/
)

link_directories(
  $ENV{CUDA_Lib}
  $ENV{TensorRT_Lib}
  ${spconv_lib}
  ${CMAKE_CURRENT_SOURCE_DIR}/lib
  $ENV{Python_Lib}
)

########################## ros2_bevfusion_node lib ################################

find_package(ament_cmake_auto REQUIRED)

cmake_policy(SET CMP0074 NEW)
find_package(PCL 1.10 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# 查找 ros2 包
find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(autoware_perception_msgs REQUIRED)
find_package(autoware_universe_utils REQUIRED)
find_package(autoware_object_recognition_utils REQUIRED)

ament_auto_find_build_dependencies()

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/bevfusion_node_async.cpp
  src/detection_class_remapper.cpp
)
# cuda_add_library(${PROJECT_NAME} SHARED
#   src/bevfusion_node.cpp
# )

target_include_directories(${PROJECT_NAME}
  SYSTEM PUBLIC
  ${CUDA_INCLUDE_DIRS}
  $(autoware_point_types_INCLUDE_DIRS)
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  bevfusion_core
  libnvinfer.so
  ${Protobuf_LIBRARIES}
  libnvinfer_plugin.so
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
  TENSORRT_VERSION_MAJOR=${TENSORRT_VERSION_MAJOR}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "didrive::ros2_bevfusion_async::NVBEVFusionNodeAsync"
  EXECUTABLE ${PROJECT_NAME}_node
)

install(
  TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the models directory
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
  PATTERN ".git" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)

# ament_auto_package()
#######################################################################
