cmake_minimum_required(VERSION 3.16)
project(didrive_patchwork VERSION 1.0.4 LANGUAGES CXX)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -g")
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fsanitize=address -g")

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package (ament_cmake_auto REQUIRED) 
ament_auto_find_build_dependencies ()

# 查找并包含 Eigen3
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS
    core
    imgproc
    imgcodecs
    highgui
    calib3d
)


include_directories(
  include
  SYSTEM
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
ament_auto_add_library (${PROJECT_NAME} SHARED 
    src/patchworkpp_seg_node.cpp
    include/patchworkpp/src/patchworkpp.cpp)

target_link_libraries(didrive_patchwork
    ${OpenCV_LIBS}  # 包含 core, imgproc, highgui
)

rclcpp_components_register_node(${PROJECT_NAME} PLUGIN "didrive::patchworkpp_ros::PatchworkppSegNode" EXECUTABLE patchworkpp_seg_node)

ament_auto_package (
	INSTALL_TO_SHARE
	launch
	rviz)
