cmake_minimum_required(VERSION 3.14)
project(didrive_object_class_remapper)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/object_class_remapper_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "didrive::object_class_remapper::ObjectClassRemapperNode"
  EXECUTABLE object_class_remapper_node
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_object_class_remapper test/test_object_class_remapper.cpp)
  target_link_libraries(test_object_class_remapper ${PROJECT_NAME})
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
