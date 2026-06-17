cmake_minimum_required(VERSION 3.5)
project(tamagawa_imu_driver)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

### Find Packages
find_package(ament_cmake_auto REQUIRED)
# if ENABLE_AGNOCAST environment variable is set to 1, enable agnocast components
# and define a compile definition to conditionally compile code that depends on agnocast
set(ENABLE_AGNOCAST_DEFAULT "$ENV{ENABLE_AGNOCAST}")
if(ENABLE_AGNOCAST_DEFAULT STREQUAL "1")
  set(ENABLE_AGNOCAST_DEFAULT "ON")
else()
  # default to OFF when the environment variable is unset or not equal to "1"
  set(ENABLE_AGNOCAST_DEFAULT "OFF")
endif()

option(ENABLE_AGNOCAST "Enable Agnocast components" ${ENABLE_AGNOCAST_DEFAULT})

if(ENABLE_AGNOCAST)
  find_package(agnocastlib REQUIRED)
  find_package(agnocast_cie_thread_configurator REQUIRED)
endif()

ament_auto_find_build_dependencies()

ament_auto_add_executable(tag_serial_driver
	src/tag_serial_driver.cpp
)

ament_auto_add_executable(tag_can_driver
	src/tag_can_driver.cpp
)

if(ENABLE_AGNOCAST)
  target_compile_definitions(tag_can_driver PRIVATE USE_AGNOCAST_ENABLED)
  ament_target_dependencies(tag_can_driver agnocastlib)
  target_compile_definitions(tag_serial_driver PRIVATE USE_AGNOCAST_ENABLED)
  ament_target_dependencies(tag_serial_driver agnocastlib agnocast_cie_thread_configurator)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
)
