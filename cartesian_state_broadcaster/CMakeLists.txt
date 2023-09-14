cmake_minimum_required(VERSION 3.5)
project(cartesian_state_broadcaster)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(controller_interface REQUIRED)
find_package(cartesian_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(kinematics_interface REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

add_library(
  cartesian_state_broadcaster
  SHARED
  src/cartesian_state_broadcaster.cpp
)
target_include_directories(
  cartesian_state_broadcaster
  PUBLIC
  include
)
ament_target_dependencies(
  cartesian_state_broadcaster
  Eigen3
  cartesian_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  kinematics_interface
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(cartesian_state_broadcaster PRIVATE "EXTERNAL_TORQUE_SENSOR_BROADCASTER_BUILDING_DLL")
# prevent pluginlib from using boost
target_compile_definitions(cartesian_state_broadcaster PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  controller_interface controller_plugin.xml)

install(
  TARGETS
  cartesian_state_broadcaster
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  cartesian_state_broadcaster
)
ament_export_dependencies(
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  cartesian_msgs
  hardware_interface
  kinematics_interface
  Eigen3
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
