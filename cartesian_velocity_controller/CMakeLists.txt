cmake_minimum_required(VERSION 3.5)
project(cartesian_velocity_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  generate_parameter_library
  controller_interface
  realtime_tools
  kinematics_interface
  geometry_msgs
  Eigen3
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(${EIGEN3_INCLUDE_DIR})

add_library(
  cartesian_velocity_controller
  SHARED
  src/cartesian_velocity_controller.cpp
)
target_include_directories(
  cartesian_velocity_controller
  PUBLIC
  include
)

# Add library of the controller and export it
generate_parameter_library(cartesian_velocity_controller_parameters
  src/cartesian_velocity_controller_parameters.yaml
)

target_link_libraries(cartesian_velocity_controller PUBLIC cartesian_velocity_controller_parameters)
ament_target_dependencies(cartesian_velocity_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(cartesian_velocity_controller PRIVATE "EXTERNAL_TORQUE_SENSOR_BROADCASTER_BUILDING_DLL")
# prevent pluginlib from using boost
target_compile_definitions(cartesian_velocity_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(controller_interface controller_plugin.xml)

install(
  TARGETS
  cartesian_velocity_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_lint_auto_find_test_dependencies()

  ament_add_gmock(
    test_load_cartesian_velocity_controller
    test/test_load_cartesian_velocity_controller.cpp
  )

  target_include_directories(
    test_load_cartesian_velocity_controller
    PRIVATE
    include
  )

  ament_target_dependencies(
    test_load_cartesian_velocity_controller
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )
endif()

ament_export_include_directories(
  include
)

ament_export_libraries(
  cartesian_velocity_controller
)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
