cmake_minimum_required(VERSION 3.16)
project(cartesian_admittance_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(DEPENDENCIES
  angles
  control_msgs
  control_toolbox
  controller_interface
  Eigen3
  generate_parameter_library
  geometry_msgs
  hardware_interface
  joint_trajectory_controller
  kinematics_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_kdl
  tf2_ros
  trajectory_msgs
  cartesian_control_msgs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(cartesian_admittance_controller_parameters
  src/cartesian_admittance_controller_parameters.yaml
)

add_library(cartesian_admittance_controller SHARED
  src/cartesian_admittance_controller.cpp
)
target_compile_features(cartesian_admittance_controller PUBLIC cxx_std_17)
target_include_directories(cartesian_admittance_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/cartesian_admittance_controller>
)
target_link_libraries(cartesian_admittance_controller PUBLIC
  cartesian_admittance_controller_parameters
)
ament_target_dependencies(cartesian_admittance_controller PUBLIC ${DEPENDENCIES})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(cartesian_admittance_controller PRIVATE "CARTESIAN_ADMITTANCE_CONTROLLER_BUILDING_DLL")

pluginlib_export_plugin_description_file(controller_interface cartesian_admittance_controller.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

endif()

install(
  DIRECTORY include/
  DESTINATION include/cartesian_admittance_controller
)

install(TARGETS cartesian_admittance_controller cartesian_admittance_controller_parameters
  EXPORT export_cartesian_admittance_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_cartesian_admittance_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})
ament_package()