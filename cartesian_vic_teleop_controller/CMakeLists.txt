cmake_minimum_required(VERSION 3.16)
project(cartesian_vic_teleop_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Use CMake to pass the current ROS_DISTRO via variables into a preprocessor template.
# We then include this file and switch between the different APIs.
if($ENV{ROS_DISTRO} STREQUAL "rolling")
  set(CARTESIAN_CONTROLLERS_ROLLING TRUE)
elseif($ENV{ROS_DISTRO} STREQUAL "jazzy")
        set(CARTESIAN_CONTROLLERS_JAZZY TRUE)
elseif($ENV{ROS_DISTRO} STREQUAL "humble")
        set(CARTESIAN_CONTROLLERS_HUMBLE TRUE)
else()
        message(WARNING "ROS2 version must be {rolling|jazzy|humble}")
endif()
configure_file(include/cartesian_vic_teleop_controller/Ros2VersionConfig.h.in include/cartesian_vic_teleop_controller/Ros2VersionConfig.h)

set(DEPENDENCIES
  Eigen3
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
  cartesian_vic_controller
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED
  # ROS2 controller impl.
  src/cartesian_vic_teleop_controller.cpp
  # Logic
  src/cartesian_vic_teleop_logic.cpp
  src/mapping_manager.cpp
  src/teleop_data.cpp
  src/teleop_rule.cpp
  # Async node impl.
  # TODO
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  # $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/rules>  # LP solvers
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>  # ROS2VersionConfig.h
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${DEPENDENCIES})

# Coin-Clp LP solver
# sudo apt-get install  coinor-clp coinor-libclp-dev -y
set(COIN_CLP_HEADERS /usr/include/coin)
set(COIN_CLP_LIBRARIES Clp ClpSolver)
target_include_directories(${PROJECT_NAME} PUBLIC ${COIN_CLP_HEADERS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${COIN_CLP_LIBRARIES})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_compile_definitions(${PROJECT_NAME} PRIVATE "cartesian_vic_teleop_controller_BUILDING_DLL")

pluginlib_export_plugin_description_file(controller_interface cartesian_vic_teleop_controller.xml)


# VIC-based teleoperation rule plugins library
add_library(cartesian_vic_teleop_rules SHARED
  src/rules/vanilla_teleop_rule.cpp
)
target_compile_features(cartesian_vic_teleop_rules PUBLIC cxx_std_17)
target_include_directories(cartesian_vic_teleop_rules PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/cartesian_vic_teleop_controller>
)
target_link_libraries(cartesian_vic_teleop_rules PUBLIC
  cartesian_vic_teleop_controller
)
ament_target_dependencies(
  cartesian_vic_teleop_rules
  PUBLIC
    ${DEPENDENCIES}
)
pluginlib_export_plugin_description_file(cartesian_vic_teleop_controller cartesian_vic_teleop_rules.xml)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS
    ${PROJECT_NAME}
    cartesian_vic_teleop_rules
  EXPORT export_cartesian_vic_teleop_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_cartesian_vic_teleop_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})
ament_package()
