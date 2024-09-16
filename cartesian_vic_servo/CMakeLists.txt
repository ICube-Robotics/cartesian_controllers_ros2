cmake_minimum_required(VERSION 3.8)
project(cartesian_vic_servo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(DEPENDENCIES
  rclcpp
  realtime_tools
  # Msgs
  geometry_msgs
  sensor_msgs
  cartesian_vic_controller
  cartesian_control_msgs

  moveit_ros_planning_interface
  moveit_msgs
  # Servo deps (see demos)
  moveit_servo
  moveit_ros_planning
  moveit_ros_planning_interface
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

add_executable(
  vic_servo_node
    src/vic_servo_node.cpp
)
target_include_directories(vic_servo_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(vic_servo_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(vic_servo_node PUBLIC ${DEPENDENCIES})


add_executable(
  demo_twist
    src/demo_twist.cpp
)
target_include_directories(demo_twist PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(demo_twist PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(demo_twist PUBLIC ${DEPENDENCIES})

install(TARGETS vic_servo_node demo_twist
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
