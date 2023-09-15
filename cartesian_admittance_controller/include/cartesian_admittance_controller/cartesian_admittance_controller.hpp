// Copyright 2023 ICUBE Laboratory, University of Strasbourg
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Thibault Poignonec (thibault.poignonec@gmail.com)

// Heavily based on package "ros2_controllers/admittance_controller", Copyright (c) 2022, PickNik, Inc.

#ifndef ADMITTANCE_CONTROLLER__ADMITTANCE_CONTROLLER_HPP_
#define ADMITTANCE_CONTROLLER__ADMITTANCE_CONTROLLER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

// include generated parameter library
#include "cartesian_admittance_controller_parameters.hpp"

#include "cartesian_admittance_controller/cartesian_admittance_rule.hpp"
#include "cartesian_admittance_controller/visibility_control.h"

#include "controller_interface/chainable_controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "semantic_components/force_torque_sensor.hpp"

// Ros msgs
#include "control_msgs/msg/admittance_controller_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

// Custom msgs
#include "cartesian_control_msgs/msg/cartesian_trajectory.hpp"
#include "cartesian_control_msgs/msg/compliant_frame_trajectory.hpp"

namespace cartesian_admittance_controller
{
using CartesianFrameMsg = cartesian_control_msgs::msg::CartesianTrajectoryPoint;
using ControllerStateMsg = control_msgs::msg::AdmittanceControllerState;

class AdmittanceController : public controller_interface::ChainableControllerInterface
{
public:
  CARTESIAN_ADMITTANCE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  /// Export configuration of required state interfaces.
  /**
   * Allowed types of state interfaces are \ref hardware_interface::POSITION,
   * \ref hardware_interface::VELOCITY, \ref hardware_interface::ACCELERATION.
   */
  CARTESIAN_ADMITTANCE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  /// Export configuration of required state interfaces.
  /**
   * Allowed types of state interfaces are \ref hardware_interface::POSITION,
   * \ref hardware_interface::VELOCITY, \ref hardware_interface::ACCELERATION.
   */
  CARTESIAN_ADMITTANCE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  CARTESIAN_ADMITTANCE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  CARTESIAN_ADMITTANCE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  CARTESIAN_ADMITTANCE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  CARTESIAN_ADMITTANCE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  CARTESIAN_ADMITTANCE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  CARTESIAN_ADMITTANCE_CONTROLLER_PUBLIC
  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  controller_interface::return_type update_reference_from_subscribers() override;

  size_t num_joints_ = 0;
  std::vector<std::string> command_joint_names_;

  // The interfaces are defined as the types in 'allowed_interface_types_' member.
  // For convenience, for each type the interfaces are ordered so that i-th position
  // matches i-th index in joint_names_
  template <typename T>
  using InterfaceReferences = std::vector<std::vector<std::reference_wrapper<T>>>;

  InterfaceReferences<hardware_interface::LoanedCommandInterface> joint_command_interface_;
  InterfaceReferences<hardware_interface::LoanedStateInterface> joint_state_interface_;

  bool has_position_state_interface_ = false;
  bool has_velocity_state_interface_ = false;
  bool has_acceleration_state_interface_ = false;
  bool has_position_command_interface_ = false;
  bool has_velocity_command_interface_ = false;
  bool has_acceleration_command_interface_ = false;
  bool has_effort_command_interface_ = false;

  // To reduce number of variables and to make the code shorter the interfaces are ordered in types
  // as the following constants
  const std::vector<std::string> allowed_interface_types_ = {
    hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
    hardware_interface::HW_IF_ACCELERATION};

  // internal reference values
  const std::vector<std::string> allowed_reference_interfaces_types_ = {
    hardware_interface::HW_IF_VELOCITY
  };
  std::vector<std::reference_wrapper<double>> velocity_reference_;

  // Admittance rule and dependent variables;
  std::unique_ptr<cartesian_admittance_controller::AdmittanceRule> admittance_;

  // force torque sensor
  std::unique_ptr<semantic_components::ForceTorqueSensor> force_torque_sensor_;

  // ROS subscribers
  rclcpp::Subscription<cartesian_control_msgs::msg::CartesianTrajectoryPoint>::SharedPtr
    input_cartesian_reference_subscriber_;
  rclcpp::Publisher<control_msgs::msg::AdmittanceControllerState>::SharedPtr s_publisher_;

  // admittance parameters
  std::shared_ptr<cartesian_admittance_controller::ParamListener> parameter_handler_;

  // ROS messages
  std::shared_ptr<cartesian_control_msgs::msg::CartesianTrajectoryPoint> cartesian_command_msg_;

  // real-time buffer
  realtime_tools::RealtimeBuffer<std::shared_ptr<cartesian_control_msgs::msg::CartesianTrajectoryPoint>>
    input_cartesian_reference_;
  std::unique_ptr<realtime_tools::RealtimePublisher<ControllerStateMsg>> state_publisher_;

  geometry_msgs::msg::Twist last_commanded_twist_;
  cartesian_control_msgs::msg::CartesianTrajectoryPoint last_cartesian_reference_;

  // Control loop data
  // cartesian_reference_: reference cartesian frame read by the controller
  // cartesian_state_: current robot cartesian pose (from joint states and kinematics)
  cartesian_control_msgs::msg::CartesianTrajectoryPoint cartesian_reference_, cartesian_state_;
  // joint_state_: current joint readings from the hardware
  trajectory_msgs::msg::JointTrajectoryPoint joint_state_;
  // cartesian_velocity_command_: twist reference value computed by the controller
  geometry_msgs::msg::Twist cartesian_velocity_command_;
  // ft_values_: values read from the force torque sensor
  geometry_msgs::msg::Wrench ft_values_;

  /**
   * @brief Read values from hardware interfaces and set corresponding fields of joint_state_ and
   * ft_values
   */
  void read_state_from_hardware(
    trajectory_msgs::msg::JointTrajectoryPoint & state_current,
    geometry_msgs::msg::Wrench & ft_values);

  /**
   * @brief Set fields of state_reference with values from controllers exported position and
   * velocity references
   */
  //void read_state_reference_interfaces(cartesian_control_msgs::msg::CartesianTrajectoryPoint & cartesian_reference);

  /**
   * @brief Write values from cartesian_velocity_command to claimed hardware interfaces
   */
  void write_state_to_hardware(const geometry_msgs::msg::Twist & cartesian_velocity_command);
};

}  // namespace cartesian_admittance_controller

#endif  // ADMITTANCE_CONTROLLER__ADMITTANCE_CONTROLLER_HPP_