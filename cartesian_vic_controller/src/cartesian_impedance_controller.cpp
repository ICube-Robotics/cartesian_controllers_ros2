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
/// \authors: Thibault Poignonec


#include "cartesian_vic_controller/cartesian_impedance_controller.hpp"

namespace cartesian_vic_controller
{
controller_interface::CallbackReturn CartesianImpedanceController::on_init()
{
  // initialize controller config
  if (CartesianVicController::on_init() != controller_interface::CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to initialize CartesianImpedanceController");
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianImpedanceController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (vic_->get_control_mode() != ControlMode::IMPEDANCE) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Invalid control mode for CartesianImpedanceController (expected IMPEDANCE)");
    return controller_interface::return_type::ERROR;
  }
  auto ret = CartesianVicController::update(time, period);
  if (ret != controller_interface::return_type::OK) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to update CartesianImpedanceController");
  }
  return ret;
}

controller_interface::CallbackReturn CartesianImpedanceController::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  auto ret = CartesianVicController::on_configure(previous_state);
  if (ret != controller_interface::CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to configure CartesianImpedanceController");
  }
  return ret;
}

controller_interface::CallbackReturn CartesianImpedanceController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  auto ret = CartesianVicController::on_activate(previous_state);
  if (ret != controller_interface::CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to activate CartesianImpedanceController");
  }
  return ret;
}

controller_interface::CallbackReturn CartesianImpedanceController::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  auto ret = CartesianVicController::on_deactivate(previous_state);
  if (ret != controller_interface::CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to deactivate CartesianImpedanceController");
  }
  return ret;
}

controller_interface::CallbackReturn CartesianImpedanceController::on_cleanup(
  const rclcpp_lifecycle::State & previous_state)
{
  auto ret = CartesianVicController::on_cleanup(previous_state);
  if (ret != controller_interface::CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to cleanup CartesianImpedanceController");
  }
  return ret;
}

controller_interface::CallbackReturn CartesianImpedanceController::on_error(
  const rclcpp_lifecycle::State & previous_state)
{
  auto ret = CartesianVicController::on_error(previous_state);
  if (ret != controller_interface::CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "CartesianImpedanceController failed to recover from error!");
  }
  return ret;
}

bool CartesianImpedanceController::is_command_interfaces_config_valid() const
{
  bool all_ok = true;

  if (!has_position_command_interface_ && !has_velocity_command_interface_) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "CartesianImpedanceController: No position or velocity command interface specified!");
    all_ok = false;
  }

  if (has_effort_command_interface_) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "CartesianImpedanceController: unsupported effort command interface specified!");
    all_ok = false;
  }

  if (vic_->get_control_mode() != ControlMode::ADMITTANCE) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "CartesianImpedanceController: invalid VIC rule control mode (expected ADMITTANCE)!");
    all_ok = false;
  }

  return all_ok;
}

bool CartesianImpedanceController::write_state_to_hardware(
  trajectory_msgs::msg::JointTrajectoryPoint & joint_state_c)
{
  return write_impedance_state_to_hardware(joint_state_c);
}

}  // namespace cartesian_vic_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  cartesian_vic_controller::CartesianImpedanceController,
  controller_interface::ControllerInterface
)
