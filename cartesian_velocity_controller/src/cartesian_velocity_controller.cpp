// Copyright 2023, ICube Laboratory, University of Strasbourg
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

#include "cartesian_velocity_controller/cartesian_velocity_controller.hpp"
#include "controller_interface/helpers.hpp"

namespace cartesian_velocity_controller
{

CallbackReturn CartesianVelocityController::on_init()
{
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianVelocityController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();
  command_joint_names_ = params_.command_joints;
  reference_joint_names_ = params_.reference_joints;

  joint_positions_.resize(command_joint_names_.size());
  joint_velocities_.resize(command_joint_names_.size());
  cart_velocities_.resize(6);

  for (const auto & joint : command_joint_names_) {
    command_interface_names_.push_back(joint + "/velocity");
    state_interface_names_.push_back(joint + "/position");
  }

  if (!reference_joint_names_.empty() && reference_joint_names_.size() != 6) {
    RCLCPP_ERROR(
      rclcpp::get_logger("CartesianVelocityController"),
      "Exactly 6 reference joint names expected. Got %zu",
      reference_joint_names_.size()
    );
    return CallbackReturn::ERROR;
  }
  for (const auto & joint : reference_joint_names_) {
    reference_interface_names_.push_back(joint + "/velocity");
  }

  // Load the differential IK plugin
  if (!params_.kinematics.plugin_name.empty()) {
    try {
      kinematics_loader_ = std::make_shared<
        pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>(
        params_.kinematics.plugin_package, "kinematics_interface::KinematicsInterface");
      kinematics_ = std::unique_ptr<kinematics_interface::KinematicsInterface>(
        kinematics_loader_->createUnmanagedInstance(params_.kinematics.plugin_name));
      if (!kinematics_->initialize(
          get_node()->get_node_parameters_interface(),
          params_.kinematics.tip))
      {
        RCLCPP_ERROR(
          rclcpp::get_logger("CartesianVelocityController"),
          "Exception while loading the IK plugin");
        return CallbackReturn::ERROR;
      }
    } catch (pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR(
        rclcpp::get_logger("CartesianVelocityController"),
        "Exception while loading the IK plugin '%s': '%s'",
        params_.kinematics.plugin_name.c_str(), ex.what());
      return CallbackReturn::ERROR;
    }
  } else {
    RCLCPP_ERROR(
      rclcpp::get_logger("CartesianVelocityController"),
      "A differential IK plugin name was not specified in the config file.");
    return CallbackReturn::ERROR;
  }

  twist_cmd_sub_ = this->get_node()->create_subscription<DataType>(
    "~/twist_commands", rclcpp::SystemDefaultsQoS(),
    [this](const DataType::SharedPtr msg) {
      rt_buffer_ptr_.writeFromNonRT(msg);
    }
  );

  // pre-reserve command interfaces
  command_interfaces_.reserve(command_joint_names_.size());
  reference_interfaces_.resize(6, std::numeric_limits<double>::quiet_NaN());

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
CartesianVelocityController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = command_interface_names_;
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
CartesianVelocityController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names = state_interface_names_;
  return state_interfaces_config;
}

CallbackReturn CartesianVelocityController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
  ordered_interfaces;
  if (
    !controller_interface::get_ordered_interfaces(
      command_interfaces_, command_interface_names_, std::string(""), ordered_interfaces) ||
    command_interface_names_.size() != ordered_interfaces.size())
  {
    RCLCPP_ERROR(
      this->get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
      command_interface_names_.size(), ordered_interfaces.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // reset command buffer if a command came through callback when controller was inactive
  rt_buffer_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<DataType>>(nullptr);

  RCLCPP_INFO(this->get_node()->get_logger(), "activate successful");

  std::fill(
    reference_interfaces_.begin(), reference_interfaces_.end(),
    std::numeric_limits<double>::quiet_NaN());

  return CallbackReturn::SUCCESS;
}

bool CartesianVelocityController::on_set_chained_mode(bool /*chained_mode*/) {return true;}

CallbackReturn CartesianVelocityController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // reset command buffer
  rt_buffer_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<DataType>>(nullptr);

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianVelocityController::update_and_write_commands(
  const rclcpp::Time & /* time */,
  const rclcpp::Duration & /* period */)
{
  for (auto j = 0ul; j < command_joint_names_.size(); j++) {
    joint_positions_(j) = state_interfaces_[j].get_value();
  }

  for (auto j = 0ul; j < reference_interface_names_.size(); j++) {
    cart_velocities_(j) = reference_interfaces_[j];
  }

  if (!kinematics_->convert_cartesian_deltas_to_joint_deltas(
      joint_positions_,
      cart_velocities_,
      params_.kinematics.tip,
      joint_velocities_))
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("CartesianVelocityController"),
      "convert_cartesian_deltas_to_joint_deltas failed.");
    return controller_interface::return_type::ERROR;
  }

  for (auto j = 0ul; j < command_joint_names_.size(); j++) {
    command_interfaces_[j].set_value(joint_velocities_(j));
  }


  return controller_interface::return_type::OK;
}

std::vector<hardware_interface::CommandInterface>
CartesianVelocityController::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  for (size_t i = 0; i < reference_interface_names_.size(); ++i) {
    reference_interfaces.push_back(
      hardware_interface::CommandInterface(
        get_node()->get_name(), reference_interface_names_[i], &reference_interfaces_[i]));
  }
  return reference_interfaces;
}

controller_interface::return_type
CartesianVelocityController::update_reference_from_subscribers()
{
  auto twist_commands = rt_buffer_ptr_.readFromRT();
  // message is valid
  if (!(!twist_commands || !(*twist_commands))) {
    reference_interfaces_.push_back((*twist_commands)->twist.linear.x);
    reference_interfaces_.push_back((*twist_commands)->twist.linear.y);
    reference_interfaces_.push_back((*twist_commands)->twist.linear.z);
    reference_interfaces_.push_back((*twist_commands)->twist.angular.x);
    reference_interfaces_.push_back((*twist_commands)->twist.angular.y);
    reference_interfaces_.push_back((*twist_commands)->twist.angular.z);
  }

  return controller_interface::return_type::OK;
}

}  // namespace cartesian_velocity_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  cartesian_velocity_controller::CartesianVelocityController,
  controller_interface::ChainableControllerInterface)
