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
/// \authors: Thibault Poignonec, Maciej Bednarczyk

// Based on package "ros2_controllers/admittance_controller", Copyright (c) 2022, PickNik, Inc.

#include "cartesian_admittance_controller/cartesian_admittance_controller.hpp"
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/wrench.hpp"
#include "rcutils/logging_macros.h"
#include "tf2_ros/buffer.h"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "cartesian_control_msgs/msg/cartesian_trajectory_point.hpp"

namespace cartesian_admittance_controller
{
controller_interface::CallbackReturn CartesianAdmittanceController::on_init()
{
  // initialize controller config
  try {
    parameter_handler_ =
      std::make_shared<cartesian_admittance_controller::ParamListener>(get_node());
    cartesian_admittance_controller::Params parameters = parameter_handler_->get_params();

    if (!parameters.admittance.plugin_name.empty() &&
      !parameters.admittance.plugin_package.empty())
    {
      admittance_loader_ =
        std::make_shared<pluginlib::ClassLoader<cartesian_admittance_controller::CartesianAdmittanceRule>>(
        parameters.admittance.plugin_package,
        "cartesian_admittance_controller::CartesianAdmittanceRule");
      admittance_ = std::unique_ptr<cartesian_admittance_controller::CartesianAdmittanceRule>(
        admittance_loader_->createUnmanagedInstance(parameters.admittance.plugin_name));
    } else {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Please provide 'admittance.plugin_package' and 'admittance.plugin_name' parameters!");
      return controller_interface::CallbackReturn::ERROR;
    }
    // Initialize admittance rule plugin
    if (admittance_->init(parameter_handler_) == controller_interface::return_type::ERROR) {
      return controller_interface::CallbackReturn::ERROR;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // number of joints in controllers is fixed after initialization
  num_joints_ = admittance_->parameters_.joints.size();

  // allocate dynamic memory
  joint_state_.positions.assign(num_joints_, 0.0);
  joint_state_.velocities.assign(num_joints_, 0.0);
  joint_state_.accelerations.assign(num_joints_, 0.0);

  joint_command_ = joint_state_;
  last_commanded_joint_state_ = joint_state_;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration CartesianAdmittanceController::
command_interface_configuration()
const
{
  std::vector<std::string> command_interfaces_config_names;
  for (const auto & interface : admittance_->parameters_.command_interfaces) {
    for (const auto & joint : command_joint_names_) {
      auto full_name = joint + "/" + interface;
      command_interfaces_config_names.push_back(full_name);
    }
  }

  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    command_interfaces_config_names};
}

controller_interface::InterfaceConfiguration
CartesianAdmittanceController::state_interface_configuration()
const
{
  std::vector<std::string> state_interfaces_config_names;
  for (size_t i = 0; i < admittance_->parameters_.state_interfaces.size(); ++i) {
    const auto & interface = admittance_->parameters_.state_interfaces[i];
    for (const auto & joint : admittance_->parameters_.joints) {
      auto full_name = joint + "/" + interface;
      state_interfaces_config_names.push_back(full_name);
    }
  }

  auto ft_interfaces = force_torque_sensor_->get_state_interface_names();
  state_interfaces_config_names.insert(
    state_interfaces_config_names.end(), ft_interfaces.begin(), ft_interfaces.end());

  return {
    controller_interface::interface_configuration_type::INDIVIDUAL, state_interfaces_config_names};
}

controller_interface::return_type CartesianAdmittanceController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Realtime constraints are required in this function
  if (!admittance_) {
    return controller_interface::return_type::ERROR;
  }

  // update compliant frme(s) reference from ros subscriber message
  reference_compliant_frame_trajectory_msg_ = *input_compliant_frame_trajectory_msg_.readFromRT();
  if (reference_compliant_frame_trajectory_msg_.get())
  {
    admittance_->update_compliant_frame_trajectory(*reference_compliant_frame_trajectory_msg_.get());
  }

  // get all controller inputs
  read_state_from_hardware(joint_state_, ft_values_);

  // apply admittance control to reference to determine desired state
  admittance_->update(
    joint_state_,
    ft_values_,
    period,
    joint_command_
  );

  // write calculated values to joint interfaces
  write_state_to_hardware(joint_command_);

  // Publish controller state
  state_publisher_->lock();
  // state_publisher_->msg_ = admittance_->get_controller_state();
  state_publisher_->unlockAndPublish();

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn CartesianAdmittanceController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!admittance_) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Error at 'on_configure()': The admittance rule plugin should have been initialized before!"
    );
    return CallbackReturn::FAILURE;
  }
  command_joint_names_ = admittance_->parameters_.command_joints;
  //TODO: new parameter like "velocity_cmd_interface_names"
  if (command_joint_names_.empty()) {
    command_joint_names_ = admittance_->parameters_.joints;
    RCLCPP_INFO(
      get_node()->get_logger(),
      "No specific joint names are used for command interfaces. Using 'joints' parameter.");
  } else if (command_joint_names_.size() != num_joints_) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "'command_joints' parameter has to have the same size as 'joints' parameter.");
    return CallbackReturn::FAILURE;
  }

  // print and validate interface types
  for (const auto & tmp : admittance_->parameters_.state_interfaces) {
    RCLCPP_INFO(get_node()->get_logger(), "%s", ("state int types are: " + tmp + "\n").c_str());
  }
  for (const auto & tmp : admittance_->parameters_.command_interfaces) {
    RCLCPP_INFO(get_node()->get_logger(), "%s", ("command int types are: " + tmp + "\n").c_str());
  }

  // Check if only allowed interface types are used and initialize storage to avoid memory
  // allocation during activation
  auto contains_interface_type =
    [](const std::vector<std::string> & interface_type_list, const std::string & interface_type)
    {
      return std::find(interface_type_list.begin(), interface_type_list.end(), interface_type) !=
             interface_type_list.end();
    };

  joint_command_interface_.resize(allowed_interface_types_.size());
  for (const auto & interface : admittance_->parameters_.command_interfaces) {
    auto it =
      std::find(allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    if (it == allowed_interface_types_.end()) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Command interface type '%s' not allowed!", interface.c_str());
      return CallbackReturn::FAILURE;
    }
  }

  has_position_command_interface_ = contains_interface_type(
    admittance_->parameters_.command_interfaces, hardware_interface::HW_IF_POSITION);
  has_velocity_command_interface_ = contains_interface_type(
    admittance_->parameters_.command_interfaces, hardware_interface::HW_IF_VELOCITY);
  has_acceleration_command_interface_ = contains_interface_type(
    admittance_->parameters_.command_interfaces, hardware_interface::HW_IF_ACCELERATION);
  has_effort_command_interface_ = contains_interface_type(
    admittance_->parameters_.command_interfaces, hardware_interface::HW_IF_EFFORT);

  // Check if only allowed interface types are used and initialize storage to avoid memory
  // allocation during activation
  joint_state_interface_.resize(allowed_interface_types_.size());
  for (const auto & interface : admittance_->parameters_.state_interfaces) {
    auto it =
      std::find(allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    if (it == allowed_interface_types_.end()) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "State interface type '%s' not allowed!", interface.c_str());
      return CallbackReturn::FAILURE;
    }
  }

  has_position_state_interface_ = contains_interface_type(
    admittance_->parameters_.state_interfaces, hardware_interface::HW_IF_POSITION);
  has_velocity_state_interface_ = contains_interface_type(
    admittance_->parameters_.state_interfaces, hardware_interface::HW_IF_VELOCITY);
  has_acceleration_state_interface_ = contains_interface_type(
    admittance_->parameters_.state_interfaces, hardware_interface::HW_IF_ACCELERATION);

  auto get_interface_list = [](const std::vector<std::string> & interface_types)
    {
      std::stringstream ss_command_interfaces;
      for (size_t index = 0; index < interface_types.size(); ++index) {
        if (index != 0) {
          ss_command_interfaces << " ";
        }
        ss_command_interfaces << interface_types[index];
      }
      return ss_command_interfaces.str();
    };
  RCLCPP_INFO(
    get_node()->get_logger(), "Command interfaces are [%s] and and state interfaces are [%s].",
    get_interface_list(admittance_->parameters_.command_interfaces).c_str(),
    get_interface_list(admittance_->parameters_.state_interfaces).c_str());

  // setup subscribers and publishers
  auto compliant_frame_trajectory_callback =
    [this](const std::shared_ptr<cartesian_control_msgs::msg::CompliantFrameTrajectory> msg)
    {input_compliant_frame_trajectory_msg_.writeFromNonRT(msg);};
  input_compliant_frame_trajectory_subscriber_ =
    get_node()->create_subscription<cartesian_control_msgs::msg::CompliantFrameTrajectory>(
    "~/reference_compliant_frame_trajectory",
    rclcpp::SystemDefaultsQoS(), compliant_frame_trajectory_callback);
  s_publisher_ = get_node()->create_publisher<control_msgs::msg::AdmittanceControllerState>(
    "~/status", rclcpp::SystemDefaultsQoS());
  state_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<ControllerStateMsg>>(s_publisher_);

  // Initialize state message
  state_publisher_->lock();
  //state_publisher_->msg_ = admittance_->get_controller_state();
  state_publisher_->unlock();

  // Initialize FTS semantic semantic_component
  force_torque_sensor_ = std::make_unique<semantic_components::ForceTorqueSensor>(
    semantic_components::ForceTorqueSensor(admittance_->parameters_.ft_sensor.name));

  // configure admittance rule
  if (admittance_->configure(get_node(), num_joints_) == controller_interface::return_type::ERROR) {
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianAdmittanceController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // on_activate is called when the lifecycle node activates.
  if (!admittance_) {
    return controller_interface::CallbackReturn::ERROR;
  }

  // order all joints in the storage
  for (const auto & interface : admittance_->parameters_.state_interfaces) {
    auto it =
      std::find(allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    auto index = std::distance(allowed_interface_types_.begin(), it);
    if (!controller_interface::get_ordered_interfaces(
        state_interfaces_, admittance_->parameters_.joints, interface,
        joint_state_interface_[index]))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Expected %zu '%s' state interfaces, got %zu.", num_joints_,
        interface.c_str(), joint_state_interface_[index].size());
      return CallbackReturn::ERROR;
    }
  }
  for (const auto & interface : admittance_->parameters_.command_interfaces) {
    auto it =
      std::find(allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    auto index = std::distance(allowed_interface_types_.begin(), it);
    if (!controller_interface::get_ordered_interfaces(
        command_interfaces_, command_joint_names_, interface, joint_command_interface_[index]))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Expected %zu '%s' command interfaces, got %zu.", num_joints_,
        interface.c_str(), joint_command_interface_[index].size());
      return CallbackReturn::ERROR;
    }
  }

  // update parameters if any have changed
  admittance_->apply_parameters_update();

  // initialize interface of the FTS semantic component
  force_torque_sensor_->assign_loaned_state_interfaces(state_interfaces_);

  // initialize states
  read_state_from_hardware(joint_state_, ft_values_);
  for (auto val : joint_state_.positions) {
    if (std::isnan(val)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to read joint positions from the hardware.\n");
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  // number of joints in controllers is fixed after initialization
  num_joints_ = admittance_->parameters_.joints.size();

  // Use current joint_state as a default reference
  joint_command_ = joint_state_;
  last_commanded_joint_state_ = joint_state_;

  // TODO: set cartesian_reference_ from "joint_state_" using kinematics

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianAdmittanceController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!admittance_) {
    return controller_interface::CallbackReturn::ERROR;
  }

  // release force torque sensor interface
  force_torque_sensor_->release_interfaces();

  // reset to prevent stale references
  for (size_t index = 0; index < allowed_interface_types_.size(); ++index) {
    joint_command_interface_[index].clear();
    joint_state_interface_[index].clear();
  }
  release_interfaces();
  admittance_->reset(num_joints_);

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianAdmittanceController::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianAdmittanceController::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!admittance_) {
    return controller_interface::CallbackReturn::ERROR;
  }
  admittance_->reset(num_joints_);
  return controller_interface::CallbackReturn::SUCCESS;
}

void CartesianAdmittanceController::read_state_from_hardware(
  trajectory_msgs::msg::JointTrajectoryPoint & state_current,
  geometry_msgs::msg::Wrench & ft_values)
{
  // if any interface has nan values, assume state_current is the last command state
  bool nan_position = false;
  bool nan_velocity = false;
  bool nan_acceleration = false;

  size_t pos_ind = 0;
  size_t vel_ind = pos_ind + has_velocity_command_interface_;
  size_t acc_ind = vel_ind + has_acceleration_state_interface_;
  for (size_t joint_ind = 0; joint_ind < num_joints_; ++joint_ind) {
    if (has_position_state_interface_) {
      state_current.positions[joint_ind] =
        state_interfaces_[pos_ind * num_joints_ + joint_ind].get_value();
      nan_position |= std::isnan(state_current.positions[joint_ind]);
    } else if (has_velocity_state_interface_) {
      state_current.velocities[joint_ind] =
        state_interfaces_[vel_ind * num_joints_ + joint_ind].get_value();
      nan_velocity |= std::isnan(state_current.velocities[joint_ind]);
    } else if (has_acceleration_state_interface_) {
      state_current.accelerations[joint_ind] =
        state_interfaces_[acc_ind * num_joints_ + joint_ind].get_value();
      nan_acceleration |= std::isnan(state_current.accelerations[joint_ind]);
    }
  }

  if (nan_position) {
    state_current.positions = last_commanded_joint_state_.positions;
  }
  if (nan_velocity) {
    state_current.velocities = last_commanded_joint_state_.velocities;
  }
  if (nan_acceleration) {
    state_current.accelerations = last_commanded_joint_state_.accelerations;
  }

  // if any ft_values are nan, assume values are zero
  force_torque_sensor_->get_values_as_message(ft_values);
  if (
    std::isnan(ft_values.force.x) || std::isnan(ft_values.force.y) ||
    std::isnan(ft_values.force.z) || std::isnan(ft_values.torque.x) ||
    std::isnan(ft_values.torque.y) || std::isnan(ft_values.torque.z))
  {
    ft_values = geometry_msgs::msg::Wrench();
  }
}

void CartesianAdmittanceController::write_state_to_hardware(
  trajectory_msgs::msg::JointTrajectoryPoint & joint_state_command)
{
  // if any interface has nan values, assume state_commanded is the last command state
  size_t pos_ind = 0;
  size_t vel_ind = pos_ind + has_velocity_command_interface_;
  size_t acc_ind = vel_ind + has_acceleration_state_interface_;
  for (size_t joint_ind = 0; joint_ind < num_joints_; ++joint_ind) {
    if (has_position_command_interface_) {
      command_interfaces_[pos_ind * num_joints_ + joint_ind].set_value(
        joint_state_command.positions[joint_ind]);
    } else if (has_velocity_command_interface_) {
      command_interfaces_[vel_ind * num_joints_ + joint_ind].set_value(
        joint_state_command.positions[joint_ind]);
    } else if (has_acceleration_command_interface_) {
      command_interfaces_[acc_ind * num_joints_ + joint_ind].set_value(
        joint_state_command.positions[joint_ind]);
    }
  }
  last_commanded_joint_state_ = joint_state_command;
}

}  // namespace cartesian_admittance_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  cartesian_admittance_controller::CartesianAdmittanceController,
  controller_interface::ControllerInterface
)
