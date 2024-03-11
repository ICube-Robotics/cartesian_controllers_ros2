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

// Based on package "ros2_controllers/admittance_controller", Copyright (c) 2022, PickNik, Inc.

#include "cartesian_admittance_controller/cartesian_admittance_controller.hpp"
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/wrench.hpp"
// #include "rcutils/logging_macros.h"
#include "rclcpp/logging.hpp"
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
    // number of joints in controllers is fixed after initialization
    num_joints_ = parameters.joints.size();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // allocate dynamic memory
  joint_state_.positions.assign(num_joints_, 0.0);  // std::nan);
  joint_state_.velocities.assign(num_joints_, 0.0);  //  std::nan);
  joint_state_.accelerations.assign(num_joints_, 0.0);  //  std::nan);

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
  (void) time;
  // Realtime constraints are required in this function
  if (!admittance_) {
    return controller_interface::return_type::ERROR;
  }

  // get all controller inputs
  bool is_state_valid = read_state_from_hardware(joint_state_, ft_values_);

  bool all_ok = true;
  // make sure impedance was initialized
  if (!is_impedance_initialized_ && !is_state_valid) {
    // Exit and wait for valid data...
    return controller_interface::return_type::OK;
  } else if (!is_impedance_initialized_ && is_state_valid) {
    // Init current desired pose from current joint position
    if (!initialize_impedance_rule(joint_state_)) {
      return controller_interface::return_type::ERROR;
    }
  } else if (!is_state_valid) {
    // TODO(tpoignonec): return ERROR?
    all_ok &= false;
    joint_command_ = last_commanded_joint_state_;
  }

  // Control logic
  if (all_ok) {
    // update compliant frme(s) reference from ros subscriber message
    reference_compliant_frame_trajectory_msg_ = *input_compliant_frame_trajectory_msg_.readFromRT();
    if (reference_compliant_frame_trajectory_msg_.get()) {
      admittance_->update_compliant_frame_trajectory(
        *reference_compliant_frame_trajectory_msg_.get());
    }
    // apply admittance control to reference to determine desired state
    auto ret_admittance = admittance_->update(
      joint_state_,
      ft_values_,
      period,
      joint_command_
    );
    if (ret_admittance != controller_interface::return_type::OK) {
      std::fill(
        joint_command_.accelerations.begin(), joint_command_.accelerations.end(), 0.0);
      std::fill(
        joint_command_.velocities.begin(), joint_command_.velocities.end(), 0.0);
      write_state_to_hardware(joint_command_);
      return controller_interface::return_type::ERROR;
    }
  }
  /*
  RCLCPP_INFO(
    get_node()->get_logger(),
    "Joint position cmd: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n",
    joint_command_.positions[0],
    joint_command_.positions[1],
    joint_command_.positions[2],
    joint_command_.positions[3],
    joint_command_.positions[4],
    joint_command_.positions[5],
    joint_command_.positions[6]
  );

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Joint velocity cmd: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f] \n\n",
    joint_command_.velocities[0],
    joint_command_.velocities[1],
    joint_command_.velocities[2],
    joint_command_.velocities[3],
    joint_command_.velocities[4],
    joint_command_.velocities[5],
    joint_command_.velocities[6]
  );
  */

  // write calculated values to joint interfaces
  write_state_to_hardware(joint_command_);

  // Publish controller state
  if (admittance_->controller_state_to_msg(controller_state_msg_) != \
      controller_interface::return_type::OK) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Failled to retrieve Admittance rule state!");
  }
  else {
    controller_state_msg_.header.stamp = get_node()->get_clock()->now();
    state_publisher_->lock();
    state_publisher_->msg_ = controller_state_msg_;
    state_publisher_->unlockAndPublish();
  }
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn CartesianAdmittanceController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
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
        std::make_shared<pluginlib::ClassLoader<
            cartesian_admittance_controller::CartesianAdmittanceRule>>(
        parameters.admittance.plugin_package,
        "cartesian_admittance_controller::CartesianAdmittanceRule"
            );
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
      get_node()->get_logger(), "Exception thrown during configure stage with message: %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  command_joint_names_ = admittance_->parameters_.command_joints;
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

  if (!has_position_command_interface_ && !has_velocity_command_interface_) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "At least the position or velocity command interface must be available!");
    return CallbackReturn::FAILURE;
  }

  if (!has_position_state_interface_ || !has_velocity_state_interface_) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Both the position AND velocity state interface must be available!");
    return CallbackReturn::FAILURE;
  }

  // setup subscribers and publishers
  auto compliant_frame_trajectory_callback =
    [this](const std::shared_ptr<cartesian_control_msgs::msg::CompliantFrameTrajectory> msg)
    {input_compliant_frame_trajectory_msg_.writeFromNonRT(msg);};
  input_compliant_frame_trajectory_subscriber_ =
    get_node()->create_subscription<cartesian_control_msgs::msg::CompliantFrameTrajectory>(
    "~/reference_compliant_frame_trajectory",
    rclcpp::SystemDefaultsQoS(), compliant_frame_trajectory_callback);
  s_publisher_ = get_node()->create_publisher<ControllerStateMsg>(
    "~/status", rclcpp::SystemDefaultsQoS());
  state_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<ControllerStateMsg>>(s_publisher_);

  // Initialize state message
  state_publisher_->lock();
  // state_publisher_->msg_ = admittance_->get_controller_state();
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

bool CartesianAdmittanceController::read_state_from_hardware(
  trajectory_msgs::msg::JointTrajectoryPoint & state_current,
  geometry_msgs::msg::Wrench & ft_values)
{
  // if any interface has nan values, assume state_current is the last command state
  bool all_ok = true;
  bool nan_position = false;
  bool nan_velocity = false;
  bool nan_acceleration = false;

  size_t pos_ind = 0;  // Mandatory state interface
  size_t vel_ind = pos_ind + has_velocity_state_interface_;
  size_t acc_ind = vel_ind + has_acceleration_state_interface_;
  for (size_t joint_ind = 0; joint_ind < num_joints_; ++joint_ind) {
    if (has_position_state_interface_) {
      state_current.positions[joint_ind] =
        state_interfaces_[pos_ind * num_joints_ + joint_ind].get_value();
      nan_position |= std::isnan(state_current.positions[joint_ind]);
    }
    if (has_velocity_state_interface_) {
      state_current.velocities[joint_ind] =
        state_interfaces_[vel_ind * num_joints_ + joint_ind].get_value();
      nan_velocity |= std::isnan(state_current.velocities[joint_ind]);
    }
    if (has_acceleration_state_interface_) {
      state_current.accelerations[joint_ind] =
        state_interfaces_[acc_ind * num_joints_ + joint_ind].get_value();
      nan_acceleration |= std::isnan(state_current.accelerations[joint_ind]);
    }
  }

  auto clock = get_node()->get_clock();
  if (nan_position) {
    all_ok &= false;
    state_current.positions = last_commanded_joint_state_.positions;
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *clock, 1000, "State position is NaN!");
  }
  if (nan_velocity) {
    all_ok &= false;
    state_current.velocities = last_commanded_joint_state_.velocities;
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *clock, 1000, "State velocity is NaN!");
  }
  if (nan_acceleration) {
    all_ok &= false;
    state_current.accelerations = last_commanded_joint_state_.accelerations;
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *clock, 1000, "State acceleration is NaN!");
  }

  // if any ft_values are nan, assume values are zero
  force_torque_sensor_->get_values_as_message(ft_values);
  if (
    std::isnan(ft_values.force.x) || std::isnan(ft_values.force.y) ||
    std::isnan(ft_values.force.z) || std::isnan(ft_values.torque.x) ||
    std::isnan(ft_values.torque.y) || std::isnan(ft_values.torque.z))
  {
    all_ok &= false;
    ft_values = geometry_msgs::msg::Wrench();
  }
  return all_ok;
}

bool CartesianAdmittanceController::write_state_to_hardware(
  trajectory_msgs::msg::JointTrajectoryPoint & joint_state_command)
{
  // if any interface has nan values, assume state_commanded is the last command state
  size_t pos_ind = 0;
  size_t vel_ind = pos_ind + (has_velocity_command_interface_ && has_position_command_interface_);
  size_t acc_ind = vel_ind + has_acceleration_state_interface_;
  // You never know...
  bool has_nan = false;
  for (size_t joint_ind = 0; joint_ind < num_joints_; ++joint_ind) {
    if (has_position_command_interface_) {
      if (std::isnan(joint_state_command.positions[joint_ind])) {
        has_nan |= true;
      }
    } else if (has_velocity_command_interface_) {
      if (std::isnan(joint_state_command.velocities[joint_ind])) {
        has_nan |= true;
      }
    } else if (has_acceleration_command_interface_) {
      if (std::isnan(joint_state_command.accelerations[joint_ind])) {
        has_nan |= true;
      }
    }
  }
  if (has_nan) {
    auto clock = get_node()->get_clock();
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *clock, 1000, "Joint command has NaN value(s)!");
    return false;
  }

  for (size_t joint_ind = 0; joint_ind < num_joints_; ++joint_ind) {
    if (has_position_command_interface_) {
      command_interfaces_[pos_ind * num_joints_ + joint_ind].set_value(
        joint_state_command.positions[joint_ind]);
    }
    if (has_velocity_command_interface_) {
      command_interfaces_[vel_ind * num_joints_ + joint_ind].set_value(
        joint_state_command.velocities[joint_ind]);
    }
    if (has_acceleration_command_interface_) {
      command_interfaces_[acc_ind * num_joints_ + joint_ind].set_value(
        joint_state_command.accelerations[joint_ind]);
    }
  }
  last_commanded_joint_state_ = joint_state_command;
  return true;
}

bool CartesianAdmittanceController::initialize_impedance_rule(
  const trajectory_msgs::msg::JointTrajectoryPoint & joint_state)
{
  bool all_ok = true;

  // Use current joint_state as a default reference
  auto ret = admittance_->init_reference_frame_trajectory(joint_state);
  if (ret != controller_interface::return_type::OK) {
    all_ok = false;
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to initialize the reference compliance frame trajectory.\n");
    return false;
  }
  RCLCPP_INFO(
    get_node()->get_logger(),
    "Joint position initialization: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n",
    joint_state.positions[0],
    joint_state.positions[1],
    joint_state.positions[2],
    joint_state.positions[3],
    joint_state.positions[4],
    joint_state.positions[5],
    joint_state.positions[6]
  );
  joint_command_ = joint_state;
  last_commanded_joint_state_ = joint_state;
  is_impedance_initialized_ = all_ok;
  return all_ok;
}

}  // namespace cartesian_admittance_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  cartesian_admittance_controller::CartesianAdmittanceController,
  controller_interface::ControllerInterface
)
