// Copyright 2022, ICube Laboratory, University of Strasbourg
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

#include "cartesian_state_broadcaster/cartesian_state_broadcaster.hpp"

#include <memory>
#include <string>

namespace cartesian_state_broadcaster
{
CartesianStateBroadcaster::CartesianStateBroadcaster()
: controller_interface::ControllerInterface()
{
}

CallbackReturn CartesianStateBroadcaster::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>({}));
    auto_declare<std::string>("end_effector_name", "tool0");
    auto_declare<std::string>("kinematics_plugin_name", "kinematics_interface_kdl/KDLKinematics");
    auto_declare<std::string>("kinematics_plugin_package", "kinematics_interface_kdl");
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianStateBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();

  joint_positions_.resize(joint_names_.size(), std::numeric_limits<double>::quiet_NaN());
  joint_velocities_.resize(joint_names_.size(), std::numeric_limits<double>::quiet_NaN());
  cart_velocities_.resize(7, std::numeric_limits<double>::quiet_NaN());

  end_effector_name_ = get_node()->get_parameter("end_effector_name").as_string();
  auto kinematics_plugin_name = get_node()->get_parameter("kinematics_plugin_name").as_string();
  auto kinematics_plugin_package =
    get_node()->get_parameter("kinematics_plugin_package").as_string();

  // Load the differential IK plugin
  if (!kinematics_plugin_name.empty()) {
    try {
      kinematics_loader_ = std::make_shared<
        pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>(
        kinematics_plugin_package, "kinematics_interface::KinematicsInterface");
      kinematics_ = std::unique_ptr<kinematics_interface::KinematicsInterface>(
        kinematics_loader_->createUnmanagedInstance(kinematics_plugin_name));
      if (!kinematics_->initialize(
          get_node()->get_node_parameters_interface(),
          end_effector_name_))
      {
        RCLCPP_ERROR(
          rclcpp::get_logger("CartesianStateBroadcaster"),
          "Exception while loading the IK plugin");
        return CallbackReturn::ERROR;
      }
    } catch (pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR(
        rclcpp::get_logger("CartesianStateBroadcaster"),
        "Exception while loading the IK plugin '%s': '%s'",
        kinematics_plugin_name.c_str(), ex.what());
      return CallbackReturn::ERROR;
    }
  } else {
    RCLCPP_ERROR(
      rclcpp::get_logger("CartesianStateBroadcaster"),
      "A differential IK plugin name was not specified in the config file.");
    return CallbackReturn::ERROR;
  }

  try {
    // register data publisher
    state_publisher_ = get_node()->create_publisher<StateMsg>(
      "~/cartesian_state", rclcpp::SystemDefaultsQoS());
    realtime_publisher_ = std::make_unique<StatePublisher>(state_publisher_);
  } catch (const std::exception & e) {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage"
      "with message : %s \n",
      e.what());
    return CallbackReturn::ERROR;
  }

  RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
CartesianStateBroadcaster::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
CartesianStateBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joint_names_) {
    state_interfaces_config.names.push_back(joint + "/position");
    state_interfaces_config.names.push_back(joint + "/velocity");
  }
  return state_interfaces_config;
}

CallbackReturn CartesianStateBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianStateBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianStateBroadcaster::update(
  const rclcpp::Time & /* time */,
  const rclcpp::Duration & /* period */)
{
  for (auto j = 0ul; j < joint_names_.size(); j++) {
    joint_positions_[j] = state_interfaces_[2 * j].get_value();
    joint_velocities_[j] = state_interfaces_[2 * j + 1].get_value();
  }
  if (!kinematics_->calculate_link_transform(joint_positions_, end_effector_name_, cart_trafo_)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("CartesianStateBroadcaster"),
      "calculate_link_transform failed.");
    return controller_interface::return_type::ERROR;
  }

  if (!kinematics_->convert_joint_deltas_to_cartesian_deltas(
      joint_positions_, joint_velocities_,
      end_effector_name_, cart_velocities_))
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("CartesianStateBroadcaster"),
      "convert_joint_deltas_to_cartesian_deltas failed.");
    return controller_interface::return_type::ERROR;
  }

  if (realtime_publisher_ && realtime_publisher_->trylock()) {
    auto & cart_state_msg = realtime_publisher_->msg_;
    cart_state_msg.header.stamp = get_node()->now();
    cart_state_msg.link_name = end_effector_name_;

    cart_state_msg.pose.position.x = cart_trafo_(0, 3);
    cart_state_msg.pose.position.y = cart_trafo_(1, 3);
    cart_state_msg.pose.position.z = cart_trafo_(2, 3);
    cart_state_msg.pose.orientation.x = cart_quat_.x();
    cart_state_msg.pose.orientation.y = cart_quat_.y();
    cart_state_msg.pose.orientation.z = cart_quat_.z();
    cart_state_msg.pose.orientation.w = cart_quat_.w();

    cart_state_msg.twist.linear.x = cart_velocities_[0];
    cart_state_msg.twist.linear.y = cart_velocities_[1];
    cart_state_msg.twist.linear.z = cart_velocities_[2];
    cart_state_msg.twist.angular.x = cart_velocities_[3];
    cart_state_msg.twist.angular.y = cart_velocities_[4];
    cart_state_msg.twist.angular.z = cart_velocities_[5];

    realtime_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

template<typename T1, typename T2>
void CartesianStateBroadcaster::vec_to_eigen(const std::vector<T1> & data, T2 & matrix)
{
  for (auto col = 0; col < matrix.cols(); col++) {
    for (auto row = 0; row < matrix.rows(); row++) {
      matrix(row, col) = data[row + col * matrix.rows()];
    }
  }
}

template<typename T1, typename T2>
void CartesianStateBroadcaster::eigen_to_vec(const T2 & matrix, std::vector<T1> & data)
{
  for (auto col = 0; col < matrix.cols(); col++) {
    for (auto row = 0; row < matrix.rows(); row++) {
      data[row + col * matrix.rows()] = matrix(row, col);
    }
  }
}

}  // namespace cartesian_state_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  cartesian_state_broadcaster::CartesianStateBroadcaster,
  controller_interface::ControllerInterface)
