// Copyright 2024 ICUBE Laboratory, University of Strasbourg
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
/// \authors: Thibault Poignonec <thibault.poignonec@gmail.com>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cmath>

#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/time.hpp"

#include "cartesian_vic_teleop_controller/rules/vanilla_teleop_rule.hpp"

namespace cartesian_vic_teleop_controller
{

VanillaTeleopRule::VanillaTeleopRule()
: TeleopRule()
{
  logger_ = rclcpp::get_logger("VanillaTeleopRule");
  // Nothing else to do, see init().
}

bool VanillaTeleopRule::init(
  std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface>/* parameters_interface */,
  const std::string & /* param_namespace */)
{
  is_initialized_ = true;
  return true;
}

bool VanillaTeleopRule::update(
  const rclcpp::Time & /* time */,
  const rclcpp::Duration & period,
  const TeleopDataInput & teleop_data_input,
  TeleopDataOutput & teleop_data_output)
{
  // Clear the diagnostic_data field of the TeleopControllerState msg
  clear_diagnostic_data();

  // Nominal behavior:
  // Copy the leader and follower pose, velocity, acceleration, wrench
  teleop_data_output.desired_leader_pose = teleop_data_input.follower_pose;
  teleop_data_output.desired_leader_velocity = teleop_data_input.follower_velocity;
  teleop_data_output.desired_leader_acceleration = teleop_data_input.follower_acceleration;
  teleop_data_output.desired_leader_wrench = teleop_data_input.follower_wrench;

  teleop_data_output.desired_leader_inertia = teleop_data_input.desired_leader_inertia;
  teleop_data_output.desired_leader_stiffness = teleop_data_input.desired_leader_stiffness;
  teleop_data_output.desired_leader_damping = teleop_data_input.desired_leader_damping;

  teleop_data_output.desired_follower_pose = teleop_data_input.leader_pose;
  teleop_data_output.desired_follower_velocity = teleop_data_input.leader_velocity;
  teleop_data_output.desired_follower_acceleration = teleop_data_input.leader_acceleration;
  teleop_data_output.desired_follower_wrench = teleop_data_input.leader_wrench;

  teleop_data_output.desired_follower_inertia = teleop_data_input.desired_follower_inertia;
  teleop_data_output.desired_follower_stiffness = teleop_data_input.desired_follower_stiffness;
  teleop_data_output.desired_follower_damping = teleop_data_input.desired_follower_damping;

  teleop_data_output.desired_follower_velocity.tail(3).setZero();
  teleop_data_output.desired_follower_acceleration.tail(3).setZero();

  // Logging
  double real_Ts = period.seconds();
  add_diagnostic_value("real_Ts", real_Ts);

  // Set the desired velocity to zero if the workspace is disengaged
  if (!teleop_data_input.workspace_is_engaged) {
    // At minima...
    teleop_data_output.desired_leader_velocity.setZero();
    teleop_data_output.desired_leader_acceleration.setZero();
    teleop_data_output.desired_leader_wrench.setZero();
    teleop_data_output.desired_follower_velocity.setZero();
    teleop_data_output.desired_follower_acceleration.setZero();
    teleop_data_output.desired_follower_wrench.setZero();
    // TODO(anyone): Is this right? Doesn't seem so...

    // To increase comfort, set K = 0 and D ~ 0
    teleop_data_output.desired_leader_stiffness.setZero();
    teleop_data_output.desired_leader_damping = 1.0 * Eigen::Matrix<double, 6, 6>::Identity();
  }
  return true;
}

}  // namespace cartesian_vic_teleop_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  cartesian_vic_teleop_controller::VanillaTeleopRule,
  cartesian_vic_teleop_controller::TeleopRule
)
