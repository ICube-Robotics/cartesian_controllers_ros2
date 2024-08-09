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

#include "cartesian_vic_controller/cartesian_vic_state.hpp"
#include "cartesian_vic_controller/utils.hpp"

#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_kdl/tf2_kdl.hpp"


namespace cartesian_vic_controller
{

VicState::VicState(size_t num_joints, ControlMode mode, size_t trajectory_lenght)
: input_data(num_joints, trajectory_lenght),
  command_data(num_joints)
{
  control_mode = mode;
}

bool
VicState::to_msg(cartesian_control_msgs::msg::VicControllerState & vic_state_msg)
{
  bool success = true;

  // Fill control mode
  switch (control_mode) {
    case ControlMode::INVALID:
      vic_state_msg.control_mode.data = "invalid";
      break;
    case ControlMode::ADMITTANCE:
      vic_state_msg.control_mode.data = "admittance";
      break;
    case ControlMode::IMPEDANCE:
      vic_state_msg.control_mode.data = "impedance";
      break;
    default:
      RCLCPP_ERROR(
        rclcpp::get_logger("CartesianVicRule"),
        "Unknown control mode in 'to_msg()'!");
      success = false;
  }

  // Fill desired compliance
  auto desired_frame_0 = \
    input_data.reference_compliant_frames.get_compliant_frame(0);
  vic_state_msg.desired_pose = Eigen::toMsg(desired_frame_0.pose);
  vic_state_msg.desired_velocity = Eigen::toMsg(desired_frame_0.velocity);
  vic_state_msg.desired_acceleration = AccelToMsg(desired_frame_0.acceleration);
  vic_state_msg.desired_wrench = WrenchToMsg(desired_frame_0.wrench);
  matrixEigenToMsg(desired_frame_0.inertia, vic_state_msg.desired_inertia);
  matrixEigenToMsg(desired_frame_0.stiffness, vic_state_msg.desired_stiffness);
  matrixEigenToMsg(desired_frame_0.damping, vic_state_msg.desired_damping);

  // Fill robot state
  vic_state_msg.pose = Eigen::toMsg(input_data.robot_current_pose);
  vic_state_msg.velocity = Eigen::toMsg(input_data.robot_current_velocity);
  vic_state_msg.acceleration = AccelToMsg(input_data.robot_estimated_acceleration);
  if (input_data.has_ft_sensor()) {
    vic_state_msg.has_valid_wrench = true;
    vic_state_msg.wrench = WrenchToMsg(input_data.get_ft_sensor_wrench());
  } else {
    vic_state_msg.has_valid_wrench = false;
    vic_state_msg.wrench.force.x = 0.0;
    vic_state_msg.wrench.force.y = 0.0;
    vic_state_msg.wrench.force.z = 0.0;
    vic_state_msg.wrench.torque.x = 0.0;
    vic_state_msg.wrench.torque.y = 0.0;
    vic_state_msg.wrench.torque.z = 0.0;
  }
  matrixEigenToMsg(input_data.natural_cartesian_inertia, vic_state_msg.natural_inertia);

  // Fill rendered impedance
  matrixEigenToMsg(command_data.inertia, vic_state_msg.rendered_inertia);
  matrixEigenToMsg(command_data.stiffness, vic_state_msg.rendered_stiffness);
  matrixEigenToMsg(command_data.damping, vic_state_msg.rendered_damping);

  size_t num_joints = input_data.joint_state_position.size();
  // Fill position commands
  if (command_data.has_position_command) {
    vic_state_msg.joint_command_position.resize(num_joints);
    for (size_t i = 0; i < num_joints; i++) {
      vic_state_msg.joint_command_position[i] = command_data.joint_command_position[i];
    }
  } else {
    vic_state_msg.joint_command_position.clear();
  }
  // Fill velocity commands
  if (command_data.has_velocity_command) {
    vic_state_msg.joint_command_velocity.resize(num_joints);
    for (size_t i = 0; i < num_joints; i++) {
      vic_state_msg.joint_command_velocity[i] = command_data.joint_command_velocity[i];
    }
  } else {
    vic_state_msg.joint_command_velocity.clear();
  }
  // Fill acceleration commands
  if (command_data.has_acceleration_command) {
    vic_state_msg.joint_command_acceleration.resize(num_joints);
    for (size_t i = 0; i < num_joints; i++) {
      vic_state_msg.joint_command_acceleration[i] = command_data.joint_command_acceleration[i];
    }
  } else {
    vic_state_msg.joint_command_acceleration.clear();
  }
  // Fill effort commands
  if (command_data.has_effort_command) {
    vic_state_msg.joint_command_effort.resize(num_joints);
    for (size_t i = 0; i < num_joints; i++) {
      vic_state_msg.joint_command_effort[i] = command_data.joint_command_effort[i];
    }
  } else {
    vic_state_msg.joint_command_effort.clear();
  }

  // Fill diagnostic data
  vic_state_msg.diagnostic_data.keys.clear();
  vic_state_msg.diagnostic_data.values.clear();

  for (const auto & [key, value] : diagnostic_data) {
    vic_state_msg.diagnostic_data.keys.push_back(key);
    vic_state_msg.diagnostic_data.values.push_back(value);
  }

  return success;
}

}  // namespace cartesian_vic_controller
