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

#include "cartesian_vic_teleop_controller/teleop_data.hpp"

#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_kdl/tf2_kdl.hpp"

#include "cartesian_vic_controller/utils.hpp"


namespace cartesian_vic_teleop_controller
{

using cartesian_vic_controller::matrixEigenToMsg;
using cartesian_vic_controller::AccelToMsg;
using cartesian_vic_controller::WrenchToMsg;


void set_default_safe_behavior(const TeleopDataInput & input_data, TeleopDataOutput & output_data)
{
  output_data.leader_desired_pose = input_data.follower_pose;
  output_data.leader_desired_velocity.setZero();
  output_data.leader_desired_acceleration.setZero();
  output_data.leader_desired_wrench.setZero();

  output_data.leader_desired_inertia = input_data.leader_desired_inertia;
  output_data.leader_desired_stiffness = input_data.leader_desired_stiffness;
  output_data.leader_desired_damping = input_data.leader_desired_damping;

  output_data.follower_desired_pose = input_data.leader_pose;
  output_data.follower_desired_velocity.setZero();
  output_data.follower_desired_acceleration.setZero();
  output_data.follower_desired_wrench.setZero();

  output_data.follower_desired_inertia = input_data.follower_desired_inertia;
  output_data.follower_desired_stiffness = input_data.follower_desired_stiffness;
  output_data.follower_desired_damping = input_data.follower_desired_damping;
}

bool to_msg(
  const TeleopDataInput & input_data,
  const TeleopDataOutput & output_data,
  const std::vector<std::string> & diagnostic_keys,
  const std::vector<double> & diagnostic_values,
  cartesian_control_msgs::msg::TeleopControllerState & msg)
{
  // Fill input data
  msg.workspace_is_engaged.data = input_data.workspace_is_engaged;

  msg.x_1 = Eigen::toMsg(input_data.leader_pose);
  msg.x_1_dot = Eigen::toMsg(input_data.leader_velocity);
  msg.x_1_ddot = AccelToMsg(input_data.leader_acceleration);
  msg.f_1 = WrenchToMsg(input_data.leader_wrench);

  msg.x_2 = Eigen::toMsg(input_data.follower_pose);
  msg.x_2_dot = Eigen::toMsg(input_data.follower_velocity);
  msg.x_2_ddot = AccelToMsg(input_data.follower_acceleration);
  msg.f_2 = WrenchToMsg(input_data.follower_wrench);

  matrixEigenToMsg(input_data.leader_desired_inertia, msg.desired_inertia_1);
  matrixEigenToMsg(input_data.leader_desired_stiffness, msg.desired_stiffness_1);
  matrixEigenToMsg(input_data.leader_desired_damping, msg.desired_damping_1);
  matrixEigenToMsg(input_data.follower_desired_inertia, msg.desired_inertia_2);
  matrixEigenToMsg(input_data.follower_desired_stiffness, msg.desired_stiffness_2);
  matrixEigenToMsg(input_data.follower_desired_damping, msg.desired_damping_2);

  // Fill output data
  msg.x_1_d = Eigen::toMsg(output_data.leader_desired_pose);
  msg.x_1_dot_d = Eigen::toMsg(output_data.leader_desired_velocity);
  msg.x_1_ddot_d = AccelToMsg(output_data.leader_desired_acceleration);
  msg.f_1_d = WrenchToMsg(output_data.leader_desired_wrench);

  msg.x_2_d = Eigen::toMsg(output_data.follower_desired_pose);
  msg.x_2_dot_d = Eigen::toMsg(output_data.follower_desired_velocity);
  msg.x_2_ddot_d = AccelToMsg(output_data.follower_desired_acceleration);
  msg.f_2_d = WrenchToMsg(output_data.follower_desired_wrench);

  matrixEigenToMsg(output_data.leader_desired_inertia, msg.rendered_inertia_1);
  matrixEigenToMsg(output_data.leader_desired_stiffness, msg.rendered_stiffness_1);
  matrixEigenToMsg(output_data.leader_desired_damping, msg.rendered_damping_1);
  matrixEigenToMsg(output_data.follower_desired_inertia, msg.rendered_inertia_2);
  matrixEigenToMsg(output_data.follower_desired_stiffness, msg.rendered_stiffness_2);
  matrixEigenToMsg(output_data.follower_desired_damping, msg.rendered_damping_2);

  // Fill diagnostic data
  if (diagnostic_keys.size() != diagnostic_values.size()) {
    std::cerr << "Diagnostic keys and values have different sizes!" << std::endl;
    return false;
  }
  msg.diagnostic_data.keys.clear();
  msg.diagnostic_data.values.clear();
  for (size_t idx = 0; idx < diagnostic_keys.size(); ++idx) {
    msg.diagnostic_data.keys.push_back(diagnostic_keys[idx]);
    msg.diagnostic_data.values.push_back(diagnostic_values[idx]);
  }

  return true;
}

}  // namespace cartesian_vic_teleop_controller
