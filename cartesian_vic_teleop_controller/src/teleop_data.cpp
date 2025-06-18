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
  output_data.desired_leader_pose = input_data.follower_pose;
  output_data.desired_leader_velocity.setZero();
  output_data.desired_leader_acceleration.setZero();
  output_data.desired_leader_wrench.setZero();

  output_data.desired_leader_inertia = input_data.desired_leader_inertia;
  output_data.desired_leader_stiffness = input_data.desired_leader_stiffness;
  output_data.desired_leader_damping = input_data.desired_leader_damping;

  output_data.desired_follower_pose = input_data.leader_pose;
  output_data.desired_follower_velocity.setZero();
  output_data.desired_follower_acceleration.setZero();
  output_data.desired_follower_wrench.setZero();

  output_data.desired_follower_inertia = input_data.desired_follower_inertia;
  output_data.desired_follower_stiffness = input_data.desired_follower_stiffness;
  output_data.desired_follower_damping = input_data.desired_follower_damping;
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

  msg.leader_pose = Eigen::toMsg(input_data.leader_pose);
  msg.leader_velocity = Eigen::toMsg(input_data.leader_velocity);
  msg.leader_acceleration = AccelToMsg(input_data.leader_acceleration);
  msg.leader_wrench = WrenchToMsg(input_data.leader_wrench);

  msg.follower_pose = Eigen::toMsg(input_data.follower_pose);
  msg.follower_velocity = Eigen::toMsg(input_data.follower_velocity);
  msg.follower_acceleration = AccelToMsg(input_data.follower_acceleration);
  msg.follower_wrench = WrenchToMsg(input_data.follower_wrench);

  matrixEigenToMsg(input_data.desired_leader_inertia, msg.desired_leader_inertia);
  matrixEigenToMsg(input_data.desired_leader_stiffness, msg.desired_leader_stiffness);
  matrixEigenToMsg(input_data.desired_leader_damping, msg.desired_leader_damping);
  matrixEigenToMsg(input_data.desired_follower_inertia, msg.desired_follower_inertia);
  matrixEigenToMsg(input_data.desired_follower_stiffness, msg.desired_follower_stiffness);
  matrixEigenToMsg(input_data.desired_follower_damping, msg.desired_follower_damping);

  // Fill output data
  msg.desired_leader_pose = Eigen::toMsg(output_data.desired_leader_pose);
  msg.desired_leader_velocity = Eigen::toMsg(output_data.desired_leader_velocity);
  msg.desired_leader_acceleration = AccelToMsg(output_data.desired_leader_acceleration);
  msg.desired_leader_wrench = WrenchToMsg(output_data.desired_leader_wrench);

  msg.desired_follower_pose = Eigen::toMsg(output_data.desired_follower_pose);
  msg.desired_follower_velocity = Eigen::toMsg(output_data.desired_follower_velocity);
  msg.desired_follower_acceleration = AccelToMsg(output_data.desired_follower_acceleration);
  msg.desired_follower_wrench = WrenchToMsg(output_data.desired_follower_wrench);

  matrixEigenToMsg(output_data.desired_leader_inertia, msg.rendered_leader_inertia);
  matrixEigenToMsg(output_data.desired_leader_stiffness, msg.rendered_leader_stiffness);
  matrixEigenToMsg(output_data.desired_leader_damping, msg.rendered_leader_damping);
  matrixEigenToMsg(output_data.desired_follower_inertia, msg.rendered_follower_inertia);
  matrixEigenToMsg(output_data.desired_follower_stiffness, msg.rendered_follower_stiffness);
  matrixEigenToMsg(output_data.desired_follower_damping, msg.rendered_follower_damping);

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
