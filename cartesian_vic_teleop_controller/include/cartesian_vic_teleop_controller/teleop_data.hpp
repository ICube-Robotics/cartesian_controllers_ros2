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

#ifndef CARTESIAN_VIC_TELEOP_CONTROLLER__TELEOP_DATA_HPP_
#define CARTESIAN_VIC_TELEOP_CONTROLLER__TELEOP_DATA_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <string>
#include <vector>

#include "cartesian_control_msgs/msg/key_values.hpp"
#include "cartesian_control_msgs/msg/teleop_controller_state.hpp"

namespace cartesian_vic_teleop_controller
{

/// Container for input teleop data !!!IMPORTANT!!! All in the follower frame!!!
struct TeleopDataInput
{
  bool workspace_is_engaged = false;

  Eigen::Isometry3d leader_pose;
  Eigen::Matrix<double, 6, 1> leader_velocity;
  Eigen::Matrix<double, 6, 1> leader_acceleration;
  Eigen::Matrix<double, 6, 1> leader_wrench;
  Eigen::Matrix<double, 6, 6> leader_natural_inertia;

  Eigen::Matrix<double, 6, 6> leader_desired_inertia;
  Eigen::Matrix<double, 6, 6> leader_desired_stiffness;
  Eigen::Matrix<double, 6, 6> leader_desired_damping;

  Eigen::Isometry3d follower_pose;
  Eigen::Matrix<double, 6, 1> follower_velocity;
  Eigen::Matrix<double, 6, 1> follower_acceleration;
  Eigen::Matrix<double, 6, 1> follower_wrench;
  Eigen::Matrix<double, 6, 6> follower_natural_inertia;

  Eigen::Matrix<double, 6, 6> follower_desired_inertia;
  Eigen::Matrix<double, 6, 6> follower_desired_stiffness;
  Eigen::Matrix<double, 6, 6> follower_desired_damping;
};

/// Container for output teleop data !!!IMPORTANT!!! All in the follower frame!!!
struct TeleopDataOutput
{
  Eigen::Isometry3d leader_desired_pose;
  Eigen::Matrix<double, 6, 1> leader_desired_velocity;
  Eigen::Matrix<double, 6, 1> leader_desired_acceleration;
  Eigen::Matrix<double, 6, 1> leader_desired_wrench;

  Eigen::Matrix<double, 6, 6> leader_desired_inertia;
  Eigen::Matrix<double, 6, 6> leader_desired_stiffness;
  Eigen::Matrix<double, 6, 6> leader_desired_damping;

  Eigen::Isometry3d follower_desired_pose;
  Eigen::Matrix<double, 6, 1> follower_desired_velocity;
  Eigen::Matrix<double, 6, 1> follower_desired_acceleration;
  Eigen::Matrix<double, 6, 1> follower_desired_wrench;

  Eigen::Matrix<double, 6, 6> follower_desired_inertia;
  Eigen::Matrix<double, 6, 6> follower_desired_stiffness;
  Eigen::Matrix<double, 6, 6> follower_desired_damping;
};

/// Default copy from input to output
void set_default_safe_behavior(
  const TeleopDataInput & input_data, TeleopDataOutput & output_data);

bool to_msg(
  const TeleopDataInput & input_data,
  const TeleopDataOutput & output_data,
  const std::vector<std::string> & diagnostic_keys,
  const std::vector<double> & diagnostic_values,
  cartesian_control_msgs::msg::TeleopControllerState & msg);

}  // namespace cartesian_vic_teleop_controller

#endif  // CARTESIAN_VIC_TELEOP_CONTROLLER__TELEOP_DATA_HPP_
