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

#ifndef CARTESIAN_VIC_CONTROLLER__CARTESIAN_VIC_STATE_HPP_
#define CARTESIAN_VIC_CONTROLLER__CARTESIAN_VIC_STATE_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <map>
#include <memory>
#include <string>
#include <vector>

// include data structures
#include "cartesian_vic_controller/compliance_frame_trajectory.hpp"

namespace cartesian_vic_controller
{

enum class ControlMode
{
  INVALID = 0,
  ADMITTANCE = 1,
  IMPEDANCE = 2
};

// VicState is the internal state of the VIC controller
struct VicState
{
  explicit VicState(size_t num_joints, ControlMode mode, size_t trajectory_lenght = 1)
  : reference_compliant_frames(trajectory_lenght)
  {
    control_mode = mode;

    // Allocate joint state
    joint_state_position = Eigen::VectorXd::Zero(num_joints);
    joint_state_velocity = Eigen::VectorXd::Zero(num_joints);

    // Allocate and reset command
    robot_command_twist.setZero();
    joint_command_position = Eigen::VectorXd::Zero(num_joints);
    joint_command_velocity = Eigen::VectorXd::Zero(num_joints);
    joint_command_acceleration = Eigen::VectorXd::Zero(num_joints);
    joint_command_effort = Eigen::VectorXd::Zero(num_joints);

    // Allocate and reset history
    last_robot_commanded_twist.setZero();
  }

  ControlMode control_mode;

  // General parameters
  //------------------------
  /// Name of the robot base frame
  std::string base_frame;
  /// Name of the compliance (i.e., vic) frame in which compliance parameters are specified
  std::string vic_frame;
  /// Name of the control frame in which is expressed the cartesian pose/vel/acc/wrench reference
  std::string control_frame;
  /// Name of the force/torque sensor frame in which is expressed the measured wrench
  std::string ft_sensor_frame;

  // Desired compliant frame(s)
  //----------------------------
  CompliantFrameTrajectory reference_compliant_frames;

  // Rendered compliance
  //----------------------------
  /// Rendered inertia matrix
  Eigen::Matrix<double, 6, 6> inertia;
  /// Rendered stiffness matrix
  Eigen::Matrix<double, 6, 6> stiffness;
  /// Rendered damping matrix
  Eigen::Matrix<double, 6, 6> damping;

  // Measured robot state
  //-----------------------
  Eigen::VectorXd joint_state_position;
  Eigen::VectorXd joint_state_velocity;
  // Cartesian state (control frame w.r.t. robot base frame)
  Eigen::Isometry3d robot_current_pose;
  Eigen::Matrix<double, 6, 1> robot_current_velocity;
  Eigen::Matrix<double, 6, 1> robot_current_wrench_at_ft_frame;  // ft_frame w.r.t. base

  // Computed command
  //------------------------
  // Commanded twist (control frame w.r.t. robot base frame)
  Eigen::Matrix<double, 6, 1> robot_command_twist;
  // Last commanded twist
  Eigen::Matrix<double, 6, 1> last_robot_commanded_twist;

  // Commanded joint state computed from "robot_command_twist"
  Eigen::VectorXd joint_command_position;
  Eigen::VectorXd joint_command_velocity;
  Eigen::VectorXd joint_command_acceleration;
  Eigen::VectorXd joint_command_effort;

  // Additional data
  //------------------------
  /// Natural inertia matrix (from dynamic model)
  Eigen::Matrix<double, 6, 6> natural_inertia;

  // Diagnostics
  std::map<std::string, double> diagnostic_data;
};

/// Transforms between frames used in the Vic controller
struct VicTransforms
{
  // transformation from force torque sensor frame to base link frame at reference joint angles
  Eigen::Isometry3d ref_base_ft_;
  // transformation from force torque sensor frame to base link frame at reference + vic
  // offset joint angles
  Eigen::Isometry3d base_ft_;
  // transformation from control frame to base link frame at reference + vic offset joint
  // angles
  Eigen::Isometry3d base_control_;
  Eigen::Isometry3d base_vic_;
  // transformation from end effector frame to base link frame at reference + vic offset
  // joint angles
  Eigen::Isometry3d base_tip_;
  // transformation from center of gravity frame to base link frame at reference + vic offset
  // joint angles
  Eigen::Isometry3d base_cog_;
  // transformation from world frame to base link frame
  Eigen::Isometry3d world_base_;
};


}  // namespace cartesian_vic_controller

#endif  // CARTESIAN_VIC_CONTROLLER__CARTESIAN_VIC_STATE_HPP_
