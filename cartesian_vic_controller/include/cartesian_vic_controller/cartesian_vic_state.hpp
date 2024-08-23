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

// custom msgs
#include "cartesian_control_msgs/msg/vic_controller_state.hpp"

namespace cartesian_vic_controller
{

enum class ControlMode
{
  INVALID = 0,
  ADMITTANCE = 1,
  IMPEDANCE = 2
};

class VicInputData
{
public:
  explicit VicInputData(size_t num_joints, size_t trajectory_lenght = 1)
  : reference_compliant_frames(trajectory_lenght)
  {
    // Allocate joint state
    joint_state_position = Eigen::VectorXd::Zero(num_joints);
    joint_state_velocity = Eigen::VectorXd::Zero(num_joints);
    joint_state_external_torques_ = Eigen::VectorXd::Zero(num_joints);

    natural_joint_space_inertia = \
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(num_joints, num_joints);

    // Allocate nullspace parameters
    nullspace_desired_joint_positions = Eigen::VectorXd::Zero(num_joints);
    nullspace_joint_inertia = Eigen::VectorXd::Zero(num_joints);
    nullspace_joint_stiffness = Eigen::VectorXd::Zero(num_joints);
    nullspace_joint_damping = Eigen::VectorXd::Zero(num_joints);

    // Reset cartesian state
    robot_current_velocity.setZero();
    robot_estimated_acceleration.setZero();
    robot_current_wrench_at_ft_frame_.setZero();
    natural_joint_space_inertia = \
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(num_joints, num_joints);

    // Reset data availability
    has_ft_sensor_ = false;
    has_external_torque_sensor_ = false;
  }
  bool set_joint_state_external_torques(const Eigen::VectorXd & joint_state_external_torques)
  {
    if (joint_state_external_torques.size() != joint_state_external_torques_.size()) {
      has_external_torque_sensor_ = false;
      return false;
    }
    joint_state_external_torques_ = joint_state_external_torques;
    has_external_torque_sensor_ = true;
    return true;
  }

  bool set_ft_sensor_wrench(const Eigen::Matrix<double, 6, 1> & ft_sensor_wrench)
  {
    robot_current_wrench_at_ft_frame_ = ft_sensor_wrench;
    has_ft_sensor_ = true;
    return true;
  }

  const Eigen::VectorXd & get_joint_state_external_torques() const
  {
    return joint_state_external_torques_;
  }

  const Eigen::Matrix<double, 6, 1> & get_ft_sensor_wrench() const
  {
    return robot_current_wrench_at_ft_frame_;
  }

  bool reset_joint_state_external_torques()
  {
    has_external_torque_sensor_ = false;
    return true;
  }

  bool reset_ft_sensor_wrench()
  {
    has_ft_sensor_ = false;
    return true;
  }
  bool has_external_torque_sensor() const
  {
    return has_external_torque_sensor_;
  }
  bool has_ft_sensor() const
  {
    return has_ft_sensor_;
  }

public:
  // General parameters
  //------------------------
  // Flags
  bool activate_nullspace_control = false;
  bool activate_gravity_compensation = false;

  /// Name of the control frame in which is expressed the cartesian pose/vel/acc/wrench reference
  std::string base_frame;
  /// Name of the compliance (i.e., vic) frame in which compliance parameters are specified
  std::string vic_frame;
  /// Name of the controlled robot end-effector frame
  std::string end_effector_frame;
  /// Name of the force/torque sensor frame in which is expressed the measured wrench
  std::string ft_sensor_frame;

  // Desired compliant frame(s)
  //----------------------------
  CompliantFrameTrajectory reference_compliant_frames;

  // Nullspace control parameters
  //------------------------------
  Eigen::VectorXd nullspace_desired_joint_positions;
  Eigen::VectorXd nullspace_joint_inertia;
  Eigen::VectorXd nullspace_joint_stiffness;
  Eigen::VectorXd nullspace_joint_damping;

  // Measured robot state
  //-----------------------
  Eigen::VectorXd joint_state_position;
  Eigen::VectorXd joint_state_velocity;

  // Cartesian state (control frame w.r.t. robot base frame)
  Eigen::Isometry3d robot_current_pose;
  Eigen::Matrix<double, 6, 1> robot_current_velocity;
  Eigen::Matrix<double, 6, 1> robot_estimated_acceleration;

  /// Natural inertia matrix (from dynamic model)
  Eigen::Matrix<double, 6, 6> natural_cartesian_inertia;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> natural_joint_space_inertia;

protected:
  Eigen::VectorXd joint_state_external_torques_;
  Eigen::Matrix<double, 6, 1> robot_current_wrench_at_ft_frame_;  // ft_frame w.r.t. base
  bool has_ft_sensor_ = false;
  bool has_external_torque_sensor_ = false;
};

class VicCommandData
{
public:
  explicit VicCommandData(size_t num_joints)
  {
    // Allocate and reset command
    joint_command_position = Eigen::VectorXd::Zero(num_joints);
    joint_command_velocity = Eigen::VectorXd::Zero(num_joints);
    joint_command_acceleration = Eigen::VectorXd::Zero(num_joints);
    joint_command_effort = Eigen::VectorXd::Zero(num_joints);
  }

public:
  // Rendered compliance
  //----------------------------
  /// Rendered inertia matrix
  Eigen::Matrix<double, 6, 6> inertia;
  /// Rendered stiffness matrix
  Eigen::Matrix<double, 6, 6> stiffness;
  /// Rendered damping matrix
  Eigen::Matrix<double, 6, 6> damping;

  // Computed command
  //------------------------
  // Commanded joint state computed from "robot_command_twist"
  Eigen::VectorXd joint_command_position;
  Eigen::VectorXd joint_command_velocity;
  Eigen::VectorXd joint_command_acceleration;
  Eigen::VectorXd joint_command_effort;

  // Command availability
  //------------------------
  bool has_position_command = false;
  bool has_velocity_command = false;
  bool has_acceleration_command = false;
  bool has_effort_command = false;
};

// VicState is the internal state of the VIC controller
class VicState
{
public:
  explicit VicState(
    size_t num_joints,
    ControlMode mode,
    size_t trajectory_lenght = 1);

  ~VicState() = default;

  bool to_msg(cartesian_control_msgs::msg::VicControllerState & vic_state_msg);

public:
  /// VIC control mode: INVALID / ADMITTANCE / IMPEDANCE
  ControlMode control_mode;

  /// Input data used for VIC: reference compliant frame(s) and robot measurements
  VicInputData input_data;

  /// Output data computed by the VIC controller: robot joint commands
  VicCommandData command_data;

  // Diagnostics data
  std::map<std::string, double> diagnostic_data;
};

/// Transforms between frames used in the Vic controller
struct VicTransforms
{
  // transformation from fixed world frame to base link frame
  Eigen::Isometry3d world_base_;
  // transformation from force torque sensor frame to base link frame
  Eigen::Isometry3d base_ft_;
  // transformation from controlled end effector (EE) frame to base link frame
  Eigen::Isometry3d base_end_effector_;
  // transformation from vic frame to base link frame
  Eigen::Isometry3d base_vic_;
  // transformation from CoG frame (i.e., post-sensor inertial frame) to base link frame
  Eigen::Isometry3d base_cog_;
};


}  // namespace cartesian_vic_controller

#endif  // CARTESIAN_VIC_CONTROLLER__CARTESIAN_VIC_STATE_HPP_
