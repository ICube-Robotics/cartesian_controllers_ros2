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

#ifndef CARTESIAN_ADMITTANCE_CONTROLLER__CARTESIAN_ADMITTANCE_RULE_HPP_
#define CARTESIAN_ADMITTANCE_CONTROLLER__CARTESIAN_ADMITTANCE_RULE_HPP_

#include <memory>
#include <vector>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "controller_interface/controller_interface.hpp"
#include "kinematics_interface/kinematics_interface.hpp"
#include "pluginlib/class_loader.hpp"

// msgs
#include "control_msgs/msg/admittance_controller_state.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// Custom msgs
#include "cartesian_control_msgs/msg/cartesian_trajectory.hpp"
#include "cartesian_control_msgs/msg/compliant_frame_trajectory.hpp"

// include generated parameter library
#include "cartesian_admittance_controller_parameters.hpp"

// include data structures
#include "cartesian_admittance_controller/compliance_frame_trajectory.hpp"

namespace cartesian_admittance_controller
{

struct AdmittanceState
{
  explicit AdmittanceState(size_t num_joints, size_t trajectory_lenght = 1)
  : reference_compliant_frames(trajectory_lenght)
  {
    // Allocate joint state
    joint_state_position = Eigen::VectorXd::Zero(num_joints);
    joint_state_velocity = Eigen::VectorXd::Zero(num_joints);

    // Allocate and reset command
    robot_command_twist.setZero();
    joint_command_position = Eigen::VectorXd::Zero(num_joints);
    joint_command_velocity = Eigen::VectorXd::Zero(num_joints);
    joint_command_acceleration = Eigen::VectorXd::Zero(num_joints);
  }

  // General parameters
  //------------------------
  /// Name of the robot base frame
  std::string base_frame;
  /// Name of the compliance (i.e., admittance) frame in which compliance parameters are specified
  std::string admittance_frame;
  /// Name of the control frame in wich is expressed the cartesian pose/vel/acc/wrench reference
  std::string control_frame;
  /// Name of the force/torque sensor frame in wich is expressed the measured wrench
  std::string ft_sensor_frame;

  // Desired compliant frame(s)
  //----------------------------
  CompliantFrameTrajectory reference_compliant_frames;

  // Measured robot state
  //-----------------------
  Eigen::VectorXd joint_state_position;
  Eigen::VectorXd joint_state_velocity;
  // Cartesian state (control frame w.r.t. robot base frame)
  Eigen::Isometry3d robot_current_pose;
  Eigen::Matrix<double, 6, 1> robot_current_velocity;
  Eigen::Matrix<double, 6, 1> robot_current_wrench_at_ft_frame; // ft_frame w.r.t. base

  // Computed command
  //------------------------
  // Commanded twist (control frame w.r.t. robot base frame)
  Eigen::Matrix<double, 6, 1> robot_command_twist;
  // Commanded joint state computed from "robot_command_twist"
  Eigen::VectorXd joint_command_position;
  Eigen::VectorXd joint_command_velocity;
  Eigen::VectorXd joint_command_acceleration;
};

struct AdmittanceTransforms
{
  // transformation from force torque sensor frame to base link frame at reference joint angles
  Eigen::Isometry3d ref_base_ft_;
  // transformation from force torque sensor frame to base link frame at reference + admittance
  // offset joint angles
  Eigen::Isometry3d base_ft_;
  // transformation from control frame to base link frame at reference + admittance offset joint
  // angles
  Eigen::Isometry3d base_control_;
  Eigen::Isometry3d base_admittance_;
  // transformation from end effector frame to base link frame at reference + admittance offset
  // joint angles
  Eigen::Isometry3d base_tip_;
  // transformation from center of gravity frame to base link frame at reference + admittance offset
  // joint angles
  Eigen::Isometry3d base_cog_;
  // transformation from world frame to base link frame
  Eigen::Isometry3d world_base_;
};

class CartesianAdmittanceRule
{
public:
  CartesianAdmittanceRule() = default;
  virtual ~CartesianAdmittanceRule() = default;

  /// Initialize admittance rule
  virtual controller_interface::return_type init(
    const std::shared_ptr<cartesian_admittance_controller::ParamListener> & parameter_handler);

  /// Configure admittance rule
  virtual controller_interface::return_type configure(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node,
    const size_t num_joint);

  /// Soft reset admittance rule (set cartesian ref as current pose)
  controller_interface::return_type init_reference_frame_trajectory(
    const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state);

  /// Reset all values back to default
  virtual controller_interface::return_type reset(const size_t num_joints);

  /// Retrieve parameters and update if applicable
  void apply_parameters_update();

  controller_interface::return_type update_compliant_frame_trajectory(
    const cartesian_control_msgs::msg::CompliantFrameTrajectory & compliant_frame_trajectory);

  /**
  * Compute joint (velocity) command from the current cartesian tracking errors
  * and the desired interaction parameters (M, K, D).
  *
  * \param[in] current_joint_state current joint state of the robot
  * \param[in] measured_wrench most recent measured wrench from force torque sensor
  * \param[in] period time in seconds since last controller update
  * \param[out] joint_state_command computed joint state command
  */
  controller_interface::return_type update(
    const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state,
    const geometry_msgs::msg::Wrench & measured_wrench,
    const rclcpp::Duration & period,
    trajectory_msgs::msg::JointTrajectoryPoint & joint_state_command
  );

protected:
  /// Manual setting of inertia, damping, and stiffness (diagonal matrices)
  void set_interaction_parameters(
    const Eigen::Matrix<double, 6, 1> & desired_inertia,
    const Eigen::Matrix<double, 6, 1> & desired_stiffness,
    const Eigen::Matrix<double, 6, 1> & desired_damping
  );

  bool update_internal_state(
    const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state,
    const geometry_msgs::msg::Wrench & measured_wrench);

  bool process_wrench_measurements(
    const geometry_msgs::msg::Wrench & measured_wrench);

  /// Actual admittance control logic
  virtual bool compute_controls(
    AdmittanceState & admittance_state,
    double dt /*period in seconds*/) = 0;

public:
  // Parameters management
  std::shared_ptr<cartesian_admittance_controller::ParamListener> parameter_handler_;
  cartesian_admittance_controller::Params parameters_;

  // Admittance controllers internal state
  AdmittanceState admittance_state_{0};

protected:
  template<typename T1, typename T2>
  void vec_to_eigen(const std::vector<T1> & data, T2 & matrix);

  /// Number of robot joints
  size_t num_joints_;

  /// If true, the parameters values are ignored (use "set_interaction_parameters()" instead)
  bool use_streamed_interaction_parameters_ = false;

  // Kinematics interface plugin loader
  std::shared_ptr<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>
  kinematics_loader_;

  /// Kinematics interface
  std::unique_ptr<kinematics_interface::KinematicsInterface> kinematics_;

  // transforms needed for admittance update
  AdmittanceTransforms admittance_transforms_;

  /// Filtered wrench expressed in world frame
  Eigen::Matrix<double, 6, 1> wrench_world_;

};

}  // namespace cartesian_admittance_controller

#endif  // CARTESIAN_ADMITTANCE_CONTROLLER__CARTESIAN_ADMITTANCE_RULE_HPP_
