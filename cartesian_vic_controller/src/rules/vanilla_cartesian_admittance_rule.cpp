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

#include "cartesian_vic_controller/rules/vanilla_cartesian_admittance_rule.hpp"

#include <iostream>  // for debug purposes...

#include "control_toolbox/filters.hpp"


namespace cartesian_vic_controller
{

controller_interface::return_type VanillaCartesianAdmittanceRule::init(
  const std::shared_ptr<cartesian_vic_controller::ParamListener> & parameter_handler)
{
  auto ret = CartesianVicRule::init(parameter_handler);
  control_mode_ = ControlMode::ADMITTANCE;
  return ret;
}

controller_interface::return_type VanillaCartesianAdmittanceRule::configure(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node,
  const size_t num_joints)
{
  return CartesianVicRule::configure(node, num_joints);
}

controller_interface::return_type VanillaCartesianAdmittanceRule::reset(const size_t num_joints)
{
  return CartesianVicRule::reset(num_joints);
}

bool VanillaCartesianAdmittanceRule::compute_controls(
  VicState & vic_state,
  double dt /*period in seconds*/)
{
  const CompliantFrame & reference_compliant_frame =
    vic_state.reference_compliant_frames.get_compliant_frame(0);

  // auto rot_base_control = vic_transforms_.base_control_.rotation();
  auto rot_base_admittance = vic_transforms_.base_vic_.rotation();
  // Express M, K, D matrices in base (provided in base_vic frame)

  Eigen::Matrix<double, 6, 6> K = Eigen::Matrix<double, 6, 6>::Zero();
  K.block<3, 3>(0, 0) =
    rot_base_admittance * \
    reference_compliant_frame.stiffness.block<3, 3>(0, 0) * \
    rot_base_admittance.transpose();
  K.block<3, 3>(3, 3) =
    rot_base_admittance * \
    reference_compliant_frame.stiffness.block<3, 3>(3, 3) * \
    rot_base_admittance.transpose();

  Eigen::Matrix<double, 6, 6> D = Eigen::Matrix<double, 6, 6>::Zero();
  D.block<3, 3>(0, 0) =
    rot_base_admittance * \
    reference_compliant_frame.damping.block<3, 3>(0, 0) * \
    rot_base_admittance.transpose();
  D.block<3, 3>(3, 3) =
    rot_base_admittance * \
    reference_compliant_frame.damping.block<3, 3>(3, 3) * \
    rot_base_admittance.transpose();

  Eigen::Matrix<double, 6, 6> M = Eigen::Matrix<double, 6, 6>::Zero();
  M.block<3, 3>(0, 0) =
    rot_base_admittance * \
    reference_compliant_frame.inertia.block<3, 3>(0, 0) * \
    rot_base_admittance.transpose();
  M.block<3, 3>(3, 3) =
    rot_base_admittance * \
    reference_compliant_frame.inertia.block<3, 3>(3, 3) * \
    rot_base_admittance.transpose();

  Eigen::Matrix<double, 6, 6> M_inv = M.inverse();

  // Compute pose tracking errors
  Eigen::Matrix<double, 6, 1> error_pose;
  error_pose.head(3) =
    reference_compliant_frame.pose.translation() - \
    vic_state.robot_current_pose.translation();

  auto R_angular_error = \
    reference_compliant_frame.pose.rotation() * \
    vic_state.robot_current_pose.rotation().transpose();
  auto angle_axis = Eigen::AngleAxisd(R_angular_error);

  error_pose.tail(3) = angle_axis.angle() * angle_axis.axis();

  // Compute velocity tracking errors in ft frame
  Eigen::Matrix<double, 6, 1> error_velocity =
    reference_compliant_frame.velocity - \
    vic_state.robot_current_velocity;

  // External force at interaction frame (assumed to be control frame), expressed in the base frame
  Eigen::Matrix<double, 6, 1> F_ext = vic_state.robot_current_wrench_at_ft_frame;

  // Compute admittance control law in the base frame
  // commanded_acc = p_ddot_desired + inv(M) * (K * err_p + D * err_p_dot + f_ext)
  Eigen::Matrix<double, 6, 1> commanded_cartesian_acc =
    reference_compliant_frame.acceleration + \
    M_inv * (K * error_pose + D * error_velocity + F_ext);
  // std::cerr << "commanded_cartesian_acc = " << commanded_cartesian_acc.transpose() << std::endl;

  vic_state.robot_command_twist += vic_state.last_robot_commanded_twist +
    commanded_cartesian_acc * dt;

  auto previous_jnt_cmd_velocity = vic_state.joint_command_velocity;

  bool success = dynamics_->convert_cartesian_deltas_to_joint_deltas(
    vic_state.joint_state_position,
    vic_state.robot_command_twist,
    vic_state.control_frame,
    vic_state.joint_command_velocity
  );

  // Filter joint command velocity
  double cutoff_freq_cmd = parameters_.filters.command_filter_cuttoff_freq;
  if (cutoff_freq_cmd > 0.0) {  // No smoothing otherwise
    double cmd_filter_coefficient = 1.0 - exp(-dt * 2 * 3.14 * cutoff_freq_cmd);

    for (size_t i = 0; i < static_cast<size_t>(
        vic_state.joint_command_velocity.size()); i++)
    {
      vic_state.joint_command_velocity(i) = filters::exponentialSmoothing(
        vic_state.joint_command_velocity(i),
        previous_jnt_cmd_velocity(i),
        cmd_filter_coefficient
      );
    }
  }

  // Integrate motion in joint space
  vic_state.joint_command_position += vic_state.joint_command_velocity * dt;

  // Estimate joint command acceleration
  // TODO(tpoigonec): simply set to zero or NaN ?!
  vic_state.joint_command_acceleration.setZero();
  //  (vic_state.joint_command_velocity - previous_jnt_cmd_velocity) / dt;

  /*
  // add damping if cartesian velocity falls below threshold
  for (int64_t i = 0; i < vic_state.joint_acc.size(); ++i)
  {
    vic_state.joint_command_acceleration[i] -=
      parameters_.admittance.joint_damping * vic_state.joint_state_velocity[i];
  }
  */

  return success;
}


}  // namespace cartesian_vic_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  cartesian_vic_controller::VanillaCartesianAdmittanceRule,
  cartesian_vic_controller::CartesianVicRule
)
