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

#include "cartesian_admittance_controller/rules/vanilla_cartesian_admittance_rule.hpp"

#include <iostream>  // for debug purposes...

namespace cartesian_admittance_controller
{

controller_interface::return_type VanillaCartesianAdmittanceRule::init(
  const std::shared_ptr<cartesian_admittance_controller::ParamListener> & parameter_handler)
{
  return CartesianAdmittanceRule::init(parameter_handler);
}

controller_interface::return_type VanillaCartesianAdmittanceRule::configure(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node,
  const size_t num_joints)
{
  return CartesianAdmittanceRule::configure(node, num_joints);
}

controller_interface::return_type VanillaCartesianAdmittanceRule::reset(const size_t num_joints)
{
  return CartesianAdmittanceRule::reset(num_joints);
}

bool VanillaCartesianAdmittanceRule::compute_controls(
  AdmittanceState & admittance_state,
  double dt /*period in seconds*/)
{
  const CompliantFrame & reference_compliant_frame =
    admittance_state.reference_compliant_frames.get_compliant_frame(0);

  auto rot_base_control = admittance_transforms_.base_control_.rotation();
  auto rot_base_admittance = admittance_transforms_.base_admittance_.rotation();
  // Express M, K, D matrices in base (provided as diagonal terms in control frame)

  Eigen::Matrix<double, 6, 6> K = Eigen::Matrix<double, 6, 6>::Zero();
  K.block<3, 3>(0, 0) =
    rot_base_admittance * \
    reference_compliant_frame.stiffness.head(3).asDiagonal() * \
    rot_base_admittance.transpose();
  K.block<3, 3>(3, 3) =
    rot_base_admittance * \
    reference_compliant_frame.stiffness.tail(3).asDiagonal() * \
    rot_base_admittance.transpose();

  Eigen::Matrix<double, 6, 6> D = Eigen::Matrix<double, 6, 6>::Zero();
  D.block<3, 3>(0, 0) =
    rot_base_admittance * \
    reference_compliant_frame.damping.head(3).asDiagonal() * \
    rot_base_admittance.transpose();
  D.block<3, 3>(3, 3) =
    rot_base_admittance * \
    reference_compliant_frame.damping.tail(3).asDiagonal() * \
    rot_base_admittance.transpose();

  Eigen::Matrix<double, 6, 6> M_inv = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 1> inertia_inv = reference_compliant_frame.inertia.cwiseInverse();
  M_inv.block<3, 3>(0, 0) = rot_base_admittance *
    inertia_inv.head(3).asDiagonal() * rot_base_admittance.transpose();
  M_inv.block<3, 3>(3, 3) = rot_base_admittance *
    inertia_inv.tail(3).asDiagonal() * rot_base_admittance.transpose();

  // Compute pose tracking errors
  Eigen::Matrix<double, 6, 1> error_pose;
  error_pose.head(3) =
    reference_compliant_frame.pose.translation() - \
    admittance_state.robot_current_pose.translation();

  auto R_angular_error = \
    reference_compliant_frame.pose.rotation() \
    * admittance_state.robot_current_pose.rotation().transpose();
  auto angle_axis = Eigen::AngleAxisd(R_angular_error);

  error_pose.tail(3) = angle_axis.angle() * angle_axis.axis();

  // Compute velocity tracking errors in ft frame
  Eigen::Matrix<double, 6, 1> error_velocity =
    reference_compliant_frame.velocity - \
    admittance_state.robot_current_velocity;

  // External force at interaction frame (assumed to be control frame), expressed in the base frame
  Eigen::Matrix<double, 6, 1> F_ext = admittance_state.robot_current_wrench_at_ft_frame;

  // Compute admittance control law in the base frame
  // commanded_acc = p_ddot_desired + inv(M) * (K * err_p + D * err_p_dot + f_ext)
  Eigen::Matrix<double, 6, 1> commanded_cartesian_acc =
    reference_compliant_frame.acceleration + \
    M_inv * (K * error_pose + D * error_velocity + F_ext);

  admittance_state.robot_command_twist += admittance_state.last_robot_commanded_twist + commanded_cartesian_acc * dt;

  auto previous_jnt_cmd_velocity = admittance_state.joint_command_velocity;

  bool success = kinematics_->convert_cartesian_deltas_to_joint_deltas(
    admittance_state.joint_state_position,
    admittance_state.robot_command_twist,
    admittance_state.control_frame,
    admittance_state.joint_command_velocity
  );

  // Integrate motion in joint space
  admittance_state.joint_command_position += admittance_state.joint_command_velocity * dt;

  /*
  std::cerr << "error_pose = " << error_pose.transpose() << std::endl;
  std::cerr << "error_vel = " << error_velocity.transpose() << std::endl;
  std::cerr << "F_ext = " << F_ext.transpose() << std::endl;

  std::cerr << "commanded acc = " << commanded_cartesian_acc.transpose() << std::endl;
  std::cerr << "commanded twist = " << admittance_state.robot_command_twist.transpose() << std::endl;

  std::cerr << "commanded joint vel = " << admittance_state.joint_command_velocity.transpose() << std::endl;
  std::cerr << "commanded joint pos = " << admittance_state.joint_command_position.transpose() << std::endl;
  */

  // Estimate joint command acceleration
  // TODO(tpoigonec): simply set to zero or NaN ?!
  admittance_state.joint_command_acceleration.setZero();
  //  (admittance_state.joint_command_velocity - previous_jnt_cmd_velocity) / dt;

  /*
  // add damping if cartesian velocity falls below threshold
  for (int64_t i = 0; i < admittance_state.joint_acc.size(); ++i)
  {
    admittance_state.joint_command_acceleration[i] -=
      parameters_.admittance.joint_damping * admittance_state.joint_state_velocity[i];
  }
  */

  return success;
}


} // namespace cartesian_admittance_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  cartesian_admittance_controller::VanillaCartesianAdmittanceRule,
  cartesian_admittance_controller::CartesianAdmittanceRule
)
