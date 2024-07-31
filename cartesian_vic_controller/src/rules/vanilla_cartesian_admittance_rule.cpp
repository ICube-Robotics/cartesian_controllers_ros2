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

#include "cartesian_vic_controller/rules/vanilla_cartesian_admittance_rule.hpp"

#include <iostream>  // for debug purposes...

#include "control_toolbox/filters.hpp"


namespace cartesian_vic_controller
{

controller_interface::return_type VanillaCartesianAdmittanceRule::init(
  const std::shared_ptr<cartesian_vic_controller::ParamListener> & parameter_handler)
{
  // Initialize CartesianVicRule
  control_mode_ = ControlMode::ADMITTANCE;
  auto ret = CartesianVicRule::init(parameter_handler);
  return ret;
}

controller_interface::return_type VanillaCartesianAdmittanceRule::configure(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node,
  const size_t num_joints)
{
  // Configure CartesianVicRule
  auto ret = CartesianVicRule::configure(node, num_joints);

  // Reset internal data
  reset_rule__internal_storage(num_joints);

  // Check if we still are in impedance mode
  if (control_mode_ != ControlMode::ADMITTANCE ||
    vic_state_.control_mode != ControlMode::ADMITTANCE)
  {
    return controller_interface::return_type::ERROR;
  }
  return ret;
}

controller_interface::return_type VanillaCartesianAdmittanceRule::reset(const size_t num_joints)
{
  reset_rule__internal_storage(num_joints);
  return CartesianVicRule::reset(num_joints);
}

bool VanillaCartesianAdmittanceRule::compute_controls(
  double dt /*period in seconds*/,
  const VicInputData & vic_input_data,
  VicCommandData & vic_command_data)
{
  const CompliantFrame & reference_compliant_frame =
    vic_input_data.reference_compliant_frames.get_compliant_frame(0);

  // No passivation of impedance parameters
  vic_command_data.inertia = reference_compliant_frame.inertia;
  vic_command_data.stiffness = reference_compliant_frame.stiffness;
  vic_command_data.damping = reference_compliant_frame.damping;

  // Prepare data
  // --------------------------------

  // auto rot_base_control = vic_transforms_.base_control_.rotation();
  auto rot_base_admittance = vic_transforms_.base_vic_.rotation();

  // Express M, K, D matrices in base (provided in base_vic frame)
  Eigen::Matrix<double, 6, 6> K = Eigen::Matrix<double, 6, 6>::Zero();
  K.block<3, 3>(0, 0) =
    rot_base_admittance * \
    vic_command_data.stiffness.block<3, 3>(0, 0) * \
    rot_base_admittance.transpose();
  K.block<3, 3>(3, 3) =
    rot_base_admittance * \
    vic_command_data.stiffness.block<3, 3>(3, 3) * \
    rot_base_admittance.transpose();

  Eigen::Matrix<double, 6, 6> D = Eigen::Matrix<double, 6, 6>::Zero();
  D.block<3, 3>(0, 0) =
    rot_base_admittance * \
    vic_command_data.damping.block<3, 3>(0, 0) * \
    rot_base_admittance.transpose();
  D.block<3, 3>(3, 3) =
    rot_base_admittance * \
    vic_command_data.damping.block<3, 3>(3, 3) * \
    rot_base_admittance.transpose();

  Eigen::Matrix<double, 6, 6> M = Eigen::Matrix<double, 6, 6>::Zero();
  M.block<3, 3>(0, 0) =
    rot_base_admittance * \
    vic_command_data.inertia.block<3, 3>(0, 0) * \
    rot_base_admittance.transpose();
  M.block<3, 3>(3, 3) =
    rot_base_admittance * \
    vic_command_data.inertia.block<3, 3>(3, 3) * \
    rot_base_admittance.transpose();

  Eigen::Matrix<double, 6, 6> M_inv = M.inverse();

  // Compute pose tracking errors
  Eigen::Matrix<double, 6, 1> error_pose;
  error_pose.head(3) =
    reference_compliant_frame.pose.translation() - \
    vic_input_data.robot_current_pose.translation();

  auto R_angular_error = \
    reference_compliant_frame.pose.rotation() * \
    vic_input_data.robot_current_pose.rotation().transpose();
  auto angle_axis = Eigen::AngleAxisd(R_angular_error);

  error_pose.tail(3) = angle_axis.angle() * angle_axis.axis();

  // Compute velocity tracking errors in ft frame
  Eigen::Matrix<double, 6, 1> error_velocity =
    reference_compliant_frame.velocity - \
    vic_input_data.robot_current_velocity;

  // External force at interaction frame (assumed to be control frame), expressed in the base frame
  // (note that this is the measured force, the the generalized wrench used in VIC papers...)
  Eigen::Matrix<double, 6, 1> F_ext = vic_input_data.robot_current_wrench_at_ft_frame;

  // Compute admittance control law in the base frame
  // ------------------------------------------------
  // commanded_acc = p_ddot_desired + inv(M) * (K * err_p + D * err_p_dot + f_ext)
  // where err_p = p_desired - p_current

  Eigen::Matrix<double, 6, 1> commanded_cartesian_acc =
    reference_compliant_frame.acceleration + \
    M_inv * (K * error_pose + D * error_velocity + F_ext);

  robot_command_twist_ += last_robot_commanded_twist_ + commanded_cartesian_acc * dt;

  auto previous_jnt_cmd_velocity = vic_command_data.joint_command_velocity;

  bool success = dynamics_->convert_cartesian_deltas_to_joint_deltas(
    vic_input_data.joint_state_position,
    robot_command_twist_,
    vic_input_data.control_frame,
    vic_command_data.joint_command_velocity
  );

  // Filter joint command velocity
  double cutoff_freq_cmd = parameters_.filters.command_filter_cuttoff_freq;
  if (cutoff_freq_cmd > 0.0) {  // No smoothing otherwise
    double cmd_filter_coefficient = 1.0 - exp(-dt * 2 * 3.14 * cutoff_freq_cmd);

    for (size_t i = 0; i < static_cast<size_t>(
        vic_command_data.joint_command_velocity.size()); i++)
    {
      vic_command_data.joint_command_velocity(i) = filters::exponentialSmoothing(
        vic_command_data.joint_command_velocity(i),
        previous_jnt_cmd_velocity(i),
        cmd_filter_coefficient
      );
    }
  }

  // Integrate motion in joint space
  vic_command_data.joint_command_position += \
    vic_command_data.joint_command_velocity * dt;

  // Set flags for available commands
  vic_command_data.has_position_command = true;
  vic_command_data.has_velocity_command = true;
  vic_command_data.has_acceleration_command = false;
  vic_command_data.has_effort_command = false;

  // Just to be safe
  vic_command_data.joint_command_acceleration.setZero();
  vic_command_data.joint_command_effort.setZero();

  // Logging
  // --------------------------------
  // Nothing to log...

  return success;
}

bool VanillaCartesianAdmittanceRule::reset_rule__internal_storage(const size_t /*num_joints*/)
{
  robot_command_twist_.setZero();
  last_robot_commanded_twist_.setZero();
  return true;
}


}  // namespace cartesian_vic_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  cartesian_vic_controller::VanillaCartesianAdmittanceRule,
  cartesian_vic_controller::CartesianVicRule
)
