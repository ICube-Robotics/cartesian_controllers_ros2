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

#include "cartesian_vic_controller/rules/vanilla_cartesian_impedance_rule.hpp"

#include <iostream>  // for debug purposes...

#include "control_toolbox/filters.hpp"


namespace cartesian_vic_controller
{

controller_interface::return_type VanillaCartesianImpedanceRule::init(
  const std::shared_ptr<cartesian_vic_controller::ParamListener> & parameter_handler)
{
  // Initialize CartesianVicRule
  control_mode_ = ControlMode::IMPEDANCE;
  auto ret = CartesianVicRule::init(parameter_handler);
  return ret;
}

controller_interface::return_type VanillaCartesianImpedanceRule::configure(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node,
  const size_t num_joints)
{
  reset_rule__internal_storage(num_joints);
  return CartesianVicRule::configure(node, num_joints);
}

controller_interface::return_type VanillaCartesianImpedanceRule::reset(const size_t num_joints)
{
  reset_rule__internal_storage(num_joints);
  return CartesianVicRule::reset(num_joints);
}

bool VanillaCartesianImpedanceRule::compute_controls(
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

  // Prepare VIC data
  // --------------------------------

  // auto rot_base_control = vic_transforms_.base_control_.rotation();
  auto rot_base_impedance = vic_transforms_.base_vic_.rotation();
  // Express M, K, D matrices in base (provided in base_vic frame)

  Eigen::Matrix<double, 6, 6> K = Eigen::Matrix<double, 6, 6>::Zero();
  K.block<3, 3>(0, 0) =
    rot_base_impedance * \
    vic_command_data.stiffness.block<3, 3>(0, 0) * \
    rot_base_impedance.transpose();
  K.block<3, 3>(3, 3) =
    rot_base_impedance * \
    vic_command_data.stiffness.block<3, 3>(3, 3) * \
    rot_base_impedance.transpose();

  Eigen::Matrix<double, 6, 6> D = Eigen::Matrix<double, 6, 6>::Zero();
  D.block<3, 3>(0, 0) =
    rot_base_impedance * \
    vic_command_data.damping.block<3, 3>(0, 0) * \
    rot_base_impedance.transpose();
  D.block<3, 3>(3, 3) =
    rot_base_impedance * \
    vic_command_data.damping.block<3, 3>(3, 3) * \
    rot_base_impedance.transpose();

  Eigen::Matrix<double, 6, 6> M = Eigen::Matrix<double, 6, 6>::Zero();
  M.block<3, 3>(0, 0) =
    rot_base_impedance * \
    vic_command_data.inertia.block<3, 3>(0, 0) * \
    rot_base_impedance.transpose();
  M.block<3, 3>(3, 3) =
    rot_base_impedance * \
    vic_command_data.inertia.block<3, 3>(3, 3) * \
    rot_base_impedance.transpose();

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

  // Compute impedance control law in the base frame
  // ------------------------------------------------
  // commanded_acc = p_ddot_desired + inv(M) * (K * err_p + D * err_p_dot + f_ext)

  Eigen::Matrix<double, 6, 1> commanded_cartesian_acc =
    reference_compliant_frame.acceleration + \
    M_inv * (K * error_pose + D * error_velocity + F_ext);
  // std::cerr << "commanded_cartesian_acc = " << commanded_cartesian_acc.transpose() << std::endl;

  auto num_joints = vic_input_data.joint_state_position.size();

  bool success = dynamics_->calculate_inertia(
    vic_input_data.joint_state_position,
    M_joint_space_
  );
  success &= dynamics_->calculate_coriolis(
    vic_input_data.joint_state_position,
    vic_input_data.joint_state_velocity,
    coriolis_
  );
  success &= dynamics_->calculate_gravity(
    vic_input_data.joint_state_position,
    gravity_
  );
  if (!success) {
    std::cerr << "Failed to calculate dynamic model!" << std::endl;
  }

  success &= dynamics_->calculate_jacobian_derivative(
    vic_input_data.joint_state_position,
    vic_input_data.joint_state_velocity,
    vic_input_data.control_frame,
    J_dot_
  );
  Eigen::Matrix<double, 6, 1> corrected_cartesian_acc = \
    commanded_cartesian_acc - J_dot_ * vic_input_data.joint_state_velocity;

  success &= dynamics_->convert_cartesian_deltas_to_joint_deltas(
    vic_input_data.joint_state_position,
    corrected_cartesian_acc,
    vic_input_data.control_frame,
    vic_command_data.joint_command_acceleration
  );

  // Compute joint command effort from desired joint acc.
  raw_joint_command_effort_ = M_joint_space_.diagonal().asDiagonal() * \
    vic_command_data.joint_command_acceleration - coriolis_ - gravity_;

  // Filter joint command effort
  double cutoff_freq_cmd = parameters_.filters.command_filter_cuttoff_freq;
  if (cutoff_freq_cmd > 0.0) {  // No smoothing otherwise
    double cmd_filter_coefficient = 1.0 - exp(-dt * 2 * 3.14 * cutoff_freq_cmd);

    for (size_t i = 0; i < static_cast<size_t>(
        vic_command_data.joint_command_effort.size()); i++)
    {
      vic_command_data.joint_command_effort(i) = filters::exponentialSmoothing(
        vic_command_data.joint_command_effort(i),
        raw_joint_command_effort_(i),
        cmd_filter_coefficient
      );
    }
  }

  // Set flags for available commands
  vic_command_data.has_position_command = false;
  vic_command_data.has_velocity_command = false;
  vic_command_data.has_acceleration_command = true;
  vic_command_data.has_effort_command = true;

  // Just to be safe
  vic_command_data.joint_command_position = \
    vic_input_data.joint_state_position;
  vic_command_data.joint_command_velocity.setZero();
  vic_command_data.joint_command_acceleration.setZero();

  // Logging

  return success;
}

bool VanillaCartesianImpedanceRule::reset_rule__internal_storage(const size_t num_joints)
{
  M_joint_space_ = \
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(num_joints, num_joints);
  J_dot_ = Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, num_joints);
  raw_joint_command_effort_ = Eigen::VectorXd::Zero(num_joints);
  coriolis_ = Eigen::VectorXd::Zero(num_joints);
  gravity_ = Eigen::VectorXd::Zero(num_joints);
  return true;
}

}  // namespace cartesian_vic_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  cartesian_vic_controller::VanillaCartesianImpedanceRule,
  cartesian_vic_controller::CartesianVicRule
)
