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

#include "cartesian_vic_controller/rules/fd_impedance_rule.hpp"

#include <cmath>
#include <iostream>  // for debug purposes...

#include "control_toolbox/filters.hpp"


namespace cartesian_vic_controller
{

controller_interface::return_type FdImpedanceRule::init(
  const std::shared_ptr<cartesian_vic_controller::ParamListener> & parameter_handler)
{
  logger_ = rclcpp::get_logger("fd_impedance_rule");
  // Initialize CartesianVicRule
  control_mode_ = ControlMode::IMPEDANCE;
  auto ret = CartesianVicRule::init(parameter_handler);
  return ret;
}

controller_interface::return_type FdImpedanceRule::configure(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node,
  const size_t num_joints)
{
  if (num_joints != 3 && num_joints != 6) {
    RCLCPP_ERROR(
      logger_,
      "For a Force Dimension device, either 3 or 6 joints are expected (got %li)",
      num_joints
    );
    return controller_interface::return_type::ERROR;
  }
  reset_rule__internal_storage(num_joints);
  return CartesianVicRule::configure(node, num_joints);
}

controller_interface::return_type FdImpedanceRule::reset(const size_t num_joints)
{
  reset_rule__internal_storage(num_joints);
  return CartesianVicRule::reset(num_joints);
}

bool FdImpedanceRule::compute_controls(
  double dt /*period in seconds*/,
  const VicInputData & vic_input_data,
  VicCommandData & vic_command_data)
{
  bool success = true;
  // auto num_joints = vic_input_data.joint_state_position.size();

  // Get reference compliant frame at t_k
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

  // Compute Kinematics and Dynamics
  bool model_is_ok = dynamics_->calculate_jacobian(
    vic_input_data.joint_state_position,
    vic_input_data.control_frame,
    J_
  );
  model_is_ok &= dynamics_->calculate_jacobian_derivative(
    vic_input_data.joint_state_position,
    vic_input_data.joint_state_velocity,
    vic_input_data.control_frame,
    J_dot_
  );
  J_pinv_ = (J_.transpose() * J_ + alpha_pinv_ * I_joint_space_).inverse() * J_.transpose();

  model_is_ok &= dynamics_->calculate_inertia(
    vic_input_data.joint_state_position,
    M_joint_space_
  );

  // Check if the model is ok
  if (!model_is_ok) {
    success = false;
    RCLCPP_ERROR(
      logger_,
      "Failed to calculate kinematic / dynamic model!"
    );
  }

  // External force at interaction frame (assumed to be control frame)
  Eigen::Matrix<double, 6, 1> F_ext = Eigen::Matrix<double, 6, 1>::Zero();
  if (parameters_.vic.use_natural_robot_inertia) {
    M_cartesian_space_ = (J_ * M_joint_space_.inverse() * J_.transpose()).inverse();
    M = M_cartesian_space_;
  } else {
    if (vic_input_data.has_ft_sensor()) {
      F_ext = -vic_input_data.get_ft_sensor_wrench();
    } else {
      RCLCPP_ERROR(
        logger_,
        "F/T sensor is required for inertia shaping! Setting wrenches to zero!"
      );
      success &= false;
    }
  }

  if (parameters_.vic.use_natural_robot_inertia) {
    RCLCPP_WARN_THROTTLE(
      logger_,
      internal_clock_,
      10000,
      "FYI: using natural robot inertia for impedance control."
    );
    // Simplified impedance controller without F/T sensor
    // see https://www.diag.uniroma1.it/~deluca/rob2_en/15_ImpedanceControl.pdf (page 13)
    raw_joint_command_effort_ = \
      M_joint_space_ * J_pinv_ *
      (reference_compliant_frame.acceleration - J_dot_ * vic_input_data.joint_state_velocity) + \
      J_.transpose() * (K * error_pose + D * error_velocity);
  } else {
    // Implement "normal" impedance control
    Eigen::Matrix<double, 6, 1> commanded_cartesian_acc = reference_compliant_frame.acceleration + \
      M_inv * (K * error_pose + D * error_velocity - F_ext);

    // Compute joint command accelerations
    vic_command_data.joint_command_acceleration = \
      J_pinv_ * (commanded_cartesian_acc - J_dot_ * vic_input_data.joint_state_velocity);

    // Compute joint command effort from desired joint acc.
    raw_joint_command_effort_ = M_joint_space_.diagonal().asDiagonal() *
      vic_command_data.joint_command_acceleration - \
      J_.transpose() * F_ext;
  }

  // Filter joint command effort
  // ------------------------------------------------
  double cutoff_freq_cmd = parameters_.filters.command_filter_cuttoff_freq;
  if (cutoff_freq_cmd > 0.0) {
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
  } else {
    // No smoothing otherwise
    vic_command_data.joint_command_effort = raw_joint_command_effort_;
  }

  // Set flags for available commands
  // ------------------------------------------------
  vic_command_data.has_position_command = false;
  vic_command_data.has_velocity_command = false;
  vic_command_data.has_acceleration_command = false;
  vic_command_data.has_effort_command = true;

  // Just to be safe
  vic_command_data.joint_command_position = \
    vic_input_data.joint_state_position;
  vic_command_data.joint_command_velocity.setZero();
  vic_command_data.joint_command_acceleration.setZero();

  // Logging
  // ------------------------------------------------

  return success;
}

bool FdImpedanceRule::reset_rule__internal_storage(const size_t num_joints)
{
  raw_joint_command_effort_ = Eigen::VectorXd::Zero(num_joints);

  J_ = Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, num_joints);
  J_pinv_ = Eigen::Matrix<double, Eigen::Dynamic, 6>::Zero(num_joints, 6);
  J_dot_ = Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, num_joints);

  I_joint_space_ = \
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Identity(num_joints, num_joints);
  M_joint_space_ = \
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(num_joints, num_joints);

  M_cartesian_space_.setZero();
  return true;
}

}  // namespace cartesian_vic_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  cartesian_vic_controller::FdImpedanceRule,
  cartesian_vic_controller::CartesianVicRule
)
