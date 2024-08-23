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

#include <cmath>
#include <iostream>  // for debug purposes...

#include "control_toolbox/filters.hpp"


namespace cartesian_vic_controller
{

controller_interface::return_type VanillaCartesianImpedanceRule::init(
  const std::shared_ptr<cartesian_vic_controller::ParamListener> & parameter_handler)
{
  logger_ = rclcpp::get_logger("vanilla_cartesian_impedance_rule");
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
  bool success = true;

  size_t num_joints = vic_state_.input_data.joint_state_position.size();
  size_t dims = 6;  // 6 DoF

  if (dt <= 0.0) {
    RCLCPP_ERROR(logger_, "Sampling time should be positive, received %f", dt);
    success = false;
  }

  // Get reference compliant frame at t_k
  RCLCPP_DEBUG(logger_, "Reading reference frame...");
  const CompliantFrame & reference_compliant_frame =
    vic_input_data.reference_compliant_frames.get_compliant_frame(0);

  // No passivation of impedance parameters
  vic_command_data.inertia = reference_compliant_frame.inertia;
  vic_command_data.stiffness = reference_compliant_frame.stiffness;
  vic_command_data.damping = reference_compliant_frame.damping;

  // Prepare VIC data
  // --------------------------------
  RCLCPP_DEBUG(logger_, "Preparing data...");

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

  // Retrieve forces if needed (not used if use_natural_robot_inertia is set to True)
  // RQ: external force at interaction frame (assumed to be control frame),
  // expressed in the base frame
  RCLCPP_DEBUG(logger_, "Reading forces...");
  Eigen::Matrix<double, 6, 1> F_ext = Eigen::Matrix<double, 6, 1>::Zero();
  if (vic_input_data.has_ft_sensor()) {
    F_ext = -vic_input_data.get_ft_sensor_wrench();
  } else {
    if (!parameters_.vic.use_natural_robot_inertia) {
      success &= false;
      RCLCPP_ERROR(
        logger_,
        "F/T sensor is required for inertia shaping! Setting wrenches to zero!"
      );
    }
  }

  // Retrieve external joint torques
  RCLCPP_DEBUG(logger_, "Reading external torques...");
  if (vic_input_data.has_external_torque_sensor()) {
    external_joint_torques_ = vic_input_data.get_joint_state_external_torques();
  } else {
    external_joint_torques_.setZero();
  }

  // Compute Kinematics and Dynamics
  RCLCPP_DEBUG(logger_, "computing J and J_dot...");
  bool model_is_ok = dynamics_->calculate_jacobian(
    vic_input_data.joint_state_position,
    vic_input_data.end_effector_frame,
    J_
  );
  model_is_ok &= dynamics_->calculate_jacobian_derivative(
    vic_input_data.joint_state_position,
    vic_input_data.joint_state_velocity,
    vic_input_data.end_effector_frame,
    J_dot_
  );
  RCLCPP_DEBUG(logger_, "Computing J_pinv...");
  const Eigen::JacobiSVD<Eigen::MatrixXd> J_svd =
    Eigen::JacobiSVD<Eigen::MatrixXd>(J_, Eigen::ComputeThinU | Eigen::ComputeThinV);
  double conditioning_J = 1000.0;
  if (J_.cols() < 6) {
    conditioning_J = J_svd.singularValues()(0) / J_svd.singularValues()(J_.cols() - 1);
  } else {
    conditioning_J = J_svd.singularValues()(0) / J_svd.singularValues()(dims - 1);
  }
  if (conditioning_J > 30) {
    success = false;
    RCLCPP_ERROR(
      logger_,
      "Jacobian singularity detected (max(singular values)/min(singular values) = %lf)!",
      conditioning_J
    );
  }
  J_pinv_ = (J_.transpose() * J_ + alpha_pinv_ * I_joint_space_).inverse() * J_.transpose();

  if (!model_is_ok) {
    success = false;
    RCLCPP_ERROR(
      logger_,
      "Failed to calculate kinematic model!"
    );
  }

  // Compute desired inertia matrix and its inverse
  RCLCPP_DEBUG(logger_, "Computing M_inv...");
  Eigen::Matrix<double, 6, 6> M_inv;
  if (parameters_.vic.use_natural_robot_inertia) {
    M = vic_input_data.natural_cartesian_inertia;
    // M_inv = vic_input_data.natural_cartesian_inertia.llt().solve(I_);
    M_inv = vic_input_data.natural_cartesian_inertia.inverse();
    RCLCPP_INFO_THROTTLE(
      logger_,
      internal_clock_,
      5000,
      "Using natural robot inertia as desired inertia matrix."
    );
  } else {
    // M_inv = M.llt().solve(I_);
    M_inv = M.inverse();
  }

  // Compute impedance control law in the base frame
  // ------------------------------------------------
  // VIC rule: M * err_p_ddot + D * err_p_dot + K * err_p = F_ext - F_ref
  // where err_p = p_desired - p_current
  //
  // -> commanded_acc = p_ddot_desired + inv(M) * (K * err_p + D * err_p_dot - F_ext + F_ref)
  if (parameters_.vic.use_natural_robot_inertia) {
    RCLCPP_WARN_THROTTLE(
      logger_,
      internal_clock_,
      10000,
      "FYI: using natural robot inertia for impedance control."
    );
    RCLCPP_DEBUG(logger_, "Cmd joint acc...");
    // Simplified impedance controller without F/T sensor
    // see https://www.diag.uniroma1.it/~deluca/rob2_en/15_ImpedanceControl.pdf (page 13)
    vic_command_data.joint_command_acceleration = J_pinv_ * \
      (reference_compliant_frame.acceleration - J_dot_ * vic_input_data.joint_state_velocity) + \
      J_.transpose() * (K * error_pose + D * error_velocity + reference_compliant_frame.wrench);
  } else {
    // Implement "normal" impedance control
    RCLCPP_DEBUG(logger_, "Cmd cartesian acc...");
    Eigen::Matrix<double, 6, 1> commanded_cartesian_acc = reference_compliant_frame.acceleration + \
      M_inv * (K * error_pose + D * error_velocity - F_ext + reference_compliant_frame.wrench);

    // Compute joint command accelerations
    RCLCPP_DEBUG(logger_, "Cmd joint acc...");
    vic_command_data.joint_command_acceleration = \
      J_pinv_ * (commanded_cartesian_acc - J_dot_ * vic_input_data.joint_state_velocity);
  }

  // Nullspace objective for stability
  // ------------------------------------------------
  if (vic_input_data.activate_nullspace_control) {
    RCLCPP_DEBUG(logger_, "Cmd nullspace joint acc...");
    nullspace_projection_ = I_joint_space_ - J_pinv_ * J_;
    M_nullspace_.diagonal() = vic_input_data.nullspace_joint_inertia;
    K_nullspace_.diagonal() = vic_input_data.nullspace_joint_stiffness;
    D_nullspace_.diagonal() = vic_input_data.nullspace_joint_damping;
    M_inv_nullspace_.diagonal() = M_nullspace_.diagonal().cwiseInverse();
    auto error_position_nullspace = \
      vic_input_data.nullspace_desired_joint_positions - vic_input_data.joint_state_position;
    // Add nullspace contribution to joint accelerations
    vic_command_data.joint_command_acceleration += nullspace_projection_ * M_inv_nullspace_ * (
      -D_nullspace_ * vic_input_data.joint_state_velocity +
      K_nullspace_ * error_position_nullspace +
      external_joint_torques_
    );
  } else {
    // Pure (small) damping in nullspace for stability
    RCLCPP_WARN_THROTTLE(
      logger_,
      internal_clock_,
      10000,  // every 10 seconds
      "WARNING! nullspace impedance control is disabled!"
    );
    RCLCPP_DEBUG(logger_, "Cmd nullspace damping (joint acc)...");
  }

  // Compute joint command effort from desired joint acc.
  // ------------------------------------------------
  RCLCPP_DEBUG(logger_, "Cmd joint torque...");
  if (parameters_.vic.use_natural_robot_inertia) {
    raw_joint_command_effort_ = \
      vic_input_data.natural_joint_space_inertia.diagonal().asDiagonal() *
      vic_command_data.joint_command_acceleration;
      // Note: F_ext is already cancelled in IC Eq.
      // See https://www.diag.uniroma1.it/deluca/rob2_en/15_ImpedanceControl.pdf (page 9)
      // J_.transpose() * (vic_input_data.natural_cartesian_inertia * M_inv - I_) * F_ext;
  } else {
    raw_joint_command_effort_ = \
      vic_input_data.natural_joint_space_inertia.diagonal().asDiagonal()
      * vic_command_data.joint_command_acceleration
      + J_.transpose() * F_ext;
  }

  // Gravity compensation
  // ------------------------------------------------
  if (vic_input_data.activate_gravity_compensation) {
    RCLCPP_DEBUG(logger_, "Gravity and coriolis compensation...");
    bool dynamic_model_is_ok = dynamics_->calculate_coriolis(
      vic_input_data.joint_state_position,
      vic_input_data.joint_state_velocity,
      coriolis_);
    dynamic_model_is_ok &= dynamics_->calculate_gravity(
      vic_input_data.joint_state_position,
      gravity_);
    if (!dynamic_model_is_ok) {
      success = false;
      RCLCPP_ERROR(
        logger_,
        "Failed to calculate dynamics (coriolis and gravity)!"
      );
    }
    raw_joint_command_effort_ += coriolis_;
    raw_joint_command_effort_ += gravity_;
    // TODO(tpoignonec): investigate Orocos implementation! This should be negative,
    //  but with "+= - gravity", the robot falls...
    // std::cout << "gravity = " << gravity_.transpose() << std::endl;
    // std::cout << "coriolis = " << coriolis_.transpose() << std::endl;
  } else {
    RCLCPP_WARN_THROTTLE(
      logger_,
      internal_clock_,
      10000,  // every 10 seconds
      "FYI: gravity compensation is disabled."
    );
  }

  // Filter joint command effort
  // ------------------------------------------------
  double cutoff_freq_cmd = parameters_.filters.command_filter_cuttoff_freq;
  if (cutoff_freq_cmd > 0.0) {
    RCLCPP_DEBUG(logger_, "Filter torque cmd...");
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
  vic_command_data.has_acceleration_command = true;
  vic_command_data.has_effort_command = true;

  // Just to be safe
  vic_command_data.joint_command_position = \
    vic_input_data.joint_state_position;
  vic_command_data.joint_command_velocity.setZero();
  vic_command_data.joint_command_acceleration.setZero();

  // Logging
  // ------------------------------------------------
  // Here, nothing to log besides VIC state
  // ------------------------------------------------
  return success;
}

bool VanillaCartesianImpedanceRule::reset_rule__internal_storage(const size_t num_joints)
{
  I_ = Eigen::Matrix<double, 6, 6>::Identity();
  I_joint_space_ = \
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Identity(num_joints, num_joints);

  J_ = Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, num_joints);
  J_pinv_ = Eigen::Matrix<double, Eigen::Dynamic, 6>::Zero(num_joints, 6);
  J_dot_ = Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, num_joints);
  nullspace_projection_ = \
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(num_joints, num_joints);

  raw_joint_command_effort_ = Eigen::VectorXd::Zero(num_joints);
  coriolis_ = Eigen::VectorXd::Zero(num_joints);
  gravity_ = Eigen::VectorXd::Zero(num_joints);

  // Nullspace control
  M_nullspace_ = \
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(num_joints, num_joints);
  M_inv_nullspace_ = \
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(num_joints, num_joints);
  K_nullspace_ = \
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(num_joints, num_joints);
  D_nullspace_ = \
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(num_joints, num_joints);
  external_joint_torques_ = Eigen::VectorXd::Zero(num_joints);
  return true;
}

}  // namespace cartesian_vic_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  cartesian_vic_controller::VanillaCartesianImpedanceRule,
  cartesian_vic_controller::CartesianVicRule
)
