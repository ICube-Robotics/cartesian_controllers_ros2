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
  logger_ = rclcpp::get_logger("vanilla_cartesian_admittance_rule");
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
  bool success = true;

  if (dt <= 0.0) {
    RCLCPP_ERROR(logger_, "Sampling time should be positive, received %f", dt);
    success = false;
  }

  size_t num_joints = vic_state_.input_data.joint_state_position.size();
  size_t dims = 6;  // 6 DoF

  // auto num_joints = vic_input_data.joint_state_position.size();
  // Get reference compliant frame at t_k
  RCLCPP_DEBUG(logger_, "Reading reference frame...");
  const CompliantFrame & reference_compliant_frame =
    vic_input_data.reference_compliant_frames.get_compliant_frame(0);

  // No passivation of impedance parameters
  vic_command_data.inertia = reference_compliant_frame.inertia;
  vic_command_data.stiffness = reference_compliant_frame.stiffness;
  vic_command_data.damping = reference_compliant_frame.damping;

  // Prepare data
  // --------------------------------
  RCLCPP_DEBUG(logger_, "Preparing data...");


  // Copy previous command velocity (used for integration)
  auto previous_jnt_cmd_velocity = vic_command_data.joint_command_velocity;

  // auto rot_base_control = vic_transforms_.base_control_.rotation();
  auto rot_base_admittance = vic_transforms_.base_vic_.rotation();

  // Express M, K, D matrices in base (provided in base_vic frame)
  auto registration_MKD = [&rot_base_admittance](
    const Eigen::Matrix<double, 6, 6> & matrix_in_adm_frame,
    Eigen::Matrix<double, 6, 6> & matrix_in_base_frame)
    {
      matrix_in_base_frame.block<3, 3>(0, 0) =
        rot_base_admittance * \
        matrix_in_adm_frame.block<3, 3>(0, 0) * \
        rot_base_admittance.transpose();
      matrix_in_base_frame.block<3, 3>(3, 3) =
        rot_base_admittance * \
        matrix_in_adm_frame.block<3, 3>(3, 3) * \
        rot_base_admittance.transpose();
      return;
    };
  Eigen::Matrix<double, 6, 6> M = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 6> K = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 6> D = Eigen::Matrix<double, 6, 6>::Zero();
  registration_MKD(vic_command_data.inertia, M);
  registration_MKD(vic_command_data.stiffness, K);
  registration_MKD(vic_command_data.damping, D);

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
    success &= false;
    RCLCPP_ERROR(
      logger_,
      "F/T sensor is required for admittance control! Setting wrenches to zero!"
    );
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

  if (!model_is_ok) {
    success = false;
    RCLCPP_ERROR(
      logger_,
      "Failed to calculate kinematic model!"
    );
  }


  RCLCPP_DEBUG(logger_, "Computing J_pinv...");
  const Eigen::JacobiSVD<Eigen::MatrixXd> J_svd =
    Eigen::JacobiSVD<Eigen::MatrixXd>(J_, Eigen::ComputeThinU | Eigen::ComputeThinV);
  double conditioning_J = 1000.0;
  if (J_.cols() < 6) {
    RCLCPP_WARN_THROTTLE(
      logger_, internal_clock_, 5000, "Jacobian has only %u columns, expecting at least 6!!!",
        J_.cols());
    conditioning_J = J_svd.singularValues()(0) / J_svd.singularValues()(J_.cols() - 1);
  } else {
    conditioning_J = J_svd.singularValues()(0) / J_svd.singularValues()(dims - 1);
  }
  if (conditioning_J > 40) {
    success = false;
    std::cerr << "J_svd.singularValues() = " << J_svd.singularValues().transpose() << std::endl;
    RCLCPP_ERROR(
      logger_,
      "Jacobian singularity detected (max(singular values)/min(singular values) = %lf)!",
      conditioning_J
    );
  } else if (conditioning_J > 15) {
    RCLCPP_WARN_THROTTLE(
      logger_,
      internal_clock_,
      5000,
      "Nearing Jacobian singularity (max(singular values)/min(singular values) = %lf), "
      "proceed with caution!",
      conditioning_J
    );
  }
  // J_pinv_ = J_svd.matrixV() * matrix_s.inverse() * J_svd.matrixU().transpose();
  J_pinv_ = (J_.transpose() * J_ + alpha_pinv_ * I_joint_space_).inverse() * J_.transpose();

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

  // Compute admittance control law in the base frame
  // ------------------------------------------------
  // VIC rule: M * err_p_ddot + D * err_p_dot + K * err_p = F_ext - F_ref
  // where err_p = p_desired - p_current
  //

  /*
  auto robot_command_twist = D.inverse() * (
    K * error_pose
    + M * (reference_compliant_frame.acceleration - vic_input_data.robot_estimated_acceleration)
    - F_ext  + reference_compliant_frame.wrench
    ) + reference_compliant_frame.velocity;
  success &= dynamics_->convert_cartesian_deltas_to_joint_deltas(
    vic_input_data.joint_state_position,
    robot_command_twist,
    vic_input_data.end_effector_frame,
    vic_command_data.joint_command_velocity
  );

  // Nullspace objective for stability
  // ------------------------------------------------
  if (false){ //(vic_input_data.activate_nullspace_control) {
    RCLCPP_DEBUG(logger_, "Cmd nullspace joint acc...");
    nullspace_projection_ = I_joint_space_ - J_pinv_ * J_;
    M_nullspace_.diagonal() = vic_input_data.nullspace_joint_inertia;
    K_nullspace_.diagonal() = vic_input_data.nullspace_joint_stiffness;
    D_nullspace_.diagonal() = vic_input_data.nullspace_joint_damping;
    auto D_inv_nullspace_ = D_nullspace_;
    D_inv_nullspace_.diagonal() = D_nullspace_.diagonal().cwiseInverse();
    auto error_position_nullspace = \
      vic_input_data.nullspace_desired_joint_positions - vic_input_data.joint_state_position;
    // Add nullspace contribution to joint velocity
    vic_command_data.joint_command_velocity += nullspace_projection_ * D_inv_nullspace_ * (
      K_nullspace_ * error_position_nullspace
      + external_joint_torques_);
  } else {
    // Pure (small) damping in nullspace for stability
    RCLCPP_WARN_THROTTLE(
      logger_,
      internal_clock_,
      10000,  // every 10 seconds
      "WARNING! nullspace impedance control is disabled!"
    );
  }

  std::cout << "ref vel: " << reference_compliant_frame.velocity.transpose() << std::endl;
  std::cout << "cur vel: " << vic_input_data.robot_current_velocity.transpose() << std::endl;
  std::cout << "ref acc: " << reference_compliant_frame.acceleration.transpose() << std::endl;
  std::cout << "est acc: " << vic_input_data.robot_estimated_acceleration.transpose() << std::endl;
  std::cout << "ref wrench: " << reference_compliant_frame.wrench.transpose() << std::endl;
  std::cout << "D.inv(): " << D.inverse() << std::endl;
  std::cout << "K: " << K << std::endl;
  std::cout << "M: " << M << std::endl;
  std::cout << "robot_command_twist: " << robot_command_twist.transpose() << std::endl;
  */

  // Alternative: use joint accelerations to integrate cartesian velocity
  // Compute joint command accelerations

  // -> commanded_acc = p_ddot_desired + inv(M) * (K * err_p + D * err_p_dot - F_ext + F_ref)
  // Implement "normal" impedance control
  RCLCPP_DEBUG(logger_, "Cmd cartesian acc...");
  Eigen::Matrix<double, 6, 1> commanded_cartesian_acc = reference_compliant_frame.acceleration + \
    M_inv * (K * error_pose + D * error_velocity - F_ext + reference_compliant_frame.wrench);

  // TODO(tpoignonec): clip cartesian acceleration in min/max range
  // (and same for velocity if possible)

  RCLCPP_DEBUG(logger_, "Cmd joint acc...");
  vic_command_data.joint_command_acceleration = \
    J_pinv_ * (commanded_cartesian_acc - J_dot_ * vic_input_data.joint_state_velocity);
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
  }

  // Compute velocity command
  RCLCPP_DEBUG(logger_, "Save last velocity...");
  RCLCPP_DEBUG(logger_, "Integration acc...");
  vic_command_data.joint_command_velocity += \
    vic_command_data.joint_command_acceleration * dt;

  // Detect and handle singularity issues
  /*
  // Singularity detection
  // from https://github.com/moveit/moveit2/blob/8a0c655e2ba48e1f93f551cd52fb5aa093021659/moveit_ros/moveit_servo/src/utils/common.cpp#L282
  double singularity_step_scale = 0.01;
  double hard_stop_singularity_threshold = 30.0;
  double lower_singularity_threshold = 17.0;
  double leaving_singularity_threshold_multiplier = 1.5;

  Eigen::VectorXd vector_towards_singularity = J_svd.matrixU().col(dims - 1);
  const double current_condition_number = J_svd.singularValues()(0) / J_svd.singularValues()(dims - 1);
  const Eigen::VectorXd delta_x = vector_towards_singularity * singularity_step_scale;
  // Compute the new joint angles if we take the small step delta_x
  Eigen::VectorXd next_joint_position = vic_input_data.joint_state_position;
  next_joint_position += J_pinv_ * delta_x;

   // Compute the Jacobian SVD for the new robot state.
  Eigen::Matrix<double, 6, Eigen::Dynamic> J_next = Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, dims);
  dynamics_->calculate_jacobian(
    next_joint_position,
    vic_input_data.end_effector_frame,
    J_next
  );
  const Eigen::JacobiSVD<Eigen::MatrixXd> next_svd = Eigen::JacobiSVD<Eigen::MatrixXd>(
      J_next, Eigen::ComputeThinU | Eigen::ComputeThinV);

  // Compute condition number for the new Jacobian.
  const double next_condition_number = next_svd.singularValues()(0) / next_svd.singularValues()(dims - 1);
  if (next_condition_number <= current_condition_number) {
    vector_towards_singularity *= -1;
  }
  // Double check the direction using dot product.
  const bool moving_towards_singularity = vector_towards_singularity.dot(target_delta_x) > 0;

  // Compute upper condition variable threshold based on if we are moving towards or away from singularity.
  // See https://github.com/moveit/moveit2/pull/620#issuecomment-1201418258 for visual explanation.
  double upper_threshold;
  if (moving_towards_singularity)
  {
    upper_threshold = hard_stop_singularity_threshold;
  }
  else
  {
    const double threshold_size = (hard_stop_singularity_threshold - lower_singularity_threshold);
    upper_threshold = lower_singularity_threshold + (threshold_size * leaving_singularity_threshold_multiplier);
  }

  // Compute the scale based on the current condition number.
  double velocity_scale = 1.0;
  const bool is_above_lower_limit = current_condition_number > lower_singularity_threshold;
  const bool is_below_hard_stop_limit = current_condition_number < hard_stop_singularity_threshold;
  if (is_above_lower_limit && is_below_hard_stop_limit)
  {
    velocity_scale -=
        (current_condition_number - lower_singularity_threshold) / (upper_threshold - lower_singularity_threshold);
  }
  // If condition number has crossed hard stop limit, halt the robot.
  else if (!is_below_hard_stop_limit)
  {
    success = false;
    velocity_scale = 0.0;
  }
  */


  // Filter joint command velocity
  double cutoff_freq_cmd = parameters_.filters.command_filter_cuttoff_freq;
  if (cutoff_freq_cmd > 0.0) {  // No smoothing otherwise
    RCLCPP_DEBUG(logger_, "Filter velocity cmd...");
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

bool VanillaCartesianAdmittanceRule::reset_rule__internal_storage(const size_t num_joints)
{
  I_ = Eigen::Matrix<double, 6, 6>::Identity();
  I_joint_space_ = \
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Identity(num_joints, num_joints);

  J_ = Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, num_joints);
  J_pinv_ = Eigen::Matrix<double, Eigen::Dynamic, 6>::Zero(num_joints, 6);
  J_dot_ = Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, num_joints);

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
  cartesian_vic_controller::VanillaCartesianAdmittanceRule,
  cartesian_vic_controller::CartesianVicRule
)
