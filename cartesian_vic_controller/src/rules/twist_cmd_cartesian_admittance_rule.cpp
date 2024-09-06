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

#include "cartesian_vic_controller/rules/twist_cmd_cartesian_admittance_rule.hpp"

#include <iostream>  // for debug purposes...

#include "control_toolbox/filters.hpp"


namespace cartesian_vic_controller
{

controller_interface::return_type TwistCmdCartesianAdmittanceRule::init(
  const std::shared_ptr<cartesian_vic_controller::ParamListener> & parameter_handler)
{
  logger_ = rclcpp::get_logger("twist_cmd_cartesian_admittance_rule");
  // Initialize CartesianVicRule
  control_mode_ = ControlMode::ADMITTANCE;
  auto ret = CartesianVicRule::init(parameter_handler);
  return ret;
}

controller_interface::return_type TwistCmdCartesianAdmittanceRule::configure(
  const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> & parameters_interface,
  const size_t num_joints)
{
  // Configure CartesianVicRule
  auto ret = CartesianVicRule::configure(parameters_interface, num_joints);

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

controller_interface::return_type TwistCmdCartesianAdmittanceRule::reset(const size_t num_joints)
{
  reset_rule__internal_storage(num_joints);
  return CartesianVicRule::reset(num_joints);
}

bool TwistCmdCartesianAdmittanceRule::compute_controls(
  double dt /*period in seconds*/,
  const VicInputData & vic_input_data,
  VicCommandData & vic_command_data)
{
  bool success = true;

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

  // Prepare data
  // --------------------------------
  RCLCPP_DEBUG(logger_, "Preparing data...");

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

  // Copy previous command velocity (used for integration)
  RCLCPP_DEBUG(logger_, "Save last velocity...");
  auto previous_twist_cmd = vic_command_data.twist_command;

  // Compute admittance control law in the base frame
  // ------------------------------------------------
  // VIC rule: M * err_p_ddot + D * err_p_dot + K * err_p = F_ext - F_ref
  // where err_p = p_desired - p_current
  //
  // -> commanded_acc = p_ddot_desired + inv(M) * (K * err_p + D * err_p_dot - F_ext + F_ref)
  // Implement "normal" impedance control
  RCLCPP_DEBUG(logger_, "Cmd cartesian acc...");
  Eigen::Matrix<double, 6, 1> commanded_cartesian_acc = reference_compliant_frame.acceleration + \
    M_inv * (K * error_pose + D * error_velocity - F_ext + reference_compliant_frame.wrench);

  // TODO(tpoignonec): clip cartesian acceleration in min/max range
  // (and same for velocity if possible)

  // Compute velocity command
  RCLCPP_DEBUG(logger_, "Integration acc...");
  vic_command_data.twist_command += commanded_cartesian_acc * dt;

  // Alternative admittance control without velocity integration
  /*
  vic_command_data.twist_command = D.inverse() * (
    K * error_pose
    + M * (reference_compliant_frame.acceleration - vic_input_data.robot_estimated_acceleration)
    - F_ext  + reference_compliant_frame.wrench
    ) + reference_compliant_frame.velocity;
  */

  // Filter twist command
  // ------------------------------------------------
  double cutoff_freq_cmd = parameters_.filters.command_filter_cuttoff_freq;
  if (cutoff_freq_cmd > 0.0) {  // No smoothing otherwise
    RCLCPP_DEBUG(logger_, "Filter velocity cmd...");
    double cmd_filter_coefficient = 1.0 - exp(-dt * 2 * 3.14 * cutoff_freq_cmd);

    for (size_t i = 0; i < static_cast<size_t>(
        vic_command_data.twist_command.size()); i++)
    {
      vic_command_data.twist_command(i) = filters::exponentialSmoothing(
        vic_command_data.twist_command(i),
        previous_twist_cmd(i),
        cmd_filter_coefficient
      );
    }
  }

  // Set flags for available commands
  vic_command_data.has_twist_command = true;

  vic_command_data.has_position_command = false;
  vic_command_data.has_velocity_command = false;
  vic_command_data.has_acceleration_command = false;
  vic_command_data.has_effort_command = false;

  // Just to be safe
  vic_command_data.joint_command_position = vic_input_data.joint_state_position;
  vic_command_data.joint_command_velocity.setZero();
  vic_command_data.joint_command_acceleration.setZero();
  vic_command_data.joint_command_effort.setZero();

  // Logging
  // --------------------------------
  // Nothing to log...

  return success;
}

bool TwistCmdCartesianAdmittanceRule::reset_rule__internal_storage(const size_t num_joints)
{
  // Nothing to do
  return true;
}


}  // namespace cartesian_vic_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  cartesian_vic_controller::TwistCmdCartesianAdmittanceRule,
  cartesian_vic_controller::CartesianVicRule
)
