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

#include "cartesian_admittance_controller/cartesian_admittance_rule.hpp"
#include "kinematics_interface/kinematics_interface.hpp"

#include "rclcpp/duration.hpp"
#include "control_toolbox/filters.hpp"

#include "pluginlib/class_loader.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_kdl/tf2_kdl.hpp"

namespace cartesian_admittance_controller
{

controller_interface::return_type
CartesianAdmittanceRule::init(
  const std::shared_ptr<cartesian_admittance_controller::ParamListener> & parameter_handler)
{
  parameter_handler_ = parameter_handler;
  parameters_ = parameter_handler_->get_params();
  num_joints_ = parameters_.joints.size();
  admittance_state_ = AdmittanceState(num_joints_);
  wrench_world_.setZero();
  use_streamed_interaction_parameters_ = false;
  return reset(num_joints_);
}

controller_interface::return_type
CartesianAdmittanceRule::configure(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node,
  const size_t num_joints)
{
  num_joints_ = num_joints;
  // reset admittance state
  reset(num_joints);
  // Load the differential IK plugin
  if (!parameters_.kinematics.plugin_name.empty()) {
    try {
      kinematics_loader_ =
        std::make_shared<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>(
        parameters_.kinematics.plugin_package,
        "kinematics_interface::KinematicsInterface");
      kinematics_ = std::unique_ptr<kinematics_interface::KinematicsInterface>(
        kinematics_loader_->createUnmanagedInstance(parameters_.kinematics.plugin_name));
      if (!kinematics_->initialize(
          node->get_node_parameters_interface(), parameters_.kinematics.tip))
      {
        return controller_interface::return_type::ERROR;
      }
    } catch (pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR(
        rclcpp::get_logger("CartesianAdmittanceRule"),
        "Exception while loading the IK plugin '%s': '%s'",
        parameters_.kinematics.plugin_name.c_str(), ex.what()
      );
      return controller_interface::return_type::ERROR;
    }
  } else {
    RCLCPP_ERROR(
      rclcpp::get_logger("CartesianAdmittanceRule"),
      "A differential IK plugin name was not specified in the config file.");
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}


controller_interface::return_type
CartesianAdmittanceRule::init_reference_frame_trajectory(
  const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state)
{
  // Load parameters
  use_streamed_interaction_parameters_ = false;
  apply_parameters_update();

  // Assume force is zero
  geometry_msgs::msg::Wrench dummy_wrench;

  // Update state
  if (!update_internal_state(current_joint_state, dummy_wrench)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("CartesianAdmittanceRule"),
      "Failed to update internal state in 'init_reference_frame_trajectory()'!");
    return controller_interface::return_type::ERROR;
  }

  // Reset robot command
  // TODO(tpoignonec): move elsewhere?
  admittance_state_.joint_command_position = admittance_state_.joint_state_position;
  admittance_state_.joint_command_velocity = admittance_state_.joint_state_velocity;

  // Set current pose as cartesian ref
  auto N = admittance_state_.reference_compliant_frames.N();
  Eigen::Matrix<double, 6, 1> null_vector_6D = Eigen::Matrix<double, 6, 1>::Zero();

  bool success = true;
  for (unsigned int i = 0; i < N; i++) {
    // TODO(tpoignonec): Check the frame is correct (i.e., control w.r.t. base)!
    success &= admittance_state_.reference_compliant_frames.fill_desired_desired_robot_state(
      i,
      admittance_state_.robot_current_pose,
      null_vector_6D,
      null_vector_6D,
      null_vector_6D
    );
    if (!success) {
      RCLCPP_ERROR(
        rclcpp::get_logger("CartesianAdmittanceRule"),
        "Failed to fill the desired robot state for index=%u!",
        i);
      return controller_interface::return_type::ERROR;
    }
  }
  return controller_interface::return_type::OK;
}

controller_interface::return_type
CartesianAdmittanceRule::reset(const size_t num_joints)
{
  // Reset admittance state
  use_streamed_interaction_parameters_ = false;
  admittance_state_ = AdmittanceState(num_joints);

  // Load parameters
  apply_parameters_update();

  return controller_interface::return_type::OK;
}


void CartesianAdmittanceRule::apply_parameters_update()
{
  if (parameter_handler_->is_old(parameters_)) {
    parameters_ = parameter_handler_->get_params();
  }
  // update param values
  admittance_state_.admittance_frame = parameters_.admittance.frame.id;
  admittance_state_.control_frame = parameters_.control.frame.id;
  admittance_state_.ft_sensor_frame = parameters_.ft_sensor.frame.id;

  if (!use_streamed_interaction_parameters_) {
    Eigen::Matrix<double, 6, 1> desired_inertia =
      Eigen::Matrix<double, 6, 1>(parameters_.admittance.inertia.data());
    Eigen::Matrix<double, 6, 1> desired_stiffness =
      Eigen::Matrix<double, 6, 1>(parameters_.admittance.stiffness.data());
    Eigen::Matrix<double, 6, 1> desired_damping;
    for (size_t i = 0; i < 6; ++i) {
      // Compute damping from damping ratio
      desired_damping[i] = parameters_.admittance.damping_ratio[i] * \
        2 * sqrt(desired_inertia[i] * desired_stiffness[i]);
    }
    admittance_state_.reference_compliant_frames.fill_desired_compliance(
      desired_inertia,
      desired_stiffness,
      desired_damping
    );
  }
}

void CartesianAdmittanceRule::set_interaction_parameters(
  const Eigen::Matrix<double, 6, 1> & desired_inertia,
  const Eigen::Matrix<double, 6, 1> & desired_stiffness,
  const Eigen::Matrix<double, 6, 1> & desired_damping)
{
  use_streamed_interaction_parameters_ = true;
  admittance_state_.reference_compliant_frames.fill_desired_compliance(
    desired_inertia,
    desired_stiffness,
    desired_damping);
}


controller_interface::return_type
CartesianAdmittanceRule::update_compliant_frame_trajectory(
  const cartesian_control_msgs::msg::CompliantFrameTrajectory & compliant_frame_trajectory)
{
  // Check cartesian ref trajectory validity
  auto N = compliant_frame_trajectory.cartesian_trajectory_points.size();
  if (admittance_state_.reference_compliant_frames.N() != N) {
    std::cerr \
      << "Warning! 'compliant_frame_trajectory.cartesian_trajectory_points.size() != N'" \
      << " and will be resized..." \
      << std::endl;
    if (!admittance_state_.reference_compliant_frames.resize(N)) {
      return controller_interface::return_type::ERROR;
    }
  }
  // TODO(tpoignonec): check frame_id and child_frame_id
  // Check compliance parameters validity
  if (compliant_frame_trajectory.compliance_at_points.size() == N) {
    use_streamed_interaction_parameters_ = true;
  } else if (compliant_frame_trajectory.compliance_at_points.empty()) {
    if (use_streamed_interaction_parameters_) {
      return controller_interface::return_type::ERROR;
    }
  } else {
    return controller_interface::return_type::ERROR;
  }

  // Update reference compliant frames
  bool success = true;
  for (unsigned int i = 0; i < N; i++) {
    // TODO(tpoignonec): Check the frame is correct (i.e., control w.r.t. base)!
    success &= admittance_state_.reference_compliant_frames.fill_desired_robot_state_from_msg(
      i,
      compliant_frame_trajectory.cartesian_trajectory_points[i]
    );
    if (use_streamed_interaction_parameters_) {
      success &= admittance_state_.reference_compliant_frames.fill_desired_compliance_from_msg(
        i,
        compliant_frame_trajectory.compliance_at_points[i]
      );
    }
  }

  if (success) {
    return controller_interface::return_type::OK;
  } else {
    return controller_interface::return_type::ERROR;
  }
}

controller_interface::return_type
CartesianAdmittanceRule::update(
  const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state,
  const geometry_msgs::msg::Wrench & measured_wrench,
  const rclcpp::Duration & period,
  trajectory_msgs::msg::JointTrajectoryPoint & joint_state_command)
{
  const double dt = period.seconds();

  if (parameters_.enable_parameter_update_without_reactivation) {
    apply_parameters_update();
  }

  // Update current robot state
  bool success = update_internal_state(
    current_joint_state,
    measured_wrench
  );

  // Compute controls
  success &= compute_controls(admittance_state_, dt);

  // If an error is detected, set commanded velocity to zero
  if (!success) {
    RCLCPP_ERROR(
      rclcpp::get_logger("CartesianAdmittanceRule"),
      "Failed to compute the controls!"
    );
    // Set commanded position to the previous one
    joint_state_command.positions = current_joint_state.positions;
    // Set commanded velocity/acc to zero
    std::fill(
      joint_state_command.velocities.begin(),
      joint_state_command.velocities.end(),
      0
    );
    std::fill(
      joint_state_command.accelerations.begin(),
      joint_state_command.accelerations.end(),
      0
    );
    return controller_interface::return_type::ERROR;
  }
  // Otherwise, set joint command and return
  for (size_t i = 0; i < parameters_.joints.size(); ++i) {
    joint_state_command.positions[i] =
      admittance_state_.joint_command_position[i];
    joint_state_command.velocities[i] =
      admittance_state_.joint_command_velocity[i];
    joint_state_command.accelerations[i] =
      admittance_state_.joint_command_acceleration[i];
  }
  return controller_interface::return_type::OK;
}

bool CartesianAdmittanceRule::update_internal_state(
  const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state,
  const geometry_msgs::msg::Wrench & measured_wrench)
{
  bool success = true;   // return flag

  // Pre-compute commonly used transformations
  success &= kinematics_->calculate_link_transform(
    current_joint_state.positions,
    admittance_state_.ft_sensor_frame,
    admittance_transforms_.base_ft_
  );
  success &= kinematics_->calculate_link_transform(
    current_joint_state.positions,
    parameters_.kinematics.tip,
    admittance_transforms_.base_tip_
  );
  success &= kinematics_->calculate_link_transform(
    current_joint_state.positions,
    parameters_.fixed_world_frame.frame.id,
    admittance_transforms_.world_base_
  );
  success &= kinematics_->calculate_link_transform(
    current_joint_state.positions,
    parameters_.gravity_compensation.frame.id,
    admittance_transforms_.base_cog_
  );
  success &= kinematics_->calculate_link_transform(
    current_joint_state.positions,
    parameters_.control.frame.id,
    admittance_transforms_.base_control_
  );
  success &= kinematics_->calculate_link_transform(
    current_joint_state.positions,
    parameters_.control.frame.id,
    admittance_transforms_.base_admittance_
  );

  // Update current robot joint states
  vec_to_eigen(current_joint_state.positions, admittance_state_.joint_state_position);
  vec_to_eigen(current_joint_state.velocities, admittance_state_.joint_state_velocity);

  // Update current cartesian pose and velocity from robot joint states
  success &= kinematics_->calculate_link_transform(
    current_joint_state.positions,
    admittance_state_.control_frame,
    admittance_state_.robot_current_pose
  );

  success = kinematics_->convert_joint_deltas_to_cartesian_deltas(
    admittance_state_.joint_state_position,
    admittance_state_.joint_state_velocity,
    admittance_state_.control_frame,
    admittance_state_.robot_current_velocity
  );

  // Process wrench measurement
  success &= process_wrench_measurements(measured_wrench);

  return true;
}

bool CartesianAdmittanceRule::process_wrench_measurements(
  const geometry_msgs::msg::Wrench & measured_wrench)
{
  // Extract wrench from msg
  Eigen::Matrix<double, 3, 2, Eigen::ColMajor> new_wrench;
  new_wrench(0, 0) = measured_wrench.force.x;
  new_wrench(1, 0) = measured_wrench.force.y;
  new_wrench(2, 0) = measured_wrench.force.z;
  new_wrench(0, 1) = measured_wrench.torque.x;
  new_wrench(1, 1) = measured_wrench.torque.y;
  new_wrench(2, 1) = measured_wrench.torque.z;

  // F/T measurement w.r.t. world frame
  Eigen::Matrix<double, 3, 3> rot_world_sensor =
    admittance_transforms_.world_base_.rotation() * admittance_transforms_.base_ft_.rotation();
  Eigen::Matrix<double, 3, 3> rot_world_cog =
    admittance_transforms_.world_base_.rotation() * admittance_transforms_.base_cog_.rotation();
  Eigen::Matrix<double, 3, 2, Eigen::ColMajor> new_wrench_world =
    rot_world_sensor * new_wrench;

  // Remove contribution of gravity
  Eigen::Vector3d cog_pos = Eigen::Vector3d(parameters_.gravity_compensation.CoG.pos.data());
  Eigen::Vector3d end_effector_weight = Eigen::Vector3d::Zero();
  end_effector_weight[2] = -parameters_.gravity_compensation.CoG.force;
  new_wrench_world(2, 0) -= end_effector_weight[2];
  new_wrench_world.block<3, 1>(0, 1) -= (rot_world_cog * cog_pos).cross(end_effector_weight);

  /*
  // Wrench at interaction point (e.g., assumed to be control frame
  F_ext.block<3, 1>(0, 0) = rot_base_control.transpose() * F_ext_base.block<3, 1>(0, 0);
  // Evaluate torques at new interaction point
  F_ext.block<3, 1>(3, 0) = rot_base_control.transpose() * (
    F_ext_base.block<3, 1>(0, 0)
    // TODO(tpoignonec): ACTUAL wrench tensor transformation from ft to control frame...
  );
  */
  // Filter measurement
  for (size_t i = 0; i < 6; ++i) {
    wrench_world_(i) = filters::exponentialSmoothing(
      new_wrench_world(i),
      wrench_world_(i),
      parameters_.ft_sensor.filter_coefficient
    );
  }

  // Transform wrench_world_ into base frame
  admittance_state_.robot_current_wrench_at_ft_frame.head(3) =
    admittance_transforms_.world_base_.rotation().transpose() * wrench_world_.head(3);
  admittance_state_.robot_current_wrench_at_ft_frame.tail(3) =
    admittance_transforms_.world_base_.rotation().transpose() * wrench_world_.tail(3);
  /*
  std::cerr << "raw wrench = " << new_wrench.transpose() << std::endl;
  std::cerr << "new_wrench_world = " << new_wrench_world.transpose() << std::endl;
  std::cerr << "filter coef = " << parameters_.ft_sensor.filter_coefficient << std::endl;
  std::cerr << "filtered_wrench_world = " << wrench_world_.transpose() << std::endl;
  std::cerr << "new_wrench_base = " << admittance_state_.robot_current_wrench_at_ft_frame.transpose() << std::endl;
  */
  return true;
}

template<typename T1, typename T2>
void CartesianAdmittanceRule::vec_to_eigen(const std::vector<T1> & data, T2 & matrix)
{
  for (auto col = 0; col < matrix.cols(); col++) {
    for (auto row = 0; row < matrix.rows(); row++) {
      matrix(row, col) = data[row + col * matrix.rows()];
    }
  }
}

}  // namespace cartesian_admittance_controller
