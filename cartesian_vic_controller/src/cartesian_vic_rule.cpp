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

#include "cartesian_vic_controller/cartesian_vic_rule.hpp"
#include "cartesian_vic_controller/utils.hpp"

#include "dynamics_interface/dynamics_interface.hpp"

#include "rclcpp/duration.hpp"
#include "control_toolbox/filters.hpp"

#include "pluginlib/class_loader.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_kdl/tf2_kdl.hpp"

namespace cartesian_vic_controller
{

CartesianVicRule::CartesianVicRule()
: num_joints_(0),
  vic_state_(0, ControlMode::INVALID),
  dynamics_loader_(nullptr)
{
  // Nothing to do, see init().
}

controller_interface::return_type
CartesianVicRule::init(
  const std::shared_ptr<cartesian_vic_controller::ParamListener> & parameter_handler)
{
  parameter_handler_ = parameter_handler;
  parameters_ = parameter_handler_->get_params();
  num_joints_ = parameters_.joints.size();
  wrench_world_.setZero();
  use_streamed_interaction_parameters_ = false;
  return reset(num_joints_);
}

controller_interface::return_type
CartesianVicRule::configure(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node,
  const size_t num_joints)
{
  num_joints_ = num_joints;
  // reset vic state
  reset(num_joints);
  // Load the dynamics (also used for IK) plugin
  if (!parameters_.dynamics.plugin_name.empty()) {
    try {
      dynamics_loader_ =
        std::make_shared<pluginlib::ClassLoader<dynamics_interface::DynamicsInterface>>(
        parameters_.dynamics.plugin_package,
        "dynamics_interface::DynamicsInterface");
      dynamics_ = std::unique_ptr<dynamics_interface::DynamicsInterface>(
        dynamics_loader_->createUnmanagedInstance(parameters_.dynamics.plugin_name));
      if (!dynamics_->initialize(
          node->get_node_parameters_interface(), parameters_.dynamics.tip))
      {
        return controller_interface::return_type::ERROR;
      }
    } catch (pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR(
        rclcpp::get_logger("CartesianVicRule"),
        "Exception while loading the IK plugin '%s': '%s'",
        parameters_.dynamics.plugin_name.c_str(), ex.what()
      );
      return controller_interface::return_type::ERROR;
    }
  } else {
    RCLCPP_ERROR(
      rclcpp::get_logger("CartesianVicRule"),
      "A differential IK plugin name was not specified in the config file.");
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}


controller_interface::return_type
CartesianVicRule::init_reference_frame_trajectory(
  const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state)
{
  // Load parameters
  use_streamed_interaction_parameters_ = false;
  apply_parameters_update();

  // Assume force is zero
  geometry_msgs::msg::Wrench dummy_wrench;

  // Update state
  if (!update_internal_state(current_joint_state, dummy_wrench, -1.0)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("CartesianVicRule"),
      "Failed to update internal state in 'init_reference_frame_trajectory()'!");
    return controller_interface::return_type::ERROR;
  }

  // Reset robot command
  // TODO(tpoignonec): move elsewhere?
  vic_state_.joint_command_position = vic_state_.joint_state_position;
  vic_state_.joint_command_velocity = vic_state_.joint_state_velocity;

  // Set current pose as cartesian ref
  auto N = vic_state_.reference_compliant_frames.N();
  Eigen::Matrix<double, 6, 1> null_vector_6D = Eigen::Matrix<double, 6, 1>::Zero();

  bool success = true;
  for (unsigned int i = 0; i < N; i++) {
    // TODO(tpoignonec): Check the frame is correct (i.e., control w.r.t. base)!
    success &= vic_state_.reference_compliant_frames.fill_desired_desired_robot_state(
      i,
      vic_state_.robot_current_pose,
      null_vector_6D,
      null_vector_6D,
      null_vector_6D
    );
    if (!success) {
      RCLCPP_ERROR(
        rclcpp::get_logger("CartesianVicRule"),
        "Failed to fill the desired robot state for index=%u!",
        i);
      return controller_interface::return_type::ERROR;
    }
  }
  return controller_interface::return_type::OK;
}

controller_interface::return_type
CartesianVicRule::reset(const size_t num_joints)
{
  if (control_mode_ == ControlMode::INVALID) {
    RCLCPP_ERROR(rclcpp::get_logger("CartesianVicRule"), "Invalid control mode!");
    return controller_interface::return_type::ERROR;
  }
  // Reset vic state
  use_streamed_interaction_parameters_ = false;
  vic_state_ = VicState(num_joints, control_mode_);

  // Load parameters
  apply_parameters_update();

  return controller_interface::return_type::OK;
}


void CartesianVicRule::apply_parameters_update()
{
  if (parameter_handler_->is_old(parameters_)) {
    parameters_ = parameter_handler_->get_params();
  }
  // update param values
  vic_state_.vic_frame = parameters_.vic.frame.id;
  vic_state_.control_frame = parameters_.control.frame.id;
  vic_state_.ft_sensor_frame = parameters_.ft_sensor.frame.id;

  if (!use_streamed_interaction_parameters_) {
    Eigen::Matrix<double, 6, 1> desired_inertia =
      Eigen::Matrix<double, 6, 1>(parameters_.vic.inertia.data());
    Eigen::Matrix<double, 6, 1> desired_stiffness =
      Eigen::Matrix<double, 6, 1>(parameters_.vic.stiffness.data());
    Eigen::Matrix<double, 6, 1> desired_damping;
    for (size_t i = 0; i < 6; ++i) {
      // Compute damping from damping ratio
      desired_damping[i] = parameters_.vic.damping_ratio[i] * \
        2 * sqrt(desired_inertia[i] * desired_stiffness[i]);
    }
    vic_state_.reference_compliant_frames.fill_desired_compliance(
      desired_inertia,
      desired_stiffness,
      desired_damping
    );
  }
}

void CartesianVicRule::set_interaction_parameters(
  const Eigen::Matrix<double, 6, 1> & desired_inertia,
  const Eigen::Matrix<double, 6, 1> & desired_stiffness,
  const Eigen::Matrix<double, 6, 1> & desired_damping)
{
  use_streamed_interaction_parameters_ = true;
  vic_state_.reference_compliant_frames.fill_desired_compliance(
    desired_inertia,
    desired_stiffness,
    desired_damping);
}


controller_interface::return_type
CartesianVicRule::update_compliant_frame_trajectory(
  const cartesian_control_msgs::msg::CompliantFrameTrajectory & compliant_frame_trajectory)
{
  // Check cartesian ref trajectory validity
  auto N = compliant_frame_trajectory.cartesian_trajectory_points.size();
  if (vic_state_.reference_compliant_frames.N() != N) {
    std::cerr \
      << "Warning! 'compliant_frame_trajectory.cartesian_trajectory_points.size() != N'" \
      << " and will be resized..." \
      << std::endl;
    if (!vic_state_.reference_compliant_frames.resize(N)) {
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
    success &= vic_state_.reference_compliant_frames.fill_desired_robot_state_from_msg(
      i,
      compliant_frame_trajectory.cartesian_trajectory_points[i]
    );
    if (use_streamed_interaction_parameters_) {
      success &= vic_state_.reference_compliant_frames.fill_desired_compliance_from_msg(
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
CartesianVicRule::controller_state_to_msg(
  cartesian_control_msgs::msg::VicControllerState & vic_state_msg)
{
  bool success = true;
  // Fill desired compliance
  auto desired_frame_0 = \
    vic_state_.reference_compliant_frames.get_compliant_frame(0);
  vic_state_msg.desired_pose = Eigen::toMsg(desired_frame_0.pose);
  vic_state_msg.desired_velocity = Eigen::toMsg(desired_frame_0.velocity);
  vic_state_msg.desired_acceleration = AccelToMsg(desired_frame_0.acceleration);
  matrixEigenToMsg(desired_frame_0.inertia, vic_state_msg.desired_inertia);
  matrixEigenToMsg(desired_frame_0.stiffness, vic_state_msg.desired_stiffness);
  matrixEigenToMsg(desired_frame_0.damping, vic_state_msg.desired_damping);

  // Fill robot state
  vic_state_msg.pose = Eigen::toMsg(vic_state_.robot_current_pose);
  vic_state_msg.velocity = Eigen::toMsg(vic_state_.robot_current_velocity);
  vic_state_msg.wrench = WrenchToMsg(vic_state_.robot_current_wrench_at_ft_frame);
  matrixEigenToMsg(vic_state_.inertia, vic_state_msg.rendered_inertia);
  matrixEigenToMsg(vic_state_.stiffness, vic_state_msg.rendered_stiffness);
  matrixEigenToMsg(vic_state_.damping, vic_state_msg.rendered_damping);

  // Fill commands
  if (vic_state_.control_mode == ControlMode::ADMITTANCE) {
    vic_state_msg.control_mode.data = "admittance";

    vic_state_msg.robot_command_twist = Eigen::toMsg(vic_state_.robot_command_twist);
    vic_state_msg.joint_command_position.resize(num_joints_);
    vic_state_msg.joint_command_velocity.resize(num_joints_);
    vic_state_msg.joint_command_acceleration.resize(num_joints_);
    vic_state_msg.joint_command_effort.clear();

    for (size_t i = 0; i < num_joints_; i++) {
      vic_state_msg.joint_command_position[i] = vic_state_.joint_command_position[i];
      vic_state_msg.joint_command_velocity[i] = vic_state_.joint_command_velocity[i];
      vic_state_msg.joint_command_acceleration[i] =
        vic_state_.joint_command_acceleration[i];
    }
  } else if (vic_state_.control_mode == ControlMode::IMPEDANCE) {
    vic_state_msg.control_mode.data = "impedance";

    vic_state_msg.joint_command_position.clear();
    vic_state_msg.joint_command_velocity.clear();
    vic_state_msg.joint_command_acceleration.clear();
    vic_state_msg.joint_command_effort.resize(num_joints_);
  } else {
    success = false;
    std::cerr << "Error! Unknown control mode!" << std::endl;

    vic_state_msg.joint_command_position.clear();
    vic_state_msg.joint_command_velocity.clear();
    vic_state_msg.joint_command_acceleration.clear();
    vic_state_msg.joint_command_effort.clear();
  }

  // Fill diagnostic data
  matrixEigenToMsg(vic_state_.natural_inertia, vic_state_msg.natural_inertia);
  vic_state_msg.diagnostic_data.keys.clear();
  vic_state_msg.diagnostic_data.values.clear();

  for (const auto & [key, value] : vic_state_.diagnostic_data) {
    vic_state_msg.diagnostic_data.keys.push_back(key);
    vic_state_msg.diagnostic_data.values.push_back(value);
  }

  // Return
  if (success) {
    return controller_interface::return_type::OK;
  } else {
    return controller_interface::return_type::ERROR;
  }
}


controller_interface::return_type
CartesianVicRule::update(
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
    measured_wrench,
    dt
  );

  // Compute controls
  success &= compute_controls(vic_state_, dt);

  // If an error is detected, set commanded velocity to zero
  if (!success) {
    RCLCPP_ERROR(
      rclcpp::get_logger("CartesianVicRule"),
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
      vic_state_.joint_command_position[i];
    joint_state_command.velocities[i] =
      vic_state_.joint_command_velocity[i];
    joint_state_command.accelerations[i] =
      vic_state_.joint_command_acceleration[i];
  }
  return controller_interface::return_type::OK;
}

bool CartesianVicRule::update_internal_state(
  const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state,
  const geometry_msgs::msg::Wrench & measured_wrench,
  double dt)
{
  bool success = true;   // return flag

  // Pre-compute commonly used transformations
  success &= dynamics_->calculate_link_transform(
    current_joint_state.positions,
    vic_state_.ft_sensor_frame,
    vic_transforms_.base_ft_
  );
  success &= dynamics_->calculate_link_transform(
    current_joint_state.positions,
    parameters_.dynamics.tip,
    vic_transforms_.base_tip_
  );
  success &= dynamics_->calculate_link_transform(
    current_joint_state.positions,
    parameters_.fixed_world_frame.frame.id,
    vic_transforms_.world_base_
  );
  success &= dynamics_->calculate_link_transform(
    current_joint_state.positions,
    parameters_.gravity_compensation.frame.id,
    vic_transforms_.base_cog_
  );
  success &= dynamics_->calculate_link_transform(
    current_joint_state.positions,
    parameters_.control.frame.id,
    vic_transforms_.base_control_
  );
  success &= dynamics_->calculate_link_transform(
    current_joint_state.positions,
    parameters_.control.frame.id,
    vic_transforms_.base_vic_
  );

  // Update current robot joint states

  // Filter velocity measurement and copy to state
  double cutoff_jnt_state = parameters_.filters.state_filter_cuttoff_freq;
  if (dt > 0 && cutoff_jnt_state > 0.0) {
    double jnt_state_filter_coefficient = 1.0 - exp(-dt * 2 * 3.14 * cutoff_jnt_state);
    for (size_t i = 0; i < num_joints_; ++i) {
      vic_state_.joint_state_position(i) = filters::exponentialSmoothing(
        current_joint_state.positions.at(i),
        vic_state_.joint_state_position(i),
        jnt_state_filter_coefficient
      );
      vic_state_.joint_state_velocity(i) = filters::exponentialSmoothing(
        current_joint_state.velocities.at(i),
        vic_state_.joint_state_velocity(i),
        jnt_state_filter_coefficient
      );
    }
  } else {
    // Initialization
    vec_to_eigen(current_joint_state.positions, vic_state_.joint_state_position);
    vec_to_eigen(current_joint_state.velocities, vic_state_.joint_state_velocity);
  }

  // Update current cartesian pose and velocity from robot joint states
  success &= dynamics_->calculate_link_transform(
    vic_state_.joint_state_position,
    vic_state_.control_frame,
    vic_state_.robot_current_pose
  );

  success = dynamics_->convert_joint_deltas_to_cartesian_deltas(
    vic_state_.joint_state_position,
    vic_state_.joint_state_velocity,
    vic_state_.control_frame,
    vic_state_.robot_current_velocity
  );

  // Process wrench measurement
  success &= process_wrench_measurements(measured_wrench);

  return true;
}

bool CartesianVicRule::process_wrench_measurements(
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
    vic_transforms_.world_base_.rotation() * vic_transforms_.base_ft_.rotation();
  Eigen::Matrix<double, 3, 3> rot_world_cog =
    vic_transforms_.world_base_.rotation() * vic_transforms_.base_cog_.rotation();
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
  vic_state_.robot_current_wrench_at_ft_frame.head(3) =
    vic_transforms_.world_base_.rotation().transpose() * wrench_world_.head(3);
  vic_state_.robot_current_wrench_at_ft_frame.tail(3) =
    vic_transforms_.world_base_.rotation().transpose() * wrench_world_.tail(3);
  /*
  std::cerr << "raw wrench = " << new_wrench.transpose() << std::endl;
  std::cerr << "new_wrench_world = " << new_wrench_world.transpose() << std::endl;
  std::cerr << "filter coef = " << parameters_.ft_sensor.filter_coefficient << std::endl;
  std::cerr << "filtered_wrench_world = " << wrench_world_.transpose() << std::endl;
  std::cerr << "new_wrench_base = " << vic_state_.robot_current_wrench_at_ft_frame.transpose() << std::endl;
  */
  return true;
}

template<typename T1, typename T2>
void CartesianVicRule::vec_to_eigen(const std::vector<T1> & data, T2 & matrix)
{
  for (auto col = 0; col < matrix.cols(); col++) {
    for (auto row = 0; row < matrix.rows(); row++) {
      matrix(row, col) = data[row + col * matrix.rows()];
    }
  }
}

}  // namespace cartesian_vic_controller
