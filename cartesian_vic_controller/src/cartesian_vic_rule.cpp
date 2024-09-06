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

#include "cartesian_vic_controller/cartesian_vic_rule.hpp"

#include <sstream>

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
  logger_(rclcpp::get_logger("cartesian_vic_rule")),
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
  filtered_external_torques_ = Eigen::VectorXd::Zero(num_joints_);
  use_streamed_interaction_parameters_ = false;
  return reset(num_joints_);
}

controller_interface::return_type
CartesianVicRule::configure(
    const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> & parameters_interface,
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
      if (!dynamics_->initialize(parameters_interface, parameters_.dynamics.tip))
      {
        return controller_interface::return_type::ERROR;
      }
    } catch (pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR(
        logger_,
        "Exception while loading the IK plugin '%s': '%s'",
        parameters_.dynamics.plugin_name.c_str(), ex.what()
      );
      return controller_interface::return_type::ERROR;
    }
  } else {
    RCLCPP_ERROR(
      logger_,
      "A differential IK plugin name was not specified in the config file.");
    return controller_interface::return_type::ERROR;
  }

  // Allocate temporary matrices
  J_private_ = \
    Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, num_joints);

  // J_dot_private_ = \
  //  Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, num_joints);

  return controller_interface::return_type::OK;
}


controller_interface::return_type
CartesianVicRule::init_reference_frame_trajectory(
  const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state)
{

  RCLCPP_INFO(logger_, "Initializing the reference frame...");
  // Load parameters
  use_streamed_interaction_parameters_ = false;

  // Update parameters
  apply_parameters_update();

  // Update kinematics state
  if (!update_kinematics(-1.0, current_joint_state)) {
    RCLCPP_ERROR(
      logger_,
      "Failed to update internal state in 'init_reference_frame_trajectory()'!");
    return controller_interface::return_type::ERROR;
  }

  RCLCPP_INFO(logger_, "TAG 3");
  // Reset robot command
  // TODO(tpoignonec): move elsewhere?
  vic_state_.command_data.joint_command_position = vic_state_.input_data.joint_state_position;
  vic_state_.command_data.joint_command_velocity.setZero();
  vic_state_.command_data.joint_command_acceleration.setZero();

  // Reset initial desired robot joint state
  initial_joint_positions_ = vic_state_.input_data.joint_state_position;
  std::stringstream ss_inital_joint_positions;
  ss_inital_joint_positions << initial_joint_positions_.transpose();
  RCLCPP_INFO(
    logger_,
    "Initial joint positions set to : %s",
    ss_inital_joint_positions.str().c_str());

  // Set current pose as cartesian ref
  auto N = vic_state_.input_data.reference_compliant_frames.N();
  Eigen::Matrix<double, 6, 1> null_vector_6D = Eigen::Matrix<double, 6, 1>::Zero();


  RCLCPP_INFO(logger_, "TAG 4");
  bool success = true;
  for (unsigned int i = 0; i < N; i++) {
    // TODO(tpoignonec): Check the frame is correct (i.e., control w.r.t. base)!
    success &= \
      vic_state_.input_data.reference_compliant_frames.fill_desired_desired_robot_state(
      i,
      vic_state_.input_data.robot_current_pose,
      null_vector_6D,
      null_vector_6D,
      null_vector_6D
      );
    if (!success) {
      RCLCPP_ERROR(
        logger_,
        "Failed to fill the desired robot state for index=%u!",
        i);
      return controller_interface::return_type::ERROR;
    }
  }

  // Refresh parameters
  apply_parameters_update();
  RCLCPP_INFO(logger_, "Reference frame has been initialized.");

  return controller_interface::return_type::OK;
}

controller_interface::return_type
CartesianVicRule::reset(const size_t num_joints)
{
  if (control_mode_ == ControlMode::INVALID) {
    RCLCPP_ERROR(logger_, "Invalid control mode!");
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
  vic_state_.input_data.activate_nullspace_control = \
    parameters_.vic.activate_nullspace_control;
  vic_state_.input_data.activate_gravity_compensation = \
    parameters_.vic.activate_gravity_compensation;

  vic_state_.input_data.vic_frame = parameters_.vic.frame.id;
  vic_state_.input_data.end_effector_frame = parameters_.end_effector_frame.frame.id;
  vic_state_.input_data.ft_sensor_frame = parameters_.ft_sensor.frame.id;

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
    vic_state_.input_data.reference_compliant_frames.fill_desired_compliance(
      desired_inertia,
      desired_stiffness,
      desired_damping
    );
  }

  // nullspace control parameters

  double default_nullspace_inertia = 10.0;
  double default_nullspace_stiffness = 0.0;
  double default_nullspace_damping = 1.0;

  for (size_t i = 0; i < num_joints_; i++) {
    // Fill M
    if (parameters_.nullspace_control.joint_inertia.size() == 1) {
      vic_state_.input_data.nullspace_joint_inertia(i) = \
        parameters_.nullspace_control.joint_inertia[0];
    } else if (parameters_.nullspace_control.joint_inertia.size() == num_joints_) {
      vic_state_.input_data.nullspace_joint_inertia(i) = \
        parameters_.nullspace_control.joint_inertia[i];
    } else {
      RCLCPP_ERROR(
        logger_,
        "Invalid size for nullspace_inertia vector!");
      vic_state_.input_data.nullspace_joint_inertia(i) = default_nullspace_inertia;
    }
    // Fill K
    if (parameters_.nullspace_control.joint_stiffness.size() == 1) {
      vic_state_.input_data.nullspace_joint_stiffness(i) = \
        parameters_.nullspace_control.joint_stiffness[0];
    } else if (parameters_.nullspace_control.joint_stiffness.size() == num_joints_) {
      vic_state_.input_data.nullspace_joint_stiffness(i) = \
        parameters_.nullspace_control.joint_stiffness[i];
    } else {
      RCLCPP_ERROR(
        logger_,
        "Invalid size for nullspace_stiffness vector!");
      vic_state_.input_data.nullspace_joint_stiffness(i) = default_nullspace_stiffness;
    }
    // Fill D
    if (parameters_.nullspace_control.joint_damping.size() == 1) {
      vic_state_.input_data.nullspace_joint_damping(i) = \
        parameters_.nullspace_control.joint_damping[0];
    } else if (parameters_.nullspace_control.joint_damping.size() == num_joints_) {
      vic_state_.input_data.nullspace_joint_damping(i) = \
        parameters_.nullspace_control.joint_damping[i];
    } else {
      RCLCPP_ERROR(
        logger_,
        "Invalid size for nullspace_damping vector!");
      vic_state_.input_data.nullspace_joint_damping(i) = default_nullspace_damping;
    }
  }
  // Fill desired joint positions
  if (parameters_.nullspace_control.desired_joint_positions.empty()) {
    vic_state_.input_data.nullspace_desired_joint_positions = \
      initial_joint_positions_;
  } else if (parameters_.nullspace_control.desired_joint_positions.size() == num_joints_) {
    vec_to_eigen(
      parameters_.nullspace_control.desired_joint_positions,
      vic_state_.input_data.nullspace_desired_joint_positions
    );
  } else {
    RCLCPP_ERROR(
      logger_,
      "Invalid size for desired_joint_position vector!");
    vic_state_.input_data.nullspace_desired_joint_positions = initial_joint_positions_;
  }
}

void CartesianVicRule::set_interaction_parameters(
  const Eigen::Matrix<double, 6, 1> & desired_inertia,
  const Eigen::Matrix<double, 6, 1> & desired_stiffness,
  const Eigen::Matrix<double, 6, 1> & desired_damping)
{
  use_streamed_interaction_parameters_ = true;
  vic_state_.input_data.reference_compliant_frames.fill_desired_compliance(
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
  if (vic_state_.input_data.reference_compliant_frames.N() != N) {
    std::cerr \
      << "Warning! 'compliant_frame_trajectory.cartesian_trajectory_points.size() != N'" \
      << " and will be resized..." \
      << std::endl;
    if (!vic_state_.input_data.reference_compliant_frames.resize(N)) {
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
    // TODO(tpoignonec): Check the frame is correct (i.e., end-effector w.r.t. base)!
    success &= \
      vic_state_.input_data.reference_compliant_frames.fill_desired_robot_state_from_msg(
      i,
      compliant_frame_trajectory.cartesian_trajectory_points[i]
      );
    if (use_streamed_interaction_parameters_) {
      success &= \
        vic_state_.input_data.reference_compliant_frames.fill_desired_compliance_from_msg(
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
  if (vic_state_.to_msg(vic_state_msg)) {
    return controller_interface::return_type::OK;
  } else {
    return controller_interface::return_type::ERROR;
  }
}

controller_interface::return_type
CartesianVicRule::update(
  const rclcpp::Duration & period,
  const MeasurementData & measurement_data,
  trajectory_msgs::msg::JointTrajectoryPoint & joint_state_command)
{
  if (update_input_data(period, measurement_data) != controller_interface::return_type::OK) {
    RCLCPP_ERROR(
      logger_,
      "Failed to update VIC input data!"
    );
    // Set commanded position to the previous one
    joint_state_command.positions = measurement_data.get_joint_state().positions;
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
    // Set efforts to zero
    std::fill(
      joint_state_command.effort.begin(),
      joint_state_command.effort.end(),
      0
    );
    return controller_interface::return_type::ERROR;
  }
  return compute_controls(period, joint_state_command);
}

controller_interface::return_type
CartesianVicRule::update_input_data(
  const rclcpp::Duration & period,
  const MeasurementData & measurement_data)
{
  bool success = true;     // return flag
  const double dt = period.seconds();

    // Update parameters
  if (parameters_.enable_parameter_update_without_reactivation) {
    apply_parameters_update();
  }

    // Process F/T sensor data
  bool has_valid_ft_wrench = measurement_data.has_ft_sensor_data();
  if (has_valid_ft_wrench) {
    has_valid_ft_wrench &= process_wrench_measurements(
        period.seconds(), measurement_data.get_ft_sensor_wrench());
  }
  if (!has_valid_ft_wrench) {
    vic_state_.input_data.reset_ft_sensor_wrench();
    if (measurement_data.has_ft_sensor_data()) {
      success = false;
      RCLCPP_WARN_THROTTLE(
          logger_,
          internal_clock_,
          1000,
          "Invalid F/T sensor data provided to VIC rule!");
    }
  }

    // Process external torques
  bool has_valid_external_torques = measurement_data.has_external_torques_data();
  if (has_valid_external_torques) {
    has_valid_external_torques &= process_external_torques_measurements(
        period.seconds(), measurement_data.get_external_torques());
  }
  if (!has_valid_external_torques) {
    vic_state_.input_data.reset_joint_state_external_torques();
    if (measurement_data.has_external_torques_data()) {
      success = false;
      RCLCPP_WARN_THROTTLE(
          logger_,
          internal_clock_,
          1000,
          "Invalid external torques provided to VIC rule!");
    }
  }

  // Update current robot kinematic state
  success &= update_kinematics(
      dt,
      measurement_data.get_joint_state()
  );

  // Retrieve inertia matrices
  // TODO(tpoignonec): move to a "update_dynamics()" function?
  success &= dynamics_->calculate_inertia(
    vic_state_.input_data.joint_state_position,
    vic_state_.input_data.natural_joint_space_inertia
  );
  success &= dynamics_->calculate_jacobian(
    vic_state_.input_data.joint_state_position,
    vic_state_.input_data.end_effector_frame,
    J_private_
  );
  vic_state_.input_data.natural_cartesian_inertia = (J_private_ * \
    vic_state_.input_data.natural_joint_space_inertia.inverse() * \
    J_private_.transpose()).inverse();

  if (!success) {
    RCLCPP_ERROR(
        logger_,
        "update_input_data(): failed to update input data!");
    return controller_interface::return_type::ERROR;
  }
  return controller_interface::return_type::OK;
}

controller_interface::return_type
CartesianVicRule::compute_controls(
  const rclcpp::Duration & period,
  trajectory_msgs::msg::JointTrajectoryPoint & joint_state_command)
{
  bool success = true;   // return flag
  const double dt = period.seconds();
  // Compute controls
  vic_state_.diagnostic_data["period"] = dt;  // log ros-control period
  success &= compute_controls(dt, vic_state_.input_data, vic_state_.command_data);

  // If an error is detected, set commanded velocity to zero
  if (!success) {
    RCLCPP_ERROR(
      logger_,
      "Failed to compute the controls!"
    );
    // Set commanded position to the previous one
    for (size_t i = 0; i < parameters_.joints.size(); ++i) {
      joint_state_command.positions[i] = vic_state_.input_data.joint_state_position(i);
    }

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

    // Set efforts to zero
    std::fill(
      joint_state_command.effort.begin(),
      joint_state_command.effort.end(),
      0
    );
    // TODO(tpoignonec): set effort to zero only if direct effort control
    // is enabled. Otherwise, the robot might fall down!!!
    //
    // On the Kuka iiwa and Franka it's OK, but still...
    // A fall-back strategy should be implemented for impedance control.
    // For instance, one could implement joint impedance control to maintain the robot in place.
    // Or, we could just set the effort to NaN and let the controller take care of it.
    // But that is a bit dangerous, as the controller might not be able to react properly...

    return controller_interface::return_type::ERROR;
  }
  // Otherwise, set joint command and return
  for (size_t i = 0; i < parameters_.joints.size(); ++i) {
    if (vic_state_.command_data.has_position_command) {
      joint_state_command.positions[i] =
        vic_state_.command_data.joint_command_position[i];
    }

    if (vic_state_.command_data.has_velocity_command) {
      joint_state_command.velocities.resize(num_joints_);
      joint_state_command.velocities[i] =
        vic_state_.command_data.joint_command_velocity[i];
    }

    if (vic_state_.command_data.has_acceleration_command) {
      joint_state_command.accelerations.resize(num_joints_);
      joint_state_command.accelerations[i] =
        vic_state_.command_data.joint_command_acceleration[i];
    }

    if (vic_state_.command_data.has_effort_command) {
      joint_state_command.effort.resize(num_joints_);
      joint_state_command.effort[i] =
        vic_state_.command_data.joint_command_effort[i];
    } else {
      joint_state_command.effort.clear();
    }
  }
  return controller_interface::return_type::OK;
}


controller_interface::return_type
CartesianVicRule::compute_controls(
  const rclcpp::Duration & period,
  geometry_msgs::msg::Twist & twist_command)
{
  bool success = true;   // return flag
  const double dt = period.seconds();
  // Compute controls
  success &= compute_controls(dt, vic_state_.input_data, vic_state_.command_data);

  // If an error is detected, set commanded velocity to zero
  if (!success) {
    RCLCPP_ERROR(
      logger_,
      "Failed to compute the controls!"
    );
    // Set twist command to zero
    twist_command = geometry_msgs::msg::Twist();
    return controller_interface::return_type::ERROR;
  }
  // Otherwise, return computed twist command
  if (vic_state_.command_data.has_twist_command) {
    twist_command.linear.x = vic_state_.command_data.twist_command[0];
    twist_command.linear.y = vic_state_.command_data.twist_command[1];
    twist_command.linear.z = vic_state_.command_data.twist_command[2];
    twist_command.angular.x = vic_state_.command_data.twist_command[3];
    twist_command.angular.y = vic_state_.command_data.twist_command[4];
    twist_command.angular.z = vic_state_.command_data.twist_command[5];
    return controller_interface::return_type::OK;
  } else {
    // Set twist command to zero
    twist_command = geometry_msgs::msg::Twist();
    RCLCPP_ERROR(
      logger_,
      "Rule plugin did not return computed twist command!");
    return controller_interface::return_type::ERROR;
  }
}

// Internal functions

bool CartesianVicRule::update_kinematics(
  double dt,
  const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state)
{
  bool success = true;   // return flag


  // Update current robot joint position
  double cutoff_jnt_position = parameters_.filters.state_position_filter_cuttoff_freq;
  if (dt > 0 && cutoff_jnt_position > 0.0) {
    double jnt_position_filter_coefficient = 1.0 - exp(-dt * 2 * 3.14 * cutoff_jnt_position);
    for (size_t i = 0; i < num_joints_; ++i) {
      vic_state_.input_data.joint_state_position(i) = filters::exponentialSmoothing(
        current_joint_state.positions.at(i),
        vic_state_.input_data.joint_state_position(i),
        jnt_position_filter_coefficient
      );
    }
  } else {
    // Initialization
    vec_to_eigen(current_joint_state.positions, vic_state_.input_data.joint_state_position);
  }

  // Update current robot joint velocity
  // auto previous_joint_velocity = vic_state_.input_data.joint_state_velocity;
  double cutoff_jnt_velocity = parameters_.filters.state_velocity_filter_cuttoff_freq;
  if (dt > 0 && cutoff_jnt_velocity > 0.0) {
    double jnt_velocity_filter_coefficient = 1.0 - exp(-dt * 2 * 3.14 * cutoff_jnt_velocity);
    for (size_t i = 0; i < num_joints_; ++i) {
      vic_state_.input_data.joint_state_velocity(i) = filters::exponentialSmoothing(
        current_joint_state.velocities.at(i),
        vic_state_.input_data.joint_state_velocity(i),
        jnt_velocity_filter_coefficient
      );
    }
  } else {
    // Initialization
    vec_to_eigen(current_joint_state.velocities, vic_state_.input_data.joint_state_velocity);
  }

  if (!dynamics_) {
    RCLCPP_ERROR(
      logger_,
      "Dynamics plugin not initialized!");
    return false;
  }
  // Update current cartesian pose and velocity from robot joint states
  success &= dynamics_->calculate_link_transform(
    vic_state_.input_data.joint_state_position,
    vic_state_.input_data.end_effector_frame,
    vic_state_.input_data.robot_current_pose
  );

  auto last_robot_cartesian_velocity = vic_state_.input_data.robot_current_velocity;
  success = dynamics_->convert_joint_deltas_to_cartesian_deltas(
    vic_state_.input_data.joint_state_position,
    vic_state_.input_data.joint_state_velocity,
    vic_state_.input_data.end_effector_frame,
    vic_state_.input_data.robot_current_velocity
  );

  // Estimation acceleration (finite diff.)
  double cutoff_acceleration = parameters_.filters.state_acceleration_filter_cuttoff_freq;
  if (dt > 0) {
    auto raw_acc = \
      (vic_state_.input_data.robot_current_velocity - last_robot_cartesian_velocity) / dt;
    /*
    auto joint_acc = (vic_state_.input_data.joint_state_velocity - previous_joint_velocity) / dt;
    success &= dynamics_->calculate_jacobian_derivative(
      vic_state_.input_data.joint_state_position,
      vic_state_.input_data.joint_state_velocity,
      vic_state_.input_data.end_effector_frame,
      J_dot_private_
    );
    auto raw_acc = J_private_ * joint_acc + J_dot_private_ *
      vic_state_.input_data.joint_state_velocity;
    */
    if (cutoff_acceleration > 0.0) {
      double cutoff_acceleration = 30.0;  // Hz
      double acceleration_filter_coefficient = 1.0 - exp(-dt * 2 * 3.14 * cutoff_acceleration);
      for (size_t i = 0; i < 6; ++i) {
        vic_state_.input_data.robot_estimated_acceleration(i) = filters::exponentialSmoothing(
          raw_acc(i),
          vic_state_.input_data.robot_estimated_acceleration(i),
          acceleration_filter_coefficient
        );
      }
    } else {
      vic_state_.input_data.robot_estimated_acceleration = raw_acc;
    }
  } else {
    vic_state_.input_data.robot_estimated_acceleration.setZero();
  }

  // Pre-compute commonly used transformations
  if (parameters_.ft_sensor.is_enabled) {
    success &= dynamics_->calculate_link_transform(
      vic_state_.input_data.joint_state_position,
      vic_state_.input_data.ft_sensor_frame,
      vic_transforms_.base_ft_
    );
  } else {
    // TODO(tpoignonec): how to make sure it is not used in that case ?
    // In theory, there is no reason to use it if no wrench is available, but you never know...
    vic_transforms_.base_ft_.setIdentity();
  }
  success &= dynamics_->calculate_link_transform(
    vic_state_.input_data.joint_state_position,
    parameters_.fixed_world_frame.frame.id,
    vic_transforms_.world_base_
  );
  success &= dynamics_->calculate_link_transform(
    vic_state_.input_data.joint_state_position,
    parameters_.gravity_compensation.frame.id,
    vic_transforms_.base_cog_
  );
  success &= dynamics_->calculate_link_transform(
    vic_state_.input_data.joint_state_position,
    parameters_.end_effector_frame.frame.id,
    vic_transforms_.base_end_effector_
  );
  success &= dynamics_->calculate_link_transform(
    vic_state_.input_data.joint_state_position,
    parameters_.vic.frame.id,
    vic_transforms_.base_vic_
  );
  return true;
}

bool CartesianVicRule::process_wrench_measurements(
  double dt, const geometry_msgs::msg::Wrench & measured_wrench)
{
  // TODO(tpoignonec): use ft_tools_ros2 package!!!

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
  // Wrench at interaction point (assumed to be end-effector frame)
  F_ext.block<3, 1>(0, 0) = rot_base_control.transpose() * F_ext_base.block<3, 1>(0, 0);
  // Evaluate torques at new interaction point
  F_ext.block<3, 1>(3, 0) = rot_base_control.transpose() * (
    F_ext_base.block<3, 1>(0, 0)
    // TODO(tpoignonec): ACTUAL wrench tensor transformation from ft to control frame...
  );
  */

  // Filter measurement
  double cutoff_ft = parameters_.filters.ft_sensor_filter_cuttoff_freq;
  if (dt > 0.0 && cutoff_ft > 0.0) {
    double ft_filter_coefficient = 1.0 - exp(-dt * 2 * 3.14 * cutoff_ft);
    for (size_t i = 0; i < 6; ++i) {
      wrench_world_(i) = filters::exponentialSmoothing(
        new_wrench_world(i),
        wrench_world_(i),
        ft_filter_coefficient);
    }
  } else {
    // Initialization
    for (size_t i = 0; i < 6; ++i) {
      wrench_world_(i) = new_wrench_world(i);
    }
  }
  // Transform wrench_world_ into base frame
  wrench_base_.head(3) = \
    vic_transforms_.world_base_.rotation().transpose() * wrench_world_.head(3);
  wrench_base_.tail(3) = \
    vic_transforms_.world_base_.rotation().transpose() * wrench_world_.tail(3);
  vic_state_.input_data.set_ft_sensor_wrench(wrench_base_);

  return true;
}

bool CartesianVicRule::process_external_torques_measurements(
  double dt /*period in seconds*/,
  const std::vector<double> & measured_external_torques)
{
  // Check data validity
  if (static_cast<size_t>(measured_external_torques.size()) != num_joints_) {
    RCLCPP_ERROR(
      logger_,
      "Invalid size for measured_external_torques vector!");
    filtered_external_torques_.setZero();
    vic_state_.input_data.set_joint_state_external_torques(filtered_external_torques_);
    vic_state_.input_data.reset_joint_state_external_torques();
    return false;
  }

  // Filter measurement
  double cutoff_freq = parameters_.filters.external_torque_sensor_filter_cuttoff_freq;
  if (dt > 0.0 && cutoff_freq > 0.0) {
    double ext_torques_filter_coefficient = 1.0 - exp(-dt * 2 * 3.14 * cutoff_freq);
    for (size_t i = 0; i < 6; ++i) {
      filtered_external_torques_(i) = \
        filters::exponentialSmoothing(
        measured_external_torques[i],
        filtered_external_torques_(i),
        ext_torques_filter_coefficient);
    }
  } else {
    // Initialization
    for (size_t i = 0; i < 6; ++i) {
      filtered_external_torques_(i) = \
        measured_external_torques[i];
    }
  }
  vic_state_.input_data.set_joint_state_external_torques(filtered_external_torques_);
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
