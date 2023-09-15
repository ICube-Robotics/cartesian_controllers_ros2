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
/// \authors: Thibault Poignonec, Maciej Bednarczyk

#ifndef CARTESIAN_ADMITTANCE_CONTROLLER__CARTESIAN_ADMITTANCE_SOLVER_HPP_
#define CARTESIAN_ADMITTANCE_CONTROLLER__CARTESIAN_ADMITTANCE_SOLVER_HPP_

#include <map>
#include <memory>
#include <vector>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "controller_interface/controller_interface.hpp"
#include "kinematics_interface/kinematics_interface.hpp"
#include "pluginlib/class_loader.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_kdl/tf2_kdl.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

// msgs
#include "control_msgs/msg/admittance_controller_state.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
// Custom msgs
#include "cartesian_control_msgs/msg/cartesian_trajectory.hpp"

// include generated parameter library
#include "cartesian_admittance_controller_parameters.hpp"

namespace cartesian_admittance_controller
{
struct AdmittanceState
{
  explicit AdmittanceState(size_t num_joints)
  {
    // Allocate joint state
    joint_state_position = Eigen::VectorXd::Zero(num_joints);
    joint_state_velocity = Eigen::VectorXd::Zero(num_joints);

    // Allocate and reset command
    robot_command_twist.setZero();
    joint_command_position = Eigen::VectorXd::Zero(num_joints);
    joint_command_velocity = Eigen::VectorXd::Zero(num_joints);
    joint_command_acceleration = Eigen::VectorXd::Zero(num_joints);

    // Reset pre-computed values
    pose_tracking_error.setZero();
    velocity_tracking_error.setZero();

  }
    bool is_configured = false;
    // General parameters
    //------------------------
    //std::string base_frame;
    //std::string control_frame;
    std::string ft_sensor_frame;

    // Interaction parameters
    //------------------------
    Eigen::Matrix<double, 6, 6> inertia;
    Eigen::Matrix<double, 6, 6> stiffness;
    Eigen::Matrix<double, 6, 6> damping;

    // Measured state
    //------------------------
    Eigen::VectorXd joint_state_position;
    Eigen::VectorXd joint_state_velocity;

    // Cartesian measurements w.r.t. world frame
    Eigen::Isometry3d robot_current_pose;
    Eigen::Matrix<double, 6, 1> robot_current_velocity;
    Eigen::Matrix<double, 6, 1> robot_current_wrench;

    // Computed errors
    //------------------------
    /// Pose tracking error e = p_desired - p_measured
    Eigen::Matrix<double, 6, 1> pose_tracking_error;
    /// Velocity tracking error e_dot = p_dot_desired - p_dot_measured
    Eigen::Matrix<double, 6, 1> velocity_tracking_error;

    // Computed command
    //------------------------
    Eigen::Matrix<double, 6, 1> robot_command_twist;
    Eigen::VectorXd joint_command_position;
    Eigen::VectorXd joint_command_velocity;
    Eigen::VectorXd joint_command_acceleration;

    // Pre-computed frames
    //------------------------

    //TODO(tpoignonec): as needed...

};

class CartesianAdmittanceSolver
{
public:
    explicit CartesianAdmittanceSolver(
        const std::shared_ptr<cartesian_admittance_controller::ParamListener> & parameter_handler);

    /// Configure admittance solver
    controller_interface::return_type configure(
        const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node,
        const size_t num_joint
    );

    /// Reset all values back to default
    controller_interface::return_type reset(const size_t num_joints);

    /// Retrieve parameters and update if applicable
    void apply_parameters_update();

    /// Manual setting of inertia, damping, and stiffness
    void set_interaction_parameters(
        const Eigen::Matrix<double, 6, 6> & desired_inertia,
        const Eigen::Matrix<double, 6, 6> & desired_stiffness,
        const Eigen::Matrix<double, 6, 6> & desired_damping
    );

    /**
    * Compute joint (velocity) command from the current cartesian tracking errors
    * and the desired interaction parameters (M, K, D).
    *
    * \param[in] current_joint_state current joint state of the robot
    * \param[in] measured_wrench most recent measured wrench from force torque sensor
    * \param[in] cartesian_reference cartesian reference (pose, twist, and acc.)
    * \param[in] period time in seconds since last controller update
    * \param[out] joint_state_command computed joint state command
    */
    controller_interface::return_type update(
        const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state,
        const geometry_msgs::msg::Wrench & measured_wrench,
        const cartesian_control_msgs::msg::CartesianTrajectoryPoint & cartesian_reference,
        const rclcpp::Duration & period,
        trajectory_msgs::msg::JointTrajectoryPoint & joint_state_command
    );

protected:
    bool update_internal_state(
        const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state,
        const geometry_msgs::msg::Wrench & measured_wrench,
        const cartesian_control_msgs::msg::CartesianTrajectoryPoint & cartesian_reference);

    bool process_wrench_measurements(
        const geometry_msgs::msg::Wrench & measured_wrench,
        const Eigen::Matrix<double, 3, 3> & ft_sensor_world_rot,
        const Eigen::Matrix<double, 3, 3> & com_world_rot);

    /// Actual admittance control logic
    bool compute_controls(
        AdmittanceState & amittance_state,
        double dt/*period in seconds*/)
    {
        // TODO: make method virtual
        return true;
    };

public:
  // Parameters management
  std::shared_ptr<cartesian_admittance_controller::ParamListener> parameter_handler_;
  cartesian_admittance_controller::Params parameters_;

  // Admittance controllers internal state
  AdmittanceState admittance_state_{0};

  /// Number of robot joints
  size_t num_joints_;

  /// If true, the parameters values are ignored (use "set_interaction_parameters()" instead)
  bool use_streamed_interaction_parameters_ = false;

  // Kinematics interface plugin loader
  std::shared_ptr<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>
    kinematics_loader_;
  /// Kinematics interface
  std::unique_ptr<kinematics_interface::KinematicsInterface> kinematics_;

  /// Filtered wrench expressed in world frame
  Eigen::Matrix<double, 6, 1> wrench_world_;

};

}  // namespace cartesian_admittance_controller

#endif  // CARTESIAN_ADMITTANCE_CONTROLLER__CARTESIAN_ADMITTANCE_RULE_HPP_