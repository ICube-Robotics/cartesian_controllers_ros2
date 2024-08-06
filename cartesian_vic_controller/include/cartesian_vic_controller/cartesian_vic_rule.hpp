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

#ifndef CARTESIAN_VIC_CONTROLLER__CARTESIAN_VIC_RULE_HPP_
#define CARTESIAN_VIC_CONTROLLER__CARTESIAN_VIC_RULE_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "dynamics_interface/dynamics_interface.hpp"
#include "pluginlib/class_loader.hpp"

// msgs
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

// custom msgs
#include "cartesian_control_msgs/msg/vic_controller_state.hpp"
#include "cartesian_control_msgs/msg/cartesian_trajectory.hpp"
#include "cartesian_control_msgs/msg/compliant_frame_trajectory.hpp"

// include generated parameter library
#include "cartesian_vic_controller_parameters.hpp"

// include data structures
#include "cartesian_vic_controller/measurement_data.hpp"
#include "cartesian_vic_controller/cartesian_vic_state.hpp"
#include "cartesian_vic_controller/compliance_frame_trajectory.hpp"

namespace cartesian_vic_controller
{

class CartesianVicRule
{
public:
  CartesianVicRule();
  virtual ~CartesianVicRule() = default;

  /// Initialize vic rule
  virtual controller_interface::return_type init(
    const std::shared_ptr<cartesian_vic_controller::ParamListener> & parameter_handler);

  /// Configure vic rule
  virtual controller_interface::return_type configure(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node,
    const size_t num_joint);

  /// Soft reset vic rule (set cartesian ref as current pose)
  controller_interface::return_type init_reference_frame_trajectory(
    const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state);

  /// Reset all values back to default
  virtual controller_interface::return_type reset(const size_t num_joints);

  /// Retrieve parameters and update if applicable
  void apply_parameters_update();

  controller_interface::return_type update_compliant_frame_trajectory(
    const cartesian_control_msgs::msg::CompliantFrameTrajectory & compliant_frame_trajectory);

  controller_interface::return_type controller_state_to_msg(
    cartesian_control_msgs::msg::VicControllerState & msg);

  /**
  * Compute joint command from the current cartesian tracking errors
  * and the desired interaction parameters (M, K, D). This function is
  * to be used when no external torque sensor is available.
  *
  * \param[in] period time in seconds since last controller update
  * \param[in] measurement_data most recent measurement data, including at
  * least the joint position and velocity
  * \param[out] joint_state_command computed joint state command
  */
  controller_interface::return_type update(
    const rclcpp::Duration & period,
    const MeasurementData & measurement_data,
    trajectory_msgs::msg::JointTrajectoryPoint & joint_state_command
  );

  /// Get current control mode (ADMITTANCE / IMPEDANCE / INVALID)
  ControlMode get_control_mode() const {return control_mode_;}

protected:
  /// Manual setting of inertia, damping, and stiffness (diagonal matrices)
  void set_interaction_parameters(
    const Eigen::Matrix<double, 6, 1> & desired_inertia,
    const Eigen::Matrix<double, 6, 1> & desired_stiffness,
    const Eigen::Matrix<double, 6, 1> & desired_damping
  );

  bool update_kinematics(
    double dt /*period in seconds*/,
    const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state);

  bool process_wrench_measurements(
    double dt /*period in seconds*/,
    const geometry_msgs::msg::Wrench & measured_wrench);

  bool process_external_torques_measurements(
    double dt /*period in seconds*/,
    const std::vector<double> & measured_external_torques);

  /// Actual vic control logic
  virtual bool compute_controls(
    double dt /*period in seconds*/,
    const VicInputData & vic_input_data,
    VicCommandData & vic_command_data) = 0;

public:
  // Parameters management
  std::shared_ptr<cartesian_vic_controller::ParamListener> parameter_handler_;
  cartesian_vic_controller::Params parameters_;

protected:
  // ROS2 logging
  rclcpp::Logger logger_;
  rclcpp::Clock internal_clock_;

  template<typename T1, typename T2>
  void vec_to_eigen(const std::vector<T1> & data, T2 & matrix);

  /// Number of robot joints
  size_t num_joints_;

  /// Initial robot joint positions
  Eigen::VectorXd initial_joint_positions_;

  /// Control mode (admittance / impedance)
  ControlMode control_mode_ = ControlMode::INVALID;

  /// Vic controllers internal state
  VicState vic_state_;

  // transforms needed for vic update
  VicTransforms vic_transforms_;

  /// If true, the parameters values are ignored (use "set_interaction_parameters()" instead)
  bool use_streamed_interaction_parameters_ = false;

  /// Dynamics interface plugin loader
  std::shared_ptr<pluginlib::ClassLoader<dynamics_interface::DynamicsInterface>>
  dynamics_loader_;

  /// Dynamics interface
  std::unique_ptr<dynamics_interface::DynamicsInterface> dynamics_;

private:
  /// Filtered wrench expressed in world frame
  Eigen::Matrix<double, 6, 1> wrench_world_;
  /// Filtered wrench expressed in robot base frame
  Eigen::Matrix<double, 6, 1> wrench_base_;

  /// Filtered external torques
  Eigen::VectorXd filtered_external_torques_;
};

}  // namespace cartesian_vic_controller

#endif  // CARTESIAN_VIC_CONTROLLER__CARTESIAN_VIC_RULE_HPP_
