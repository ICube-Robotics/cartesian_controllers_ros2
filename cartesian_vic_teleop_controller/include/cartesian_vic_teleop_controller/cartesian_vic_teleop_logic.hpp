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

#ifndef CARTESIAN_VIC_TELEOP_CONTROLLER__CARTESIAN_VIC_TELEOP_LOGIC_HPP_
#define CARTESIAN_VIC_TELEOP_CONTROLLER__CARTESIAN_VIC_TELEOP_LOGIC_HPP_

#include <string>
#include <memory>

#include "pluginlib/class_loader.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/time.hpp"

#include "cartesian_vic_teleop_controller/Ros2VersionConfig.h"  // ROS2 version macros

#include "cartesian_vic_teleop_controller/mapping_manager.hpp"
#include "cartesian_vic_teleop_controller/teleop_data.hpp"
#include "cartesian_vic_teleop_controller/teleop_rule.hpp"

#include "cartesian_control_msgs/msg/teleop_controller_state.hpp"
#include "cartesian_control_msgs/msg/teleop_compliance.hpp"

// VIC controllers state
#include "cartesian_vic_controller/cartesian_vic_state.hpp"

// Custom msgs
#include "cartesian_control_msgs/msg/vic_controller_state.hpp"
#include "cartesian_control_msgs/msg/cartesian_trajectory.hpp"
#include "cartesian_control_msgs/msg/compliant_frame_trajectory.hpp"

namespace cartesian_vic_teleop_controller
{

using VicStateMsg = cartesian_control_msgs::msg::VicControllerState;
using CompliantFrameTrajectoryMsg = cartesian_control_msgs::msg::CompliantFrameTrajectory;

class PassiveVicTeleopLogic
{
public:
  PassiveVicTeleopLogic();
  ~PassiveVicTeleopLogic() = default;

  /// Initialize the logic from VicStateMsg messages
  bool init(
    const rclcpp::Time & time,
    std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface,
    const VicStateMsg & leader_vic_state_msg,
    const VicStateMsg & follower_vic_state_msg,
    const std::string & param_namespace = "teleoperation");

  /// Convenience method to initialize the logic from leader input data
  bool init(
    const rclcpp::Time & time,
    std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface,
    const cartesian_vic_controller::VicInputData & leader_input_data,
    const VicStateMsg & follower_vic_state_msg,
    const std::string & param_namespace = "teleoperation");

  /// Update the logic: update mapping, update rule, ad compute the compliant trajectories
  bool update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period,
    bool workspace_clutch_disengaged,
    const VicStateMsg & leader_vic_state_msg,
    const VicStateMsg & follower_vic_state_msg);

  /// Convenience method to update the logic from leader input data
  bool update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period,
    bool workspace_clutch_disengaged,
    const cartesian_vic_controller::VicInputData & leader_input_data,
    const VicStateMsg & follower_vic_state_msg);

  /// Set the desired compliance of the leader / follower VIC controllers
  bool setTeleoperationCompliance(const cartesian_control_msgs::msg::TeleopCompliance & msg);

  /// Return true if the logic has been initialized
  bool is_initialized() const {return is_initialized_;}

  /// Get the computed leader compliant trajectory
  bool get_leader_vic_ref(CompliantFrameTrajectoryMsg & leader_vic_ref_msg);

  /// Get the computed follower compliant trajectory
  bool get_follower_vic_ref(CompliantFrameTrajectoryMsg & follower_vic_ref_msg);

  /// Retrieve the TeleopControllerState msg from rule. Call update() first!!!
  bool get_current_state_msg(cartesian_control_msgs::msg::TeleopControllerState & msg);

protected:
  // Internal methods
  bool extract_input_data(
    const VicStateMsg & leader_vic_state_msg,
    const VicStateMsg & follower_vic_state_msg);

  bool extract_input_data(
    const cartesian_vic_controller::VicInputData & leader_input_data,
    const VicStateMsg & follower_vic_state_msg);

  bool extract_follower_input_data(
    const VicStateMsg & follower_vic_state_msg);

  bool internal_init(
    const rclcpp::Time & time,
    std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface,
    const std::string & param_namespace);

  bool internal_update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period,
    bool workspace_clutch_disengaged);

protected:
  /// ROS2 local logger
  rclcpp::Logger logger_;

  /// ROS2 local clock for throttle-type logging
  rclcpp::Clock internal_clock_;

  /// True if init() has been called successfully.
  bool is_initialized_ = false;

  /// True if update() has been called successfully at least once.
  bool has_update_been_called_ = false;

  /// Ros2 parameter server
  std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface_;

  /// Teleop rule plugin loader
  std::shared_ptr<pluginlib::ClassLoader<cartesian_vic_teleop_controller::TeleopRule>>
  teleop_rule_loader_;

  /// Teleop rule plugin instance
  std::unique_ptr<cartesian_vic_teleop_controller::TeleopRule> teleop_rule_;

  /// True if the init() function of the rule has been called successfully.
  bool is_teleop_rule_initialized_ = false;

  /// Mapping manager: mapping from/to master/follower workspaces
  MappingManager mapping_manager_;

  /// Current pose mapping, i.e., follower_pose = transformation * leader_pose
  Eigen::Isometry3d transformation_leader_to_follower;

  /* Most recent measurement data
    *
    * !!!IMPORTANT!!! All in the follower base frame
    */
  TeleopDataInput teleop_data_input_;

  /** Computed VIC references for the leader and follower robots
    *
    *  !!!IMPORTANT!!! All in the follower base frame
    */
  TeleopDataOutput teleop_data_output_;

  /// Computed compliant frame trajectory for the leader robot
  CompliantFrameTrajectoryMsg leader_vic_ref_;

  /// Computed compliant frame trajectory for the follower robot
  CompliantFrameTrajectoryMsg follower_vic_ref_;

private:
  // Transient data storage, internal use only!
  Eigen::Isometry3d leader_pose_in_leader_base_frame_;
  Eigen::Matrix<double, 6, 1> leader_velocity_in_leader_base_frame_;
  Eigen::Matrix<double, 6, 1> leader_acceleration_in_leader_base_frame_;
  Eigen::Matrix<double, 6, 1> leader_wrench_in_leader_base_frame_;
  Eigen::Matrix<double, 6, 6> leader_natural_inertia_in_leader_base_frame_;

  Eigen::Isometry3d follower_pose_in_follower_base_frame_;
  Eigen::Matrix<double, 6, 1> follower_velocity_in_follower_base_frame_;
  Eigen::Matrix<double, 6, 1> follower_acceleration_in_follower_base_frame_;
  Eigen::Matrix<double, 6, 1> follower_wrench_in_follower_base_frame_;
  Eigen::Matrix<double, 6, 6> follower_natural_inertia_in_follower_base_frame_;

  Eigen::Isometry3d leader_reference_pose_in_leader_base_frame_;
  Eigen::Matrix<double, 6, 1> leader_reference_velocity_in_leader_base_frame_;
  Eigen::Matrix<double, 6, 1> leader_reference_acceleration_in_leader_base_frame_;
  Eigen::Matrix<double, 6, 1> leader_reference_wrench_in_leader_base_frame_;
  Eigen::Matrix<double, 6, 6> leader_reference_inertia_in_leader_base_frame_;
  Eigen::Matrix<double, 6, 6> leader_reference_stiffness_in_leader_base_frame_;
  Eigen::Matrix<double, 6, 6> leader_reference_damping_in_leader_base_frame_;
};

}  // namespace cartesian_vic_teleop_controller

#endif  // CARTESIAN_VIC_TELEOP_CONTROLLER__CARTESIAN_VIC_TELEOP_LOGIC_HPP_
