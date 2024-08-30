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

#ifndef CARTESIAN_VIC_TELEOP_CONTROLLER__CARTESIAN_VIC_TELEOP_CONTROLLER_HPP_
#define CARTESIAN_VIC_TELEOP_CONTROLLER__CARTESIAN_VIC_TELEOP_CONTROLLER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "cartesian_vic_teleop_controller/visibility_control.h"
#include "cartesian_vic_teleop_controller/mapping_manager.hpp"
#include "cartesian_vic_teleop_controller/cartesian_vic_teleop_logic.hpp"

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

// Base class for VIC controller
#include "cartesian_vic_controller/cartesian_vic_controller.hpp"

// ROS2 standard msgs
#include "std_msgs/msg/bool.hpp"

// Custom msgs
#include "cartesian_control_msgs/msg/vic_controller_state.hpp"
#include "cartesian_control_msgs/msg/cartesian_trajectory.hpp"
#include "cartesian_control_msgs/msg/compliant_frame_trajectory.hpp"
#include "cartesian_control_msgs/msg/teleop_controller_state.hpp"
#include "cartesian_control_msgs/msg/teleop_compliance.hpp"

namespace cartesian_vic_teleop_controller
{
class CartesianVicTeleopController : public cartesian_vic_controller::CartesianVicController
{
public:
  CartesianVicTeleopController();
  virtual ~CartesianVicTeleopController() = default;

  CARTESIAN_VIC_TELEOP_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  CARTESIAN_VIC_TELEOP_CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  CARTESIAN_VIC_TELEOP_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  CARTESIAN_VIC_TELEOP_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  CARTESIAN_VIC_TELEOP_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  CARTESIAN_VIC_TELEOP_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  CARTESIAN_VIC_TELEOP_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  using Base = cartesian_vic_controller::CartesianVicController;

protected:
  PassiveVicTeleopLogic teleop_logic_;

  // Publisher teleop controller state (for logging purposes only)
  rclcpp::Publisher<cartesian_control_msgs::msg::TeleopControllerState>::SharedPtr
    non_rt_teleop_state_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<
      cartesian_control_msgs::msg::TeleopControllerState>> teleop_state_publisher_;
  cartesian_control_msgs::msg::TeleopControllerState teleop_state_msg_;

  // Publisher follower robot VIC reference trajectory
  rclcpp::Publisher<cartesian_control_msgs::msg::CompliantFrameTrajectory>::SharedPtr
    non_rt_follower_vic_ref_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<
      cartesian_control_msgs::msg::CompliantFrameTrajectory>> follower_vic_ref_publisher_;

  // Subscriber to follower VIC state
  // TODO(tpoignonec): remove follower_vic_state_msg_
  cartesian_control_msgs::msg::VicControllerState follower_vic_state_msg_;
  std::shared_ptr<cartesian_control_msgs::msg::VicControllerState>
  follower_vic_state_msg_ptr_;
  rclcpp::Subscription<cartesian_control_msgs::msg::VicControllerState>::SharedPtr
    follower_vic_state_subscriber_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<
      cartesian_control_msgs::msg::VicControllerState>> input_follower_vic_state_msg_;

  // Subscriber to teleoperation compliance ref.
  rclcpp::Subscription<
    cartesian_control_msgs::msg::TeleopCompliance>::SharedPtr teleop_compliance_subscriber_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<
      cartesian_control_msgs::msg::TeleopCompliance>> input_teleop_compliance_msg_;
  std::shared_ptr<cartesian_control_msgs::msg::TeleopCompliance> teleop_compliance_msg_ptr_;

  // Subscriber to follower VIC state
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_clutched_subscriber_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::Bool>> input_is_clutched_msg_;
  std::shared_ptr<std_msgs::msg::Bool> is_clutched_msg_ptr_;

  // Storage
  cartesian_control_msgs::msg::CompliantFrameTrajectory leader_vic_ref_;
  cartesian_control_msgs::msg::CompliantFrameTrajectory follower_vic_ref_;
};

}  // namespace cartesian_vic_teleop_controller

#endif  // CARTESIAN_VIC_TELEOP_CONTROLLER__CARTESIAN_VIC_TELEOP_CONTROLLER_HPP_
