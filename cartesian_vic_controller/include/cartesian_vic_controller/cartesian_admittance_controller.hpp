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

// Based on package "ros2_controllers/admittance_controller", Copyright (c) 2022, PickNik, Inc.

#ifndef CARTESIAN_VIC_CONTROLLER__CARTESIAN_ADMITTANCE_CONTROLLER_HPP_
#define CARTESIAN_VIC_CONTROLLER__CARTESIAN_ADMITTANCE_CONTROLLER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "cartesian_vic_controller/visibility_control.h"
#include "cartesian_vic_controller/cartesian_vic_controller.hpp"

namespace cartesian_vic_controller
{

class CartesianAdmittanceController : public CartesianVicController
{
public:
  CARTESIAN_VIC_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  CARTESIAN_VIC_CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  CARTESIAN_VIC_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  CARTESIAN_VIC_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  CARTESIAN_VIC_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  CARTESIAN_VIC_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  CARTESIAN_VIC_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  /**
   * @brief Check if the command interfaces are configured correctly. This function
   * has to be implemented by specialized controllers (e.g., impedance / admittance).
   */
  virtual bool is_command_interfaces_config_valid() const;

  /**
   * @brief Write values from joint_state_command to claimed hardware interfaces. This function
   * has to be implemented by specialized controllers (e.g., impedance / admittance).
   */
  bool write_state_to_hardware(trajectory_msgs::msg::JointTrajectoryPoint & joint_state_c)
  override;
};

}  // namespace cartesian_vic_controller

#endif  // CARTESIAN_VIC_CONTROLLER__CARTESIAN_ADMITTANCE_CONTROLLER_HPP_
