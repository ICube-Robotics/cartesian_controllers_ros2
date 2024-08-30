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

#ifndef CARTESIAN_VIC_TELEOP_CONTROLLER__TELEOP_RULE_HPP_
#define CARTESIAN_VIC_TELEOP_CONTROLLER__TELEOP_RULE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/time.hpp"

#include "cartesian_vic_teleop_controller/teleop_data.hpp"

namespace cartesian_vic_teleop_controller
{

class TeleopRule
{
public:
  TeleopRule();
  ~TeleopRule() = default;

  /// Initialize the rule
  virtual bool init(
    std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface,
    const std::string & param_namespace = "teleoperation") = 0;

  /// Update the rule to compute the teleoperation commands and update the state msg
  virtual bool update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period,
    const TeleopDataInput & teleop_data_input,
    TeleopDataOutput & teleop_data_output) = 0;

  /// Retrieve the TeleopControllerState msg. Call update() first!!!
  bool get_current_state_msg(
    const TeleopDataInput & teleop_data_input,
    const TeleopDataOutput & teleop_data_output,
    cartesian_control_msgs::msg::TeleopControllerState & msg) const;

protected:
  /// Clear the diagnostic_data field of the TeleopControllerState msg
  bool clear_diagnostic_data();

  /// Add a new diagnostic value to the diagnostic_data field of the TeleopControllerState msg
  bool add_diagnostic_value(std::string key, double value);

  /** The diagnostic_data that will be added to the diagnostic_data
   * field of the TeleopControllerState msg
   */
  std::vector<std::string> diagnostic_keys_;
  std::vector<double> diagnostic_values_;

  /// ROS2 local logger
  rclcpp::Logger logger_;

  /// ROS2 local clock for throttle-type logging
  rclcpp::Clock internal_clock_;

  /// True if init() has been called successfully.
  bool is_initialized_;

  /// Teleop state msg that can be retrieved with get_current_state_msg()
  cartesian_control_msgs::msg::TeleopControllerState msg_teleop_state_;
};

}  // namespace cartesian_vic_teleop_controller

#endif  // CARTESIAN_VIC_TELEOP_CONTROLLER__TELEOP_RULE_HPP_
