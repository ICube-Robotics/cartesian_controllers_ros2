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

#include "cartesian_vic_teleop_controller/teleop_rule.hpp"

#include "rclcpp/logging.hpp"
#include "rcutils/logging_macros.h"

#include "cartesian_vic_teleop_controller/teleop_data.hpp"


namespace cartesian_vic_teleop_controller
{

TeleopRule::TeleopRule()
: is_initialized_(false),
  logger_(rclcpp::get_logger("TeleopRule"))
{
    // Nothing to do, see init() in specialized rules.
}

bool TeleopRule::get_current_state_msg(
  const TeleopDataInput & teleop_data_input,
  const TeleopDataOutput & teleop_data_output,
  cartesian_control_msgs::msg::TeleopControllerState & msg) const
{
  if (!is_initialized_) {
    RCLCPP_ERROR(
            logger_, "Failed to retrieve TeleopControllerState: TeleopRule is not initialized");
    return false;
  }
  return to_msg(teleop_data_input, teleop_data_output, diagnostic_keys_, diagnostic_values_, msg);
}

bool TeleopRule::clear_diagnostic_data()
{
  diagnostic_keys_.clear();
  diagnostic_values_.clear();
  return true;
}

bool TeleopRule::add_diagnostic_value(std::string key, double value)
{
  // Checking if the map already contains this specific key
  if (std::find(diagnostic_keys_.begin(), diagnostic_keys_.end(), key) != diagnostic_keys_.end()) {
    // the key already exists!
    RCLCPP_ERROR(
            logger_,
            "Failed to add diagnostic value with key '%s': key already exists!",
            key.c_str()
    );
    return false;
  }

  diagnostic_keys_.push_back(key);
  diagnostic_values_.push_back(value);
  return true;
}

}  // namespace cartesian_vic_teleop_controller
