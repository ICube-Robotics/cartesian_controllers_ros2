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

#ifndef CARTESIAN_VIC_TELEOP_CONTROLLER__RULES__VANILLA_TELEOP_RULE_HPP_
#define CARTESIAN_VIC_TELEOP_CONTROLLER__RULES__VANILLA_TELEOP_RULE_HPP_

#include <memory>
#include <string>

#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/time.hpp"

#include "cartesian_vic_teleop_controller/teleop_rule.hpp"
#include "cartesian_vic_teleop_controller/teleop_data.hpp"

namespace cartesian_vic_teleop_controller
{

class VanillaTeleopRule : public TeleopRule
{
public:
  VanillaTeleopRule();
  ~VanillaTeleopRule() = default;

  bool init(
    std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface,
    const std::string & param_namespace = "teleoperation") override;

  bool update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period,
    const TeleopDataInput & teleop_data_input,
    TeleopDataOutput & teleop_data_output) override;
};

}  // namespace cartesian_vic_teleop_controller

#endif  // CARTESIAN_VIC_TELEOP_CONTROLLER__RULES__VANILLA_TELEOP_RULE_HPP_
