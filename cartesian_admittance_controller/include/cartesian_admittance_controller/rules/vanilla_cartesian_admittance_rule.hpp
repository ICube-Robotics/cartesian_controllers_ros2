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
/// \description: Plugin for CartesianAdmittanceRule implementing "classical" admittance control

#ifndef CARTESIAN_ADMITTANCE_CONTROLLER__RULES__VANILLA_CARTESIAN_ADMITTANCE_RULE_HPP_
#define CARTESIAN_ADMITTANCE_CONTROLLER__RULES__VANILLA_CARTESIAN_ADMITTANCE_RULE_HPP_


#include <memory>

#include "cartesian_admittance_controller/cartesian_admittance_rule.hpp"


namespace cartesian_admittance_controller
{

class VanillaCartesianAdmittanceRule : public CartesianAdmittanceRule
{
public:
  controller_interface::return_type init(
    const std::shared_ptr<cartesian_admittance_controller::ParamListener> & parameter_handler
  ) override;

  /// Configure admittance solver
  controller_interface::return_type configure(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node,
    const size_t num_joints
  ) override;

  /// Reset all values back to default
  controller_interface::return_type reset(const size_t num_joints) override;

protected:
  /// Actual admittance control logic
  bool compute_controls(
    AdmittanceState & amittance_state,
    double dt /*period in seconds*/) override;
};

}  // namespace cartesian_admittance_controller

#endif  // CARTESIAN_ADMITTANCE_CONTROLLER__RULES__VANILLA_CARTESIAN_ADMITTANCE_RULE_HPP_
