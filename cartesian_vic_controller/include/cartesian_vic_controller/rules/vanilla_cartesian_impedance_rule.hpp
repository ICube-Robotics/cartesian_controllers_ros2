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
/// \authors: Thibault Poignonec
/// \description: Plugin for CartesianVicRule implementing "classical" impedance control.

#ifndef CARTESIAN_VIC_CONTROLLER__RULES__VANILLA_CARTESIAN_IMPEDANCE_RULE_HPP_
#define CARTESIAN_VIC_CONTROLLER__RULES__VANILLA_CARTESIAN_IMPEDANCE_RULE_HPP_


#include <memory>

#include "cartesian_vic_controller/cartesian_vic_rule.hpp"

namespace cartesian_vic_controller
{

class VanillaCartesianImpedanceRule : public CartesianVicRule
{
public:
  controller_interface::return_type init(
    const std::shared_ptr<cartesian_vic_controller::ParamListener> & parameter_handler
  ) override;

  /// Configure admittance solver
  controller_interface::return_type configure(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node,
    const size_t num_joints
  ) override;

  /// Reset all values back to default
  controller_interface::return_type reset(const size_t num_joints) override;

protected:
  /// Actual vic (admittance) control logic
  bool compute_controls(
    double dt /*period in seconds*/,
    const VicInputData & vic_input_data,
    VicCommandData & vic_command_data) override;

private:
  bool reset_rule__internal_storage(const size_t num_joints);

  // Internal data for this rule
  Eigen::Matrix<double, 6, 6> I_;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> I_joint_space_;
  Eigen::Matrix<double, 6, Eigen::Dynamic> J_;
  Eigen::Matrix<double, Eigen::Dynamic, 6> J_pinv_;
  Eigen::Matrix<double, 6, Eigen::Dynamic> J_dot_;
  Eigen::VectorXd gravity_;
  Eigen::VectorXd coriolis_;
  Eigen::VectorXd raw_joint_command_effort_;

  double alpha_pinv_ = 0.000005;

  // Nullspace solver
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> nullspace_projection_;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> M_nullspace_;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> K_nullspace_;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> D_nullspace_;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> M_inv_nullspace_;
};

}  // namespace cartesian_vic_controller

#endif  // CARTESIAN_VIC_CONTROLLER__RULES__VANILLA_CARTESIAN_IMPEDANCE_RULE_HPP_
