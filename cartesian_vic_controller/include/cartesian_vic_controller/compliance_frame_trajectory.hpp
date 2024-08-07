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
/// \authors: Thibault Poignonec <thibault.poignonec@gmail.com>

#ifndef CARTESIAN_VIC_CONTROLLER__COMPLIANCE_FRAME_TRAJECTORY_HPP_
#define CARTESIAN_VIC_CONTROLLER__COMPLIANCE_FRAME_TRAJECTORY_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

// C++
#include <map>
#include <memory>
#include <vector>
#include <string>

// Custom msgs
#include "cartesian_control_msgs/msg/cartesian_trajectory.hpp"
#include "cartesian_control_msgs/msg/compliant_frame_trajectory.hpp"


namespace cartesian_vic_controller
{

struct CompliantFrame
{
  // Time
  //------
  /// Time between first frame and this one in seconds
  /// (i.e., for the first frame of the trajectory, relative_time = 0.0)
  double relative_time = 0.0;

  // Interaction parameters
  //------------------------
  /// Desired inertia matrix, expressed in the control frame
  Eigen::Matrix<double, 6, 6> inertia;
  /// Desired stiffness matrix, expressed in the control frame
  Eigen::Matrix<double, 6, 6> stiffness;
  /// Desired damping matrix, expressed in the control frame
  Eigen::Matrix<double, 6, 6> damping;

  // Reference robot state
  // (control frame w.r.t. robot base frame)
  //-----------------------
  /// Desired cartesian robot pose as an homogeneous transformation "F_base --> F_control"
  Eigen::Isometry3d pose;
  /// Desired robot cartesian velocity at the control frame, expressed in base frame
  Eigen::Matrix<double, 6, 1> velocity;
  /// Desired robot cartesian acceleration at the control frame, expressed in base frame
  Eigen::Matrix<double, 6, 1> acceleration;
  /// Desired robot cartesian wrench at control frame, expressed in base frame
  /// (we suppose this is also the interaction frame...)
  Eigen::Matrix<double, 6, 1> wrench;
};

class CompliantFrameTrajectory
{
public:
  explicit CompliantFrameTrajectory(size_t trajectory_lenght);

  bool resize(size_t trajectory_lenght);

  const CompliantFrame & get_compliant_frame(unsigned int index = 0) const;

  size_t N() const;

  bool fill_desired_robot_state_from_msg(
    unsigned int index,
    const cartesian_control_msgs::msg::CartesianTrajectoryPoint & desired_cartesian_state);

  bool fill_desired_desired_robot_state(
    unsigned int index,
    const Eigen::Isometry3d & desired_pose,
    const Eigen::Matrix<double, 6, 1> & desired_velocity,
    const Eigen::Matrix<double, 6, 1> & desired_acceleration,
    const Eigen::Matrix<double, 6, 1> & desired_wrench = Eigen::Matrix<double, 6, 1>::Zero());

  bool fill_desired_compliance_from_msg(
    unsigned int index,
    const cartesian_control_msgs::msg::CartesianCompliance & desired_compliance);

  bool fill_desired_compliance(
    unsigned int index,
    const Eigen::Matrix<double, 6, 1> & desired_inertia,
    const Eigen::Matrix<double, 6, 1> & desired_stiffness,
    const Eigen::Matrix<double, 6, 1> & desired_damping);

  bool fill_desired_compliance(
    const Eigen::Matrix<double, 6, 1> & desired_inertia,
    const Eigen::Matrix<double, 6, 1> & desired_stiffness,
    const Eigen::Matrix<double, 6, 1> & desired_damping);

protected:
  std::vector<CompliantFrame> frames_;
};

}  // namespace cartesian_vic_controller

#endif  // CARTESIAN_VIC_CONTROLLER__COMPLIANCE_FRAME_TRAJECTORY_HPP_
