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

#include "cartesian_admittance_controller/compliance_frame_trajectory.hpp"


#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_kdl/tf2_kdl.hpp"
#include "tf2_ros/buffer.h"

namespace cartesian_admittance_controller
{
CompliantFrameTrajectory::CompliantFrameTrajectory(size_t trajectory_lenght)
{
  if (trajectory_lenght == 0) {
    // throw exception?
    trajectory_lenght = 1;
  }

  // Init compliance frame trajectory
  frames_.reserve(trajectory_lenght);
  for (unsigned int i = 0; i < trajectory_lenght; i++) {
    frames_.push_back(CompliantFrame());
    // TODO(tpoignonec): fill data with NaN?
  }
}

const CompliantFrame & CompliantFrameTrajectory::get_compliant_frame(unsigned int index) const
{
  return frames_[index];
}

size_t CompliantFrameTrajectory::N() const
{
  return frames_.size();
}

bool CompliantFrameTrajectory::fill_from_msg(
  const cartesian_control_msgs::msg::CompliantFrameTrajectory & frame_msgs)
{
  if (frame_msgs.cartesian_trajectory_points.size() != N()) {
    //TODO(tpoignonec): throw exception OR print log error?
    return false;
  }
  if (frame_msgs.cartesian_trajectory_points.size() != frame_msgs.compliance_at_points.size()) {
    //TODO(tpoignonec): throw exception OR print log error?
    return false;
  }
  // Update reference compliant frames
  bool success = true;
  for (unsigned int i = 0; i < N(); i++) {
    // TODO(tpoignonec): Check the frame is correct (i.e., control w.r.t. base)!
    success &= fill_desired_robot_state_from_msg(i, frame_msgs.cartesian_trajectory_points[i]);
    success &= fill_desired_compliance_from_msg(i, frame_msgs.compliance_at_points[i]);
  }
  return success;
}

bool CompliantFrameTrajectory::fill_desired_robot_state_from_msg(
  unsigned int index,
  const cartesian_control_msgs::msg::CartesianTrajectoryPoint & desired_cartesian_state)
{
  bool success = true; // return flag

  // Retrieve timestamp
  rclcpp::Duration time_from_start = desired_cartesian_state.time_from_start;
  frames_[index].relative_time = time_from_start.seconds();

  // Fill desired Cartesian state (pose, twist, acc., and wrench) from msg
  tf2::fromMsg(
    desired_cartesian_state.pose,
    frames_[index].pose
  );
  tf2::fromMsg(
    desired_cartesian_state.velocity,
    frames_[index].velocity
  );
  tf2::fromMsg(
    desired_cartesian_state.acceleration,
    frames_[index].acceleration
  );
  tf2::fromMsg(
    desired_cartesian_state.wrench,
    frames_[index].wrench
  );

  return success;
}

bool CompliantFrameTrajectory::fill_desired_compliance_from_msg(
  unsigned int index,
  const cartesian_control_msgs::msg::CartesianCompliance & desired_compliance)
{
  (void)index;
  (void)desired_compliance;
  /*
  // TODO(tpoignonec) --> diagonal or dense ? :/
  bool success = true; // return flag

  frames_[index].inertia = ...
  frames_[index].stiffness = ...
  frames_[index].damping = ...
  */
  return false;
}

bool CompliantFrameTrajectory::fill_desired_compliance(
  const Eigen::Matrix<double, 6, 1> & desired_inertia,
  const Eigen::Matrix<double, 6, 1> & desired_stiffness,
  const Eigen::Matrix<double, 6, 1> & desired_damping)
{
  bool success = true; // return flag
  for (unsigned int i = 0; i < N(); i++) {
    success &= fill_desired_compliance(i, desired_inertia, desired_stiffness, desired_damping);
  }
  return success;
}

bool CompliantFrameTrajectory::fill_desired_compliance(
  unsigned int index,
  const Eigen::Matrix<double, 6, 1> & desired_inertia,
  const Eigen::Matrix<double, 6, 1> & desired_stiffness,
  const Eigen::Matrix<double, 6, 1> & desired_damping)
{
  frames_[index].inertia = desired_inertia;
  frames_[index].stiffness = desired_stiffness;
  frames_[index].damping = desired_damping;

  return true;
}

} // namespace cartesian_admittance_controller
