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

bool CompliantFrameTrajectory::fill_desired_robot_state_from_msg(
  unsigned int index,
  const cartesian_control_msgs::msg::CartesianTrajectoryPoint & desired_cartesian_state)
{
  // Test index is valid
  if (index >= N()) {
    return false;
  }

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
  // No "tf2::fromMsg" impl. for geometry_msgs::msg::Accel
  frames_[index].acceleration[0] = desired_cartesian_state.acceleration.linear.x;
  frames_[index].acceleration[1] = desired_cartesian_state.acceleration.linear.y;
  frames_[index].acceleration[2] = desired_cartesian_state.acceleration.linear.z;
  frames_[index].acceleration[3] = desired_cartesian_state.acceleration.angular.x;
  frames_[index].acceleration[4] = desired_cartesian_state.acceleration.angular.y;
  frames_[index].acceleration[5] = desired_cartesian_state.acceleration.angular.z;

  // No "tf2::fromMsg" impl. for geometry_msgs::msg::wrench
  //tf2::fromMsg(
  //  desired_cartesian_state.wrench,
  //  frames_[index].wrench
  //);
  frames_[index].wrench[0] = desired_cartesian_state.wrench.force.x;
  frames_[index].wrench[1] = desired_cartesian_state.wrench.force.y;
  frames_[index].wrench[2] = desired_cartesian_state.wrench.force.z;
  frames_[index].wrench[3] = desired_cartesian_state.wrench.torque.x;
  frames_[index].wrench[4] = desired_cartesian_state.wrench.torque.y;
  frames_[index].wrench[5] = desired_cartesian_state.wrench.torque.z;

  return success;
}

bool CompliantFrameTrajectory::fill_desired_desired_robot_state(
  unsigned int index,
  const Eigen::Isometry3d & desired_pose,
  const Eigen::Matrix<double, 6, 1> & desired_velocity,
  const Eigen::Matrix<double, 6, 1> & desired_acceleration,
  const Eigen::Matrix<double, 6, 1> & desired_wrench)
{
  if (index >= N()) {
    return false;
  }
  frames_[index].relative_time = 0.0;
  frames_[index].pose = desired_pose;
  frames_[index].velocity = desired_velocity;
  frames_[index].acceleration = desired_acceleration;
  frames_[index].wrench = desired_wrench;

  return true;
}

bool CompliantFrameTrajectory::fill_desired_compliance_from_msg(
  unsigned int index,
  const cartesian_control_msgs::msg::CartesianCompliance & desired_compliance)
{
  // Test index is valid
  if (index >= N()) {
    return false;
  }

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

  // Test index is valid
  if (index >= N()) {
    return false;
  }

  frames_[index].inertia = desired_inertia;
  frames_[index].stiffness = desired_stiffness;
  frames_[index].damping = desired_damping;

  return true;
}

} // namespace cartesian_admittance_controller
