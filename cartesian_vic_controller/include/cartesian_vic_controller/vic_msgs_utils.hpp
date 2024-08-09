// Copyright 2024, ICube Laboratory, University of Strasbourg
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


//-----------------------------------------------------------------------------
/*!\file    vic_msg_utils.hpp
 *
 * \author  thibault Poignonec <tpoignonec@unistra.fr>
 * \date    2024/08/07
 */
//-----------------------------------------------------------------------------

#ifndef CARTESIAN_VIC_CONTROLLER__VIC_MSGS_UTILS_HPP_
#define CARTESIAN_VIC_CONTROLLER__VIC_MSGS_UTILS_HPP_


// Misc.
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <string>
#include <memory>

#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_kdl/tf2_kdl.hpp"

#include <rclcpp/rclcpp.hpp>

// Custom msgs
#include "cartesian_control_msgs/msg/vic_controller_state.hpp"
#include "cartesian_control_msgs/msg/cartesian_trajectory.hpp"
#include "cartesian_control_msgs/msg/compliant_frame_trajectory.hpp"
#include "cartesian_vic_controller/utils.hpp"


namespace cartesian_vic_controller
{

// Read VIC state message

bool get_robot_pose(
  const cartesian_control_msgs::msg::VicControllerState & vic_state_msg,
  Eigen::Affine3d & pose)
{
  tf2::fromMsg(vic_state_msg.pose, pose);
  return true;
}

bool get_robot_velocity(
  const cartesian_control_msgs::msg::VicControllerState & vic_state_msg,
  Eigen::Matrix<double, 6, 1> & twist)
{
  tf2::fromMsg(vic_state_msg.velocity, twist);
  return true;
}

bool get_robot_acceleration(
  const cartesian_control_msgs::msg::VicControllerState & vic_state_msg,
  Eigen::Matrix<double, 6, 1> & acceleration)
{
  acceleration[0] = vic_state_msg.acceleration.linear.x;
  acceleration[1] = vic_state_msg.acceleration.linear.y;
  acceleration[2] = vic_state_msg.acceleration.linear.z;
  acceleration[3] = vic_state_msg.acceleration.angular.x;
  acceleration[4] = vic_state_msg.acceleration.angular.y;
  acceleration[5] = vic_state_msg.acceleration.angular.z;
  return true;
}

bool get_robot_wrench(
  const cartesian_control_msgs::msg::VicControllerState & vic_state_msg,
  Eigen::Matrix<double, 6, 1> & wrench)
{
  wrench[0] = vic_state_msg.wrench.force.x;
  wrench[1] = vic_state_msg.wrench.force.y;
  wrench[2] = vic_state_msg.wrench.force.z;
  wrench[3] = vic_state_msg.wrench.torque.x;
  wrench[4] = vic_state_msg.wrench.torque.y;
  wrench[5] = vic_state_msg.wrench.torque.z;
  return true;
}

bool get_desired_robot_pose(
  const cartesian_control_msgs::msg::VicControllerState & vic_state_msg,
  Eigen::Affine3d & desired_pose)
{
  tf2::fromMsg(vic_state_msg.desired_pose, desired_pose);
  return true;
}
bool get_desired_robot_velocity(
  const cartesian_control_msgs::msg::VicControllerState & vic_state_msg,
  Eigen::Matrix<double, 6, 1> & desired_twist)
{
  tf2::fromMsg(vic_state_msg.desired_velocity, desired_twist);
  return true;
}
bool get_desired_robot_acceleration(
  const cartesian_control_msgs::msg::VicControllerState & vic_state_msg,
  Eigen::Matrix<double, 6, 1> & desired_acceleration)
{
  desired_acceleration[0] = vic_state_msg.desired_acceleration.linear.x;
  desired_acceleration[1] = vic_state_msg.desired_acceleration.linear.y;
  desired_acceleration[2] = vic_state_msg.desired_acceleration.linear.z;
  desired_acceleration[3] = vic_state_msg.desired_acceleration.angular.x;
  desired_acceleration[4] = vic_state_msg.desired_acceleration.angular.y;
  desired_acceleration[5] = vic_state_msg.desired_acceleration.angular.z;
  return true;
}
bool get_desired_robot_wrench(
  const cartesian_control_msgs::msg::VicControllerState & vic_state_msg,
  Eigen::Matrix<double, 6, 1> & desired_wrench)
{
  desired_wrench[0] = vic_state_msg.desired_wrench.force.x;
  desired_wrench[1] = vic_state_msg.desired_wrench.force.y;
  desired_wrench[2] = vic_state_msg.desired_wrench.force.z;
  desired_wrench[3] = vic_state_msg.desired_wrench.torque.x;
  desired_wrench[4] = vic_state_msg.desired_wrench.torque.y;
  desired_wrench[5] = vic_state_msg.desired_wrench.torque.z;
  return true;
}

bool get_current_impedance_profile(
  const cartesian_control_msgs::msg::VicControllerState & vic_state_msg,
  Eigen::Matrix<double, 6, 6> & M,
  Eigen::Matrix<double, 6, 6> & K,
  Eigen::Matrix<double, 6, 6> & D)
{
  bool success = fromMsg(vic_state_msg.rendered_inertia, M);
  success &= fromMsg(vic_state_msg.rendered_stiffness, K);
  success &= fromMsg(vic_state_msg.rendered_damping, D);
  return success;
}

bool get_desired_impedance_profile(
  const cartesian_control_msgs::msg::VicControllerState & vic_state_msg,
  Eigen::Matrix<double, 6, 6> & M_d,
  Eigen::Matrix<double, 6, 6> & K_d,
  Eigen::Matrix<double, 6, 6> & D_d)
{
  bool success = fromMsg(vic_state_msg.desired_inertia, M_d);
  success &= fromMsg(vic_state_msg.desired_stiffness, K_d);
  success &= fromMsg(vic_state_msg.desired_damping, D_d);
  return success;
}

bool get_natural_robot_inertia(
  const cartesian_control_msgs::msg::VicControllerState & vic_state_msg,
  Eigen::Matrix<double, 6, 6> & M_natural)
{
  bool success = fromMsg(vic_state_msg.natural_inertia, M_natural);
  return success;
}


}  // namespace cartesian_vic_controller

#endif  // CARTESIAN_VIC_CONTROLLER__VIC_MSGS_UTILS_HPP_
