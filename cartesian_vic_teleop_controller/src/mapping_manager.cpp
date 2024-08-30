// Copyright 2022, ICube Laboratory, University of Strasbourg
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
/*!\file    mapping_manager.cpp
 *
 * \author  thibault Poignonec <tpoignonec@unistra.fr>
 * \date    2022/06/24
 *
 */
//-----------------------------------------------------------------------------

#include "cartesian_vic_teleop_controller/mapping_manager.hpp"
#include <fstream>
#include "rclcpp/logging.hpp"

namespace cartesian_vic_teleop_controller
{
// -----------------------------------------
//          Constructor/destructor
// -----------------------------------------
MappingManager::MappingManager()
: logger_(rclcpp::get_logger("mapping_manager"))
{
  mapping_initialized_ = false;
}

MappingManager::~MappingManager() {}

// -----------------------------------------
//              Initialization
// -----------------------------------------
bool MappingManager::init(
  const Eigen::Matrix3d & master_base_to_follower_base_axis_mapping)
{
  // Retrieve mapping parameters
  // TODO(tpoignonec): Dynamic parameter
  master_base_to_follower_base_axis_mapping_ = master_base_to_follower_base_axis_mapping;

  // All ok
  mapping_initialized_ = false;
  workspaces_disengaged_ = false;
  return true;
}


// -----------------------------------------
//              Update mapping
// -----------------------------------------
bool MappingManager::update(
  const Eigen::Isometry3d & master_pose_in_master_base,
  bool clutch_is_on)
{
  // Security check(s)
  bool all_ok = true;
  if (!mapping_initialized_) {
    RCLCPP_ERROR(logger_, "Mapping manager: Call init() before update()!");
    return false;
  }
  // Process data
  if ((!workspaces_disengaged_ && !clutch_is_on) || !use_workspace_clutching_) {
    // !use_workspace_clutching_ --> "always engaged"
    // Save current mapping
    map_master_tool_to_follower_tool(
      master_pose_in_master_base,
      last_master_pose_mapped_into_follower_base_);
  } else if (!workspaces_disengaged_ && clutch_is_on) {
    // Toggle clutch
    workspaces_disengaged_ = true;
    info_verbose("Clutch is on: you can now reposition the master robot.");
  } else if (workspaces_disengaged_ && !clutch_is_on) {
    // Toggle clutch
    workspaces_disengaged_ = false;
    info_verbose(
        "Be carefult! Clutch is off, any master displacement will cause the robot to move.");
  } else {
    // Update mapping such that no follower side motion are generated
    all_ok &= reset_mapping(
      last_master_pose_mapped_into_follower_base_,
      master_pose_in_master_base,
      master_base_to_follower_base_axis_mapping_
    );
  }
  return all_ok;
}

bool MappingManager::reset_mapping(
  const Eigen::Isometry3d & follower_pose_in_follower_base,
  const Eigen::Isometry3d & master_pose_in_master_base)
{
  return reset_mapping(
    follower_pose_in_follower_base, master_pose_in_master_base,
    master_base_to_follower_base_axis_mapping_);
}

bool MappingManager::reset_mapping(
  const Eigen::Isometry3d & follower_pose_in_follower_base,
  const Eigen::Isometry3d & master_pose_in_master_base,
  const Eigen::Matrix3d & master_base_to_follower_base_axis_mapping)
{
  /* Compute 'master_base_to_follower_base_mapping' */
  // Apply chosen axis mapping
  master_base_to_follower_base_mapping_.linear() = master_base_to_follower_base_axis_mapping;

  // Retrieve frames relative translation offset
  master_base_to_follower_base_mapping_.translation() =
    follower_pose_in_follower_base.translation() -
    master_base_to_follower_base_axis_mapping * master_pose_in_master_base.translation();

  /* Compute 'master_base_to_follower_base_mapping' (only a rotation) */
  follower_tool_to_master_tool_axis_mapping_ = Eigen::Isometry3d::Identity();
  follower_tool_to_master_tool_axis_mapping_.linear() =
    master_pose_in_master_base.linear().transpose() *
    master_base_to_follower_base_mapping_.linear().transpose() *
    follower_pose_in_follower_base.linear();

  // Check the resulting mapping is consistent
  // TODO(tpoignonec): remove this (or make it optional) once tested
  mapping_initialized_ = true;
  map_master_tool_to_follower_tool(
    master_pose_in_master_base,
    last_master_pose_mapped_into_follower_base_);
  if (!last_master_pose_mapped_into_follower_base_.isApprox(follower_pose_in_follower_base)) {
    mapping_initialized_ = false;
    RCLCPP_ERROR(logger_, "Mapping manager: The computed mapping is invalid!");
    return false;
  }
  return true;
}

// -----------------------------------------
//          Direct/inverse mappings
// -----------------------------------------
bool MappingManager::map_master_tool_to_follower_tool(
  const Eigen::Isometry3d & master_pose_in_master_base,
  Eigen::Isometry3d & mapped_master_pose) const
{
  // Security check(s)
  bool all_ok = true;
  if (!mapping_initialized_) {
    RCLCPP_ERROR(logger_, "Mapping manager: Call init() and reset() before using mapping!");
    return false;
  }
  // Map master pose
  mapped_master_pose = \
    master_base_to_follower_base_mapping_ *
    master_pose_in_master_base *
    follower_tool_to_master_tool_axis_mapping_;
  return all_ok;
}

bool MappingManager::inverse_map_follower_tool_to_master_tool(
  const Eigen::Isometry3d & follower_pose_in_follower_base,
  Eigen::Isometry3d & mapped_follower_pose) const
{
  // Security check(s)
  bool all_ok = true;
  if (!mapping_initialized_) {
    RCLCPP_ERROR(logger_, "Mapping manager: Call init() and reset() before using mapping!");
    return false;
  }
  // Map master pose
  mapped_follower_pose =
    master_base_to_follower_base_mapping_.inverse() *
    follower_pose_in_follower_base *
    follower_tool_to_master_tool_axis_mapping_.inverse();
  return all_ok;
}


bool MappingManager::map_twist_or_wrench_from_master_to_follower(
  const Eigen::Matrix<double, 6, 1> & vector_in_master_base,
  Eigen::Matrix<double, 6, 1> & vector_in_follower_base) const
{
  // Security check(s)
  if (!mapping_initialized_) {
    RCLCPP_ERROR(logger_, "Mapping manager: Call init() and reset() before using mapping!");
    return false;
  }
  map_tensor(
    master_base_to_follower_base_mapping_.linear(), vector_in_master_base,
    vector_in_follower_base);
  return true;
}

bool MappingManager::map_twist_or_wrench_from_follower_to_master(
  const Eigen::Matrix<double, 6, 1> & vector_in_follower_base,
  Eigen::Matrix<double, 6, 1> & vector_in_master_base) const
{
  // Security check(s)
  if (!mapping_initialized_) {
    RCLCPP_ERROR(logger_, "Mapping manager: Call init() and reset() before using mapping!");
    return false;
  }
  map_tensor(
    master_base_to_follower_base_mapping_.linear().transpose(), vector_in_follower_base,
    vector_in_master_base);
  return true;
}

bool MappingManager::map_SDPD_from_master_to_follower(
  const Eigen::Matrix<double, 6, 6> & SDPD_in_master_base,
  Eigen::Matrix<double, 6, 6> & SDPD_in_follower_base)
{
  // Security check(s)
  if (!mapping_initialized_) {
    RCLCPP_ERROR(logger_, "Mapping manager: Call init() and reset() before using mapping!");
    return false;
  }
  tmp_6x6_matrix_.setZero();
  tmp_6x6_matrix_.block<3, 3>(0, 0) = master_base_to_follower_base_mapping_.linear();
  tmp_6x6_matrix_.block<3, 3>(3, 3) = master_base_to_follower_base_mapping_.linear();

  SDPD_in_follower_base = tmp_6x6_matrix_ * SDPD_in_master_base * tmp_6x6_matrix_.transpose();

  return true;
}

bool MappingManager::map_SDPD_from_follower_to_master(
  const Eigen::Matrix<double, 6, 6> & SDPD_in_follower_base,
  Eigen::Matrix<double, 6, 6> & SDPD_in_master_base)
{
  // Security check(s)
  if (!mapping_initialized_) {
    RCLCPP_ERROR(logger_, "Mapping manager: Call init() before using mapping!");
    return false;
  }
  tmp_6x6_matrix_.setZero();
  tmp_6x6_matrix_.block<3, 3>(0, 0) = master_base_to_follower_base_mapping_.linear().transpose();
  tmp_6x6_matrix_.block<3, 3>(3, 3) = master_base_to_follower_base_mapping_.linear().transpose();

  SDPD_in_master_base = tmp_6x6_matrix_ * SDPD_in_follower_base * tmp_6x6_matrix_.transpose();
  return true;
}


void MappingManager::map_tensor(
  const Eigen::Matrix3d & orientation,
  const Eigen::Matrix<double, 6, 1> & in,
  Eigen::Matrix<double, 6, 1> & out) const
{
  out.head(3) = orientation * in.head(3);
  out.tail(3) = orientation * in.tail(3);
}

void MappingManager::info_verbose(const std::string & msg) const
{
  RCLCPP_INFO(logger_, "%s", msg.c_str());
}

}  // namespace cartesian_vic_teleop_controller
