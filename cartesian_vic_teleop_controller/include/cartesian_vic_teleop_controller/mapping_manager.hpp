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
/*!\file    mapping_manager.hpp
 *
 * \author  thibault Poignonec <tpoignonec@unistra.fr>
 * \date    2022/06/25
 *
 *
 * Note: imported from https://github.com/tpoignonec/teleop_controllers
 */
//-----------------------------------------------------------------------------

#ifndef CARTESIAN_VIC_TELEOP_CONTROLLER__MAPPING_MANAGER_HPP_
#define CARTESIAN_VIC_TELEOP_CONTROLLER__MAPPING_MANAGER_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace cartesian_vic_teleop_controller
{

class MappingManager
{
public:
  enum CLUTCHING_STATE
  {
    DISABLED,
    ENGAGED,
    DISENGAGED
  };

  MappingManager();
  ~MappingManager();

  /**
   * @brief Init mapping manager: instantiate/reset object and initialize master->follower mapping
   *
   * @todo Add a "clutch rotation"cparameter (disabled for now and should be disabled by default if implemented)
   */
  bool init(
    const Eigen::Matrix3d & master_base_to_follower_base_axis_mapping
  );

  /**
   * @brief Update mapping manager internal state. To be call at each time period!
   */
  bool update(
    const Eigen::Isometry3d & master_pose_in_master_base,
    bool clutch_is_on = false
  );

  /**
   * @brief Return true if the mapping has been initialized
   */
  bool is_initialized() {return mapping_initialized_;}

  /**
   * @brief TODO
   *
   * @param follower_pose_in_follower_base TODO
   * @param master_pose_in_master_base TODO
   * @param master_base_to_follower_base_axis_mapping TODO
   */

  bool reset_mapping(
    const Eigen::Isometry3d & follower_pose_in_follower_base,
    const Eigen::Isometry3d & master_pose_in_master_base,
    const Eigen::Matrix3d & master_base_to_follower_base_axis_mapping
  );
  /// @overload
  bool reset_mapping(
    const Eigen::Isometry3d & follower_pose_in_follower_base,
    const Eigen::Isometry3d & master_pose_in_master_base
  );


  // -------------  Pose mapping utils  ---------------------
  /**
   * @brief Compute mapping from master pose to follower pose
   *
   * @param master_pose_in_master_base Cartesian pose of the master robot tool w.r.t. its frame of reference ('master base').
   *                                   The pose is represented as a homogeneous rigid transformation \f${}^{\text{master base}}T_{\text{master tool}}\f$.
   * @param mapped_master_pose Mapping of the master into follower robot operational space such that the return value is
   * \f$ {}^{\text{follower base}}T_{\text{master base}} . {}^{\text{master base}}T_{\text{master_tool}} . {}^{\text{master_tool}}T_{\text{follower tool}}\f$
   */
  bool map_master_tool_to_follower_tool(
    const Eigen::Isometry3d & master_pose_in_master_base,
    Eigen::Isometry3d & mapped_master_pose) const;

  /**
   * @brief Compute inverse mapping (from follower to master)
   *
   * @see map_master_tool_to_follower_tool
   */
  bool inverse_map_follower_tool_to_master_tool(
    const Eigen::Isometry3d & follower_pose_in_follower_base,
    Eigen::Isometry3d & mapped_follower_pose) const;

  /* Additional accessors (should not be useful) */
  /**
   * @brief Compute mapping from master (tool) pose to
   * Get internal (partial) mapping \f${}^{\text{follower base}} T_{\text{master base}}\f$
   * as 4X4 homogeneous matrix
   */
  const Eigen::Isometry3d & get_transformation_from_master_base_to_follower_base() const
  {
    return master_base_to_follower_base_mapping_;
  }
  /**
   * @brief Get internal (partial) mapping \f${}^{\text{master_tool}} T_{\text{follower tool}}\f$
   * as 4X4 homogeneous matrix
   */
  const Eigen::Isometry3d & get_transformation_from_follower_tool_to_master_tool() const
  {
    return follower_tool_to_master_tool_axis_mapping_;
  }

  // -------------  Twist mapping utils  ---------------------
  /**
   * @brief Change the reference frame (master_base -> master_base, but using orientation only!)
   * of a twist/wrench 6x1 vector
   *
   * @param vector_in_master_base Input twist/wrench vector expressed in master base frame
   * @param vector_in_follower_base Resulting twist/wrench vector expressed in follower base frame
   *
   * @see map_tensor
   */
  bool map_twist_or_wrench_from_master_to_follower(
    const Eigen::Matrix<double, 6, 1> & vector_in_master_base,
    Eigen::Matrix<double, 6, 1> & vector_in_follower_base) const;

  /**
   * @brief Change the reference frame (master_base -> master_base, but using orientation only!) of a twist/wrench 6x1 vector
   *
   * @param vector_in_master_base Input twist/wrench vector expressed in master base frame
   * @param vector_in_follower_base Resulting twist/wrench vector expressed in follower base frame
   *
   * @see map_tensor
   */
  bool map_twist_or_wrench_from_follower_to_master(
    const Eigen::Matrix<double, 6, 1> & vector_in_follower_base,
    Eigen::Matrix<double, 6, 1> & vector_in_master_base) const;

  bool map_SDPD_from_master_to_follower(
    const Eigen::Matrix<double, 6, 6> & SDPD_in_master_base,
    Eigen::Matrix<double, 6, 6> & SDPD_in_follower_base);

  bool map_SDPD_from_follower_to_master(
    const Eigen::Matrix<double, 6, 6> & SDPD_in_follower_base,
    Eigen::Matrix<double, 6, 6> & SDPD_in_master_base);

  // -------------  Workspace clutching  ---------------------
  /// Returns the state of the workspace clutching is enabled (disable by default!)
  CLUTCHING_STATE get_clutching_state() const
  {
    if (!use_workspace_clutching_) {
      return CLUTCHING_STATE::DISABLED;
    } else if (workspaces_disengaged_) {
      return CLUTCHING_STATE::DISENGAGED;
    } else {
      return CLUTCHING_STATE::ENGAGED;
    }
  }

  /// Enable/disable workspace clutching (disabled by default!)
  bool set_clutching_enabled(bool value)
  {
    // Error if 'workspaces_disengaged_' is true
    if (workspaces_disengaged_) {
      // TODO(tpoignonec:) Move to .cpp and add ERROR msg
      return false;
    }
    // Update and return
    use_workspace_clutching_ = value;
    return true;
  }

  /// Returns 'True' is workspace clutching is enabled (disable by default!)
  bool is_clutching_enabled() const {return use_workspace_clutching_;}

protected:
  // ROS2 logging
  rclcpp::Logger logger_;
  // Supervision flags
  /// False by default, use 'MappingManager::set_clutching_enabled()' method to activate
  bool use_workspace_clutching_ = true;
  /// false until the 'MappingManager::init()' method has been called
  bool mapping_initialized_ = false;
  // Current mapping state
  /* When true, a master side displacement does not impact its mapping into the follower workspace
   * (i.e., the two robot are independent)
   */
  bool workspaces_disengaged_ = true;
  /// \f$ {}^{\text{follower base}}T_{\text{master base}} \f$
  Eigen::Isometry3d master_base_to_follower_base_mapping_;
  /// \f$ {}^{\text{master_tool}}T_{\text{follower tool}} \f$
  Eigen::Isometry3d follower_tool_to_master_tool_axis_mapping_;
  Eigen::Matrix3d master_base_to_follower_base_axis_mapping_;
  // Cached history
  Eigen::Isometry3d last_master_pose_in_master_base_;   ///< \f$ x_m[k-1]\f$
  Eigen::Isometry3d last_master_pose_mapped_into_follower_base_;   ///< \f$ \mathcal{M}(x_m[k-1])\f$

  Eigen::Matrix<double, 6, 6> tmp_6x6_matrix_;

  // -------------    Misc. functions    ---------------------
  /**
   * @brief Internal method to send verbose msgs to the ros logger
   */
  void info_verbose(const std::string & msg)  const;

  /**
   * @brief Internal method used to map the twist and wrenches
   *
   */
  void map_tensor(
    const Eigen::Matrix3d & orientation,
    const Eigen::Matrix<double, 6, 1> & in,
    Eigen::Matrix<double, 6, 1> & out) const;
};

}  // namespace cartesian_vic_teleop_controller

#endif  // CARTESIAN_VIC_TELEOP_CONTROLLER__MAPPING_MANAGER_HPP_
