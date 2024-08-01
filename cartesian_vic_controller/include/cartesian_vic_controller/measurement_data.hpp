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

#ifndef CARTESIAN_VIC_CONTROLLER__MEASUREMENT_DATA_HPP_
#define CARTESIAN_VIC_CONTROLLER__MEASUREMENT_DATA_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

namespace cartesian_vic_controller
{

/** @brief Class used to store the measurement data
 *
 * Used to pass the data to the Vic rule. The measurement data is
 * at least the joint position and velocity. Optionally, the external
 * joint torques and the ft sensor wrench can be passed.
 *
 */
class MeasurementData
{
private:
  size_t num_joints_;
  bool has_external_torques_data_;
  bool has_ft_sensor_data_;

  geometry_msgs::msg::Wrench measured_wrench_;
  std::vector<double> external_torques_;

public:
  trajectory_msgs::msg::JointTrajectoryPoint joint_state;

public:
  explicit MeasurementData(size_t num_joints)
  {
    num_joints_ = num_joints;
    joint_state.positions.assign(num_joints_, 0.0);  // std::nan);
    joint_state.velocities.assign(num_joints_, 0.0);  //  std::nan);
    joint_state.accelerations.assign(num_joints_, 0.0);  //  std::nan);
    joint_state.effort.reserve(num_joints_);  // not used
    external_torques_.assign(num_joints_, 0.0);
    reset_data_availability();
  }
  ~MeasurementData() = default;

  const trajectory_msgs::msg::JointTrajectoryPoint & get_joint_state() const
  {
    return joint_state;
  }
  const geometry_msgs::msg::Wrench & get_ft_sensor_wrench() const
  {
    return measured_wrench_;
  }
  const std::vector<double> & get_external_torques() const
  {
    return external_torques_;
  }
  bool has_ft_sensor_data() const
  {
    return has_ft_sensor_data_;
  }
  bool has_external_torques_data() const
  {
    return has_external_torques_data_;
  }

    /// Reset all values back to default
  bool reset_data_availability()
  {
    has_external_torques_data_ = false;
    has_ft_sensor_data_ = false;
    return true;
  }

  bool update_joint_state(const trajectory_msgs::msg::JointTrajectoryPoint & joint_state_values)
  {
    if (joint_state_values.positions.size() != num_joints_ ||
      joint_state_values.velocities.size() != num_joints_)
    {
      return false;
    }
    joint_state = joint_state_values;
    return true;
  }
  bool update_ft_sensor_wrench(const geometry_msgs::msg::Wrench & measured_wrench)
  {
    measured_wrench_ = measured_wrench;
    has_ft_sensor_data_ = true;
    return true;
  }
  bool update_external_torques(const std::vector<double> & external_torques)
  {
    external_torques_ = external_torques;
    has_external_torques_data_ = true;
    return true;
  }
};

}  // namespace cartesian_vic_controller


#endif  // CARTESIAN_VIC_CONTROLLER__MEASUREMENT_DATA_HPP_
