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

#ifndef CARTESIAN_VIC_CONTROLLER__EXTERNAL_TORQUE_SENSOR_HPP_
#define CARTESIAN_VIC_CONTROLLER__EXTERNAL_TORQUE_SENSOR_HPP_

#include <limits>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_state_interface.hpp"

namespace cartesian_vic_controller
{

class ExternalTorqueSensor
{
public:
  explicit ExternalTorqueSensor(const std::string & name, size_t num_joints)
  {
    name_ = name;
    num_joints_ = num_joints;
    interface_names_.reserve(num_joints_);
    state_interfaces_.reserve(num_joints_);
    torques_.resize(num_joints_);
    existing_axes_.resize(num_joints_);

    // Set all interface names
    for (size_t i = 0; i < num_joints_; ++i) {
      interface_names_.emplace_back(name_ + "/" + "external_torque.joint_a" + std::to_string(i));
    }

    // Set all interfaces existing
    std::fill(existing_axes_.begin(), existing_axes_.end(), true);

    // Set default force and torque values to NaN
    std::fill(torques_.begin(), torques_.end(), std::numeric_limits<double>::quiet_NaN());
  }

  ~ExternalTorqueSensor() = default;

  bool assign_loaned_state_interfaces(
    std::vector<hardware_interface::LoanedStateInterface> & state_interfaces)
  {
    return controller_interface::get_ordered_interfaces(
      state_interfaces, interface_names_, "", state_interfaces_);
  }

  void release_interfaces() {state_interfaces_.clear();}

  bool get_values(std::vector<double> & values) const
  {
    // check we have sufficient memory
    if (values.capacity() != state_interfaces_.size()) {
      return false;
    }
    // insert all the values
    for (size_t i = 0; i < state_interfaces_.size(); ++i) {
      values.emplace_back(state_interfaces_[i].get().get_value());
    }
    return true;
  }

  std::vector<std::string> get_state_interface_names()
  {
    if (interface_names_.empty()) {
      for (auto i = 0u; i < interface_names_.capacity(); ++i) {
        interface_names_.emplace_back(name_ + "/" + std::to_string(i + 1));
      }
    }
    return interface_names_;
  }

  explicit ExternalTorqueSensor(const std::vector<std::string> & interfaces_external_torque)
  {
    name_ = "";
    num_joints_ = interfaces_external_torque.size();

    auto check_and_add_interface = [this](const std::string & interface_name, const int index) {
        if (!interface_name.empty()) {
          interface_names_.emplace_back(interface_name);
          existing_axes_[index] = true;
        } else {
          existing_axes_[index] = false;
        }
      };
    for (size_t i = 0; i < num_joints_; ++i) {
      check_and_add_interface(interfaces_external_torque[i], i);
    }

    // Set default torque values to NaN
    std::fill(torques_.begin(), torques_.end(), std::numeric_limits<double>::quiet_NaN());
  }

  std::vector<double> get_torques()
  {
    auto torque_interface_counter = 0;
    for (size_t i = 0; i < num_joints_; ++i) {
      if (existing_axes_[i]) {
        torques_[i] = state_interfaces_[torque_interface_counter].get().get_value();
        ++torque_interface_counter;
      }
    }
    return torques_;
  }

protected:
  std::string name_;
  size_t num_joints_;
  std::vector<std::string> interface_names_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> state_interfaces_;
  std::vector<double> torques_;
  std::vector<bool> existing_axes_;
};

}  // namespace cartesian_vic_controller

#endif  // CARTESIAN_VIC_CONTROLLER__EXTERNAL_TORQUE_SENSOR_HPP_
