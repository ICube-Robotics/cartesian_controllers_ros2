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

#ifndef CARTESIAN_STATE_BROADCASTER__CARTESIAN_STATE_BROADCASTER_HPP_
#define CARTESIAN_STATE_BROADCASTER__CARTESIAN_STATE_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "cartesian_state_broadcaster/visibility_control.h"
#include "cartesian_msgs/msg/cartesian_state.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.h"
#include <Eigen/Geometry> 

// kinematics plugins
#include "kinematics_interface/kinematics_interface_base.hpp"
#include "pluginlib/class_loader.hpp"

namespace cartesian_state_broadcaster
{
using CallbackReturn = controller_interface::CallbackReturn;
using StateMsg = cartesian_msgs::msg::CartesianState;

class CartesianStateBroadcaster : public controller_interface::ControllerInterface
{
public:
    CARTESIAN_STATE_BROADCASTER_PUBLIC
    CartesianStateBroadcaster();

    CARTESIAN_STATE_BROADCASTER_PUBLIC
    CallbackReturn on_init() override;

    CARTESIAN_STATE_BROADCASTER_PUBLIC
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    CARTESIAN_STATE_BROADCASTER_PUBLIC
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    CARTESIAN_STATE_BROADCASTER_PUBLIC
    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

    CARTESIAN_STATE_BROADCASTER_PUBLIC
    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    CARTESIAN_STATE_BROADCASTER_PUBLIC
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    CARTESIAN_STATE_BROADCASTER_PUBLIC
    controller_interface::return_type update(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;

protected:
    std::string end_effector_name_;
    std::vector<std::string> joint_names_;
    std::vector<double> joint_positions_;
    std::vector<double> joint_velocities_;
    std::vector<double> cart_trafo_vec_;
    Eigen::Matrix<double, 4, 4> cart_trafo_;
    Eigen::Quaterniond cart_quat_;
    std::vector<double> cart_velocities_;

    // Kinematics interface plugin loader
    std::shared_ptr<pluginlib::ClassLoader<kinematics_interface::KinematicsBaseClass>> kinematics_loader_;
    std::unique_ptr<kinematics_interface::KinematicsBaseClass> kinematics_;

    using StatePublisher = realtime_tools::RealtimePublisher<StateMsg>;
    rclcpp::Publisher<StateMsg>::SharedPtr state_publisher_;
    std::unique_ptr<StatePublisher> realtime_publisher_;

    template<typename T1, typename T2>
    void vec_to_eigen(const std::vector<T1>& data, T2 &matrix);

    template<typename T1, typename T2>
    void eigen_to_vec(const T2 &matrix, std::vector<T1>& data);
};

}  // namespace cartesian_state_broadcaster

#endif  // CARTESIAN_STATE_BROADCASTER__CARTESIAN_STATE_BROADCASTER_HPP_
