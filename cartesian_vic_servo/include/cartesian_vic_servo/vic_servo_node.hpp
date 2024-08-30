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


#ifndef CARTESIAN_VIC_SERVO__VIC_SERVO_NODE_HPP_
#define CARTESIAN_VIC_SERVO__VIC_SERVO_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "cartesian_vic_controller/measurement_data.hpp"
#include "cartesian_vic_controller/cartesian_vic_rule.hpp"

#include "cartesian_control_msgs/msg/compliant_frame_trajectory.hpp"
#include "cartesian_control_msgs/msg/vic_controller_state.hpp"

namespace cartesian_vic_servo
{

class CartesianVicServo : public rclcpp::Node
{
public:
  explicit CartesianVicServo(std::string node_name = "cartesian_vic_servo_node");
  ~CartesianVicServo() = default;

  bool init();

  bool start();

  bool stop();

  bool update();

protected:
    // void callback_new_joint_state_msg(...);
    // + wrench (StampedWrench ou Wrench)
    // + vic ref (voir cartesian_control_msgs)

    // bool send_servo_command(const Eigen::Vector<double, 6> & twist_cmd);

protected:
    /// @brief True if init() called successfully
  bool is_initialized_ = false;

    /// @brief True when ready to compute VIC commands
  bool is_ready_ = false;

  // number of robot controlled joints
  size_t num_joints_ = 0;

  // Vic rule loader
  std::shared_ptr<pluginlib::ClassLoader<cartesian_vic_controller::CartesianVicRule>>
  vic_loader_;

  // Vic rule
  std::unique_ptr<cartesian_vic_controller::CartesianVicRule> vic_;
  bool is_vic_initialized_ = false;

  // vic parameters
  std::shared_ptr<cartesian_vic_controller::ParamListener> parameter_handler_;

  // measurement data used by the vic controller
  //   - joint state: current joint readings from the hardware
  //   - measured wrench: values read from the ft sensor (optional)
  //   - external torques: values read from the external torque sensor (optional)
  cartesian_vic_controller::MeasurementData measurement_data_{0};

  sensor_msgs::msg::JointState joint_state_msg_;
  geometry_msgs::msg::WrenchStamped wrench_msg_;
  cartesian_control_msgs::msg::CompliantFrameTrajectory vic_trajectory_msg_;

  // Subscriber joint state
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
    joint_state_subscriber_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<
      sensor_msgs::msg::JointState>> rt_buffer_joint_state_;
  std::shared_ptr<sensor_msgs::msg::JointState> joint_state_msg_ptr_;

  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr
    subscriber_wrench_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<
      geometry_msgs::msg::WrenchStamped>> rt_buffer_wrench_;
  std::shared_ptr<geometry_msgs::msg::WrenchStamped> wrench_msg_ptr_;

  rclcpp::Subscription<cartesian_control_msgs::msg::CompliantFrameTrajectory>::SharedPtr
    subscriber_vic_trajectory_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<
      cartesian_control_msgs::msg::CompliantFrameTrajectory>> rt_buffer_vic_trajectory_;
  std::shared_ptr<cartesian_control_msgs::msg::CompliantFrameTrajectory> vic_trajectory_msg_ptr_;

  // Publishers
  std::shared_ptr<rclcpp::Publisher<cartesian_control_msgs::msg::VicControllerState>>
  publisher_vic_state_;
  std::shared_ptr<realtime_tools::RealtimePublisher<
      cartesian_control_msgs::msg::VicControllerState>> rt_publisher_vic_state;

  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::TwistStamped>> publisher_twist_;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>>
  rt_publisher_twist_;

  bool update_measurement_data();

  bool get_joint_state(
    sensor_msgs::msg::JointState & msg,
    double timeout = 0.01 /*seconds*/);

  bool get_wrench(
    geometry_msgs::msg::WrenchStamped & msg,
    double timeout = 0.01 /*seconds*/);

  bool get_vic_trajectory(
    cartesian_control_msgs::msg::CompliantFrameTrajectory & msg,
    double timeout = 0.01 /*seconds*/);
};

}  // namespace cartesian_vic_servo

#endif  // CARTESIAN_VIC_SERVO__VIC_SERVO_NODE_HPP_
