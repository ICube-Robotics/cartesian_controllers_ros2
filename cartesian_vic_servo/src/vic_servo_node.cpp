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

#include "cartesian_vic_servo/vic_servo_node.hpp"

// #include "rcutils/logging_macros.h"
#include "rclcpp/logging.hpp"

namespace cartesian_vic_servo
{

CartesianVicServo::CartesianVicServo(std::string node_name)
: Node(node_name)
{
  // Nothing to do
}
bool CartesianVicServo::init()
{
  // Setup joint state subscribers
  auto joint_state_callback =
    [this](const std::shared_ptr<sensor_msgs::msg::JointState> msg)
    {rt_buffer_joint_state_.writeFromNonRT(msg);};
  joint_state_subscriber_ = \
    this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 1,
      joint_state_callback);

  /*
  subscriber_wrench_ = \
    this->create_subscription<geometry_msgs::msg::Wrench>();
  subscriber_vic_trajectory_ = \
    this->create_subscription<cartesian_control_msgs::msg::CompliantFrameTrajectory>();
  */

  // Publishers
  /*
  publisher_vic_state_ = this->create_publisher<cartesian_control_msgs::msg::VicControllerState>("input_vic_state", 1);
  publisher_twist_ = this->create_publisher<geometry_msgs::msg::Twist>("/twist_command", 1);
  */

  /*
  state_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<ControllerStateMsg>>(s_publisher_);

  // How to use:
  state_publisher_->lock();
  // state_publisher_->msg_ = vic_->get_controller_state();
  state_publisher_->unlock();
  */
  return true;
}

bool CartesianVicServo::start()
{
  return false;
}

bool CartesianVicServo::stop()
{
  return false;
}

bool CartesianVicServo::update()
{
  return false;
}

// -----------------------------------------------------
//  Subscribers callbacks
// -----------------------------------------------------

bool CartesianVicServo::get_joint_state(
  sensor_msgs::msg::JointState & joint_state_msg,
  double timeout /*seconds*/)
{
  // Get msg from RT buffer
  joint_state_msg_ptr_ = \
    *rt_buffer_joint_state_.readFromRT();
  if (joint_state_msg_ptr_.get()) {
    joint_state_msg = *joint_state_msg_ptr_.get();
  } else {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to get joint state message!");
    return false;
  }

  // Check timeout
  double delay = (this->now() - joint_state_msg.header.stamp).nanoseconds();
  if (delay > timeout * 1e9) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Timeout on joint state message!");
    return false;
  }

  return true;
}
// -----------------------------------------------------
// Utils
// -----------------------------------------------------

}  // namespace cartesian_vic_servo

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cartesian_vic_servo::CartesianVicServo>());
  rclcpp::shutdown();
  return 0;
}
