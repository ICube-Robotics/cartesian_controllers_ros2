#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/Twist.hpp"
#include "geometry_msgs/msg/Wrench.hpp"
#include "sensor_msgs/msg/JointState.hpp"
#include "cartesian_control_msgs/msg/compliant_frame_trajectory.hpp"
#include "cartesian_control_msgs/msg/vic_controller_state.hpp"



bool CartesianVicServo::init()
{
  // Subscribers
  subscriber_joint_state_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_state_topic", 1, jointState_callback);
  subscriber_wrench_ = this->create_subscription<geometry_msgs::msg::Wrench>();
  subscriber_vic_trajectory_ = this->create_subscription<cartesian_control_msgs::msg::ComplianceFrameTrajectory>();


  // Publishers
  publisher_vic_state_ = this->create_publisher<cartesian_control_msgs::msg::VicControllerState>("input_vic_state", 1);
  publisher_twist_ = this->create_publisher<geometry_msgs::msg::Twist>("/twist_command", 1);

  return true;
}

auto jointState_callback(sensor_msgs::msg::JointState msg)
{
  this->jointState = msg;

}