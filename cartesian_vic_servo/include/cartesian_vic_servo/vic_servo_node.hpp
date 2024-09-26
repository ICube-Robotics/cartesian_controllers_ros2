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

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <moveit_servo/servo.hpp>
#include <moveit_servo/utils/common.hpp>

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit_msgs/srv/servo_command_type.hpp"
#include "moveit_msgs/msg/servo_status.hpp"
#include "std_srvs/srv/set_bool.hpp"


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
  /// @brief True if init() called successfully
  bool is_initialized_ = false;

  /// @brief True when ready to compute VIC commands
  bool is_running_ = false;

  /// @brief number of robot controlled joints
  size_t num_joints_ = 0;

  /// @brief name of the robot controlled joints
  std::vector<std::string> joint_names_;

  /// @brief Name of the base frame where the wrench and commands are expressed
  std::string base_frame_;

  /// @brief controller sampling time in seconds
  double Ts_;

  // timer
  rclcpp::TimerBase::SharedPtr timer_;

  // ---------- VIC ----------

  /// Vic rule loader
  std::shared_ptr<pluginlib::ClassLoader<cartesian_vic_controller::CartesianVicRule>>
  vic_loader_;

  /// Vic rule
  std::unique_ptr<cartesian_vic_controller::CartesianVicRule> vic_;
  bool is_vic_initialized_ = false;

  /// vic parameters
  std::shared_ptr<cartesian_vic_controller::ParamListener> parameter_handler_;

  // measurement data used by the vic controller
  //   - joint state: current joint readings from the hardware
  //   - measured wrench: values read from the ft sensor (optional)
  cartesian_vic_controller::MeasurementData measurement_data_{0};

  sensor_msgs::msg::JointState joint_state_msg_;
  geometry_msgs::msg::WrenchStamped wrench_msg_;
  cartesian_control_msgs::msg::CompliantFrameTrajectory vic_trajectory_msg_;

  // ---------- MoveIt servo ----------

  // MoveIt Servo instance
  std::unique_ptr<moveit_servo::Servo> servo_;
  servo::Params servo_params_;

  // Planning scene monitor
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  // Pointer towards the current robot state
  moveit::core::RobotStatePtr robot_state_;

  // Joint model group interface name
  std::string joint_model_group_name_;

  // Joint command queue (rolling window)
  std::deque<moveit_servo::KinematicState> joint_cmd_rolling_window_;


  /// @brief maximum expected latency in seconds (N.B., >= 3*Ts)
  double max_expected_latency_;

  /// @brief whether we use trajectory command or not
  bool use_trajectory_cmd_;

  // ---------- ROS2 communication ----------

  // Subscribers
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

  bool ref_has_been_received_in_the_past_ = false;

  // Publishers
  std::shared_ptr<rclcpp::Publisher<cartesian_control_msgs::msg::VicControllerState>>
  publisher_vic_state_;
  std::shared_ptr<realtime_tools::RealtimePublisher<
      cartesian_control_msgs::msg::VicControllerState>> rt_publisher_vic_state;
  cartesian_control_msgs::msg::VicControllerState vic_state_;

  std::shared_ptr<rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>> publisher_joint_cmd_;
  std::shared_ptr<realtime_tools::RealtimePublisher<trajectory_msgs::msg::JointTrajectory>>
  rt_publisher_joint_cmd_;

  /// fallback for robot description ROS parameters
  // TODO(tpoignonec): make this a parameter
  std::string robot_description_node_ = "robot_state_publisher";

  // ---------- Utils ----------

  bool reorder_joint_state(
    const sensor_msgs::msg::JointState & joint_state_msg,
    sensor_msgs::msg::JointState & joint_state_msg_reordered);

  bool update_measurement_data();

  bool send_twist_command(const geometry_msgs::msg::TwistStamped & twist_cmd);

  bool get_joint_state(
    sensor_msgs::msg::JointState & msg,
    double timeout = 0.01 /*seconds*/);

  bool get_wrench(
    geometry_msgs::msg::WrenchStamped & msg,
    double timeout = 0.01 /*seconds*/);

  bool get_vic_trajectory(
    cartesian_control_msgs::msg::CompliantFrameTrajectory & msg,
    double timeout = 0.01 /*seconds*/);

  std::string getUrdfFromServer() const;
};

}  // namespace cartesian_vic_servo

#endif  // CARTESIAN_VIC_SERVO__VIC_SERVO_NODE_HPP_
