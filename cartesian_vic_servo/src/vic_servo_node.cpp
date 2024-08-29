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
  // Setup joint state subscriber
  auto joint_state_callback =
    [this](const std::shared_ptr<sensor_msgs::msg::JointState> msg)
    {rt_buffer_joint_state_.writeFromNonRT(msg);};
  joint_state_subscriber_ = \
    this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 1,
      joint_state_callback);


  // Setup wrench  subscriber
  auto wrench_callback =
    [this](const std::shared_ptr<geometry_msgs::msg::WrenchStamped> msg)
    {rt_buffer_wrench_.writeFromNonRT(msg);};
  subscriber_wrench_ = \
    this->create_subscription<geometry_msgs::msg::WrenchStamped>("wrench", 1, wrench_callback);


  // Setup vic trajectory subscriber
  auto vic_trajectory_callback =
    [this](const std::shared_ptr<cartesian_control_msgs::msg::CompliantFrameTrajectory> msg)
    {rt_buffer_vic_trajectory_.writeFromNonRT(msg);};
  subscriber_vic_trajectory_ = \
    this->create_subscription<cartesian_control_msgs::msg::CompliantFrameTrajectory>(
      "vic_trajectory", 1, vic_trajectory_callback);

  // Publishers
  publisher_vic_state_ =
    this->create_publisher<cartesian_control_msgs::msg::VicControllerState>("input_vic_state", 1);
  publisher_twist_ = this->create_publisher<geometry_msgs::msg::Twist>("/twist_command", 1);

  // Realtime publisher
  rt_publisher_vic_state =
    std::make_unique<realtime_tools::RealtimePublisher<
        cartesian_control_msgs::msg::VicControllerState>>(publisher_vic_state_);

  rt_publisher_twist_ = std::make_unique<realtime_tools::RealtimePublisher<
        geometry_msgs::msg::Twist>>(publisher_twist_);

  // initialize VIC rule plugin
  try {
    parameter_handler_ =
      std::make_shared<cartesian_vic_controller::ParamListener>(shared_from_this());
    cartesian_vic_controller::Params parameters = parameter_handler_->get_params();
    // number of joints in controllers is fixed after initialization
    num_joints_ = parameters.joints.size();
    RCLCPP_INFO(get_logger(), "Configuring controller with %li joints", num_joints_);
    // allocate dynamic memory
    measurement_data_ = cartesian_vic_controller::MeasurementData(parameters.joints.size());
    if (!parameters.vic.plugin_name.empty() &&
      !parameters.vic.plugin_package.empty())
    {
      vic_loader_ =
        std::make_shared<pluginlib::ClassLoader<
            cartesian_vic_controller::CartesianVicRule>>(
        parameters.vic.plugin_package,
        "cartesian_vic_controller::CartesianVicRule"
            );
      vic_ = std::unique_ptr<cartesian_vic_controller::CartesianVicRule>(
        vic_loader_->createUnmanagedInstance(parameters.vic.plugin_name));
    } else {
      RCLCPP_ERROR(
        get_logger(),
        "Please provide 'vic.plugin_package' and 'vic.plugin_name' parameters!");
      return false;
    }
    // Initialize vic rule plugin
    if (vic_->init(parameter_handler_) == controller_interface::return_type::ERROR) {
      return false;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_logger(), "Exception thrown during configure stage with message: %s \n",
      e.what());
    return false;
  }
  return true;
}

bool CartesianVicServo::start()
{
  // TODO(dmeckes): connect to moveit servo

  // TODO(dmeckes): start timer with update() as callback
  return false;
}

bool CartesianVicServo::stop()
{
  // TODO(dmeckes): stop timer

  // TODO(dmeckes): send twist = 0 to moveit servo

  // TODO(dmeckes): stop moveit servo

  // TODO(dmeckes): disconnect from moveit servo
  return false;
}

bool CartesianVicServo::update()
{
  // TODO(dmeckes): move to init() (+ make those node parameters?)
  double timeout = 0.01;
  double Ts = 0.005;  // 200 Hz

  // -----------------------
  // Prepare data for vic
  // -----------------------
  bool is_state_valid = true;
  // Get joint state
  is_state_valid &= get_joint_state(joint_state_msg_, timeout);
  // Get wrench
  is_state_valid &= get_wrench(wrench_msg_, timeout);
  // Get vic trajectory
  is_state_valid &= get_vic_trajectory(vic_trajectory_msg_, timeout);

  if (is_state_valid) {
    is_state_valid &= update_measurement_data();
  } else {
    // Invalid state detected!
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to get state message!");
  }
  // -----------------------
  // Run vic
  // -----------------------
  geometry_msgs::msg::Twist twist_cmd;
  // twist_cmd.header.stamp = this->now();
  // twist_cmd.header.frame_id = base_link_;
  bool all_ok = is_state_valid;
  if (!is_vic_initialized_ && !is_state_valid) {
    // Exit and wait for valid data...
    all_ok = false;
  } else if (!is_vic_initialized_ && is_state_valid) {
    // Init current desired pose from current joint position
    is_vic_initialized_ = (vic_->init_reference_frame_trajectory(
        measurement_data_.get_joint_state()) == controller_interface::return_type::OK);
    if(!is_vic_initialized_) {
      all_ok = false;
      RCLCPP_ERROR(
      get_logger(),
      "Failed to initialize the reference compliance frame trajectory.");
    }
  }

  auto execute_fallback_policy = [this] ()
    {
      RCLCPP_ERROR(
    get_logger(),
    "Fallback policy triggered! Setting velocity / force controls to zero!");

    // TODO(dmeckes): send twist_cmd = 0
    };

  // Send zero twist if not all OK
  if(!all_ok) {
    execute_fallback_policy();
    return false;
  }

  // Otherwise, proceed to control logic
  // 1) Update vic
  rclcpp::Duration period = rclcpp::Duration::from_seconds(Ts);
  auto ret_vic = vic_->update_input_data(
      period,
      measurement_data_
  );
  if (ret_vic != controller_interface::return_type::OK) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to update VIC!");
    all_ok = false;
  }
  ret_vic = vic_->compute_controls(period, twist_cmd);
  if (ret_vic != controller_interface::return_type::OK) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to get commands from VIC rule!");
    all_ok = false;
  }

  // Send zero twist if not all OK
  if(!all_ok) {
    execute_fallback_policy();
    return false;
  }

  // 2) Send twist command

  // TODO(dmeckes): check that moveit servo is running

  // TODO(dmeckes): send the twist cmd to moveit servo

  // 3) Send controller VIC state

  // TODO(dmeckes): send the state msg

  return true;
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

bool CartesianVicServo::get_wrench(
  geometry_msgs::msg::WrenchStamped & wrench_msg,
  double timeout /*seconds*/)
{
  // Get msg from RT buffer
  wrench_msg_ptr_ = \
    *rt_buffer_wrench_.readFromRT();
  if (wrench_msg_ptr_.get()) {
    wrench_msg = *wrench_msg_ptr_.get();
  } else {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to get wrench message!");
    return false;
  }

  // Check timeout
  double delay = (this->now() - wrench_msg.header.stamp).nanoseconds();
  if (delay > timeout * 1e9) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Timeout on wrench message!");
    return false;
  }
  return true;
}

bool CartesianVicServo::get_vic_trajectory(
  cartesian_control_msgs::msg::CompliantFrameTrajectory & vic_trajectory_msg,
  double timeout /*seconds*/)
{
  // Get msg from RT buffer
  vic_trajectory_msg_ptr_ = \
    *rt_buffer_vic_trajectory_.readFromRT();
  if (vic_trajectory_msg_ptr_.get()) {
    vic_trajectory_msg = *vic_trajectory_msg_ptr_.get();
  } else {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to get vic trajectory message!");
    return false;
  }

  // Check timeout
  double delay = (this->now() - vic_trajectory_msg.header.stamp).nanoseconds();
  if (delay > timeout * 1e9) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Timeout on vic trajectory message!");
    return false;
  }

  return true;
}


// -----------------------------------------------------
// Utils
// -----------------------------------------------------

bool CartesianVicServo::update_measurement_data()
{
  measurement_data_.reset_data_availability();
  bool all_ok = true;

  auto & state_current = measurement_data_.joint_state;
  bool state_has_nan = false;
  for (size_t joint_ind = 0; joint_ind < num_joints_; ++joint_ind) {
    state_current.positions[joint_ind] = \
      joint_state_msg_.position[joint_ind];
    state_current.velocities[joint_ind] = \
      joint_state_msg_.velocity[joint_ind];
    // Detect NaNs
    state_has_nan &= std::isnan(state_current.positions[joint_ind]);
    state_has_nan &= std::isnan(state_current.velocities[joint_ind]);
  }
  if (state_has_nan) {
    all_ok = false;
    RCLCPP_ERROR(
      this->get_logger(),
      "NaN detected in state message!");
  }
  // Update ft sensor wrench
  all_ok &= measurement_data_.update_ft_sensor_wrench(wrench_msg_.wrench);

  return all_ok;
}

}  // namespace cartesian_vic_servo

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cartesian_vic_servo::CartesianVicServo>());
  rclcpp::shutdown();
  return 0;
}
