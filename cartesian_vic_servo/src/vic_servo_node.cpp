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

#include "rcutils/logging_macros.h"
#include "rclcpp/logging.hpp"
#include <realtime_tools/thread_priority.hpp>

namespace cartesian_vic_servo
{

using namespace std::chrono_literals;

CartesianVicServo::CartesianVicServo(std::string node_name)
: Node(node_name)
{
  // Configure SCHED_FIFO and priority
  int thread_priority = 40;  // High priority for this control thread
  if (realtime_tools::configure_sched_fifo(thread_priority)) {
    RCLCPP_INFO_STREAM(get_logger(), "Enabled SCHED_FIFO and higher thread priority.");
  } else {
    RCLCPP_WARN_STREAM(get_logger(),
        "Could not enable FIFO RT scheduling policy. Continuing with the default.");
  }

  // Check if a realtime kernel is available
  if (!realtime_tools::has_realtime_kernel()) {
    RCLCPP_WARN_STREAM(get_logger(), "Realtime kernel is recommended for better performance.");
  }

  is_initialized_ = false;
  is_running_ = false;
}

bool CartesianVicServo::init()
{
  if (is_initialized_) {
    RCLCPP_WARN(get_logger(), "CartesianVicServo is already initialized...");
    return true;
  }
  // Setup parameter handler
  try {
    parameter_handler_ =
      std::make_shared<cartesian_vic_controller::ParamListener>(shared_from_this());
  } catch (const std::exception & e) {
    std::string error_msg = std::string(e.what());
    RCLCPP_ERROR(
      get_logger(),
      "Caught exception while initializing parameter handler: %s",
      error_msg.c_str());
    return false;
  }


  // Try to retrieve urdf (used by kinematics / dynamics plugin)
  RCLCPP_INFO(get_logger(), "Trying to retrieve 'robot_description' parameter...");

  if (!this->get_node_parameters_interface()->has_parameter("robot_description")) {
    rcl_interfaces::msg::ParameterDescriptor urdf_param_desc;
    urdf_param_desc.name = "rule_plugin_package";
    urdf_param_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    urdf_param_desc.description = "URDF description of the robot";
    this->get_node_parameters_interface()->declare_parameter(
      "robot_description", rclcpp::ParameterValue(""), urdf_param_desc);
  }
  auto urdf_param = rclcpp::Parameter();
  if (!this->get_node_parameters_interface()->get_parameter("robot_description", urdf_param)) {
    RCLCPP_ERROR(get_logger(), "parameter 'robot_description' not set");
    return false;
  }
  std::string urdf_string = urdf_param.as_string();

  if (urdf_string.empty()) {
    RCLCPP_ERROR(
      get_logger(),
      "Could not find 'robot_description' parameter! Trying to retrieve URDF from param server...");
    // TODO(tpoignonec): get URDF from param server
    urdf_string = getUrdfFromServer();
    if (urdf_string.empty()) {
      RCLCPP_ERROR(get_logger(), "Could not retrieve URDF from param server!");
      return false;
    } else {
      rclcpp::Parameter robot_description_param("robot_description", urdf_string);
      this->set_parameter(robot_description_param);
    }
  }

  // load parameters to be used by the VIC rule
  if (!parameter_handler_) {
    RCLCPP_ERROR(get_logger(), "Parameter handler not initialized!");
    return false;
  }
  cartesian_vic_controller::Params parameters = parameter_handler_->get_params();

  // number of joints in controllers is fixed after initialization
  num_joints_ = parameters.joints.size();
  joint_names_ = parameters.joints;
  RCLCPP_INFO(get_logger(), "Configuring controller with %li joints", num_joints_);

  // frame_id used for the null twist
  base_frame_ = parameters.dynamics.base;

  // allocate dynamic memory
  measurement_data_ = cartesian_vic_controller::MeasurementData(parameters.joints.size());

  // ----------------------- VIC -----------------------
  // Initialize VIC rule plugin
  try {
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
      RCLCPP_ERROR(get_logger(), "Failed to initialize VIC rule plugin!");
      return false;
    }
  } catch (const std::exception & e) {
    std::string error_msg = std::string(e.what());
    RCLCPP_ERROR(
      get_logger(), "Exception thrown during configure stage with message: %s \n",
      error_msg.c_str());
    return false;
  }

  // Configure VIC rule
  if (vic_->configure(this->get_node_parameters_interface(),
      num_joints_) == controller_interface::return_type::ERROR)
  {
    return false;
  }

  // ----------------------- MoveIt servo -----------------------

  // Get the servo parameters.
  const std::string param_namespace = "moveit_servo";
  const std::shared_ptr<const servo::ParamListener> servo_param_listener =
    std::make_shared<const servo::ParamListener>(shared_from_this(), param_namespace);
  servo_params_ = servo_param_listener->get_params();

  try {
    // Create the servo object
    planning_scene_monitor_ = moveit_servo::createPlanningSceneMonitor(
      shared_from_this(), servo_params_);

    // Setup servo logic
    servo_ = std::make_unique<moveit_servo::Servo>(
      shared_from_this(), servo_param_listener, planning_scene_monitor_);

    // Initialize the robot state and joint model group
    robot_state_ = planning_scene_monitor_->getStateMonitor()->getCurrentState();
  } catch (const std::exception & e) {
    std::string error_msg = std::string(e.what());
    RCLCPP_ERROR(
      get_logger(),
      "Exception thrown during configure stage when setting up MoveIt servo with message: %s \n",
      error_msg.c_str());
    return false;
  }

  if (!robot_state_) {
    RCLCPP_ERROR(get_logger(), "Failed to get robot state from planning scene monitor!");
    return false;
  }

  // Set the command type for servo.
  servo_->setCommandType(moveit_servo::CommandType::TWIST);

  // Store relevant servoing parameters
  joint_model_group_name_ = servo_params_.move_group_name;
  Ts_ = servo_params_.publish_period;
  max_expected_latency_ = 3 * Ts_;
  use_trajectory_cmd_ = servo_params_.command_out_type == "trajectory_msgs/JointTrajectory";

  if (!use_trajectory_cmd_) {
    RCLCPP_ERROR(
      get_logger(),
      "Please set 'moveit_servo.command_out_type' to 'trajectory_msgs/JointTrajectory'!");
    return false;
  }
  // ----------------------- Communication -----------------------

  // Setup wrench  subscriber
  auto wrench_callback =
    [this](const std::shared_ptr<geometry_msgs::msg::WrenchStamped> msg)
    {rt_buffer_wrench_.writeFromNonRT(msg);};
  subscriber_wrench_ = \
    this->create_subscription<geometry_msgs::msg::WrenchStamped>("~/wrench", 1, wrench_callback);

  // Setup vic trajectory subscriber
  auto vic_trajectory_callback =
    [this](const std::shared_ptr<cartesian_control_msgs::msg::CompliantFrameTrajectory> msg)
    {rt_buffer_vic_trajectory_.writeFromNonRT(msg);};
  subscriber_vic_trajectory_ = \
    this->create_subscription<cartesian_control_msgs::msg::CompliantFrameTrajectory>(
      "~/reference_compliant_frame_trajectory", 1, vic_trajectory_callback);

  // Publishers
  publisher_vic_state_ =
    this->create_publisher<cartesian_control_msgs::msg::VicControllerState>(
      "~/status", rclcpp::SystemDefaultsQoS());
  publisher_joint_cmd_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    servo_params_.command_out_topic, rclcpp::SystemDefaultsQoS());

  // Realtime publisher
  rt_publisher_vic_state =
    std::make_unique<realtime_tools::RealtimePublisher<
        cartesian_control_msgs::msg::VicControllerState>>(publisher_vic_state_);

  rt_publisher_joint_cmd_ = std::make_unique<realtime_tools::RealtimePublisher<
        trajectory_msgs::msg::JointTrajectory>>(publisher_joint_cmd_);

  return true;
}

bool CartesianVicServo::start()
{
  // Initialize if not yet done
  if (!is_initialized_) {
    RCLCPP_WARN(get_logger(), "CartesianVicServo is not yet initialized...");
    is_initialized_ = this->init();
  }
  if (!is_initialized_) {
    RCLCPP_ERROR(get_logger(), "CartesianVicServo failed to initialize!");
    return false;
  }

  // Check if already started
  if (is_running_ && timer_) {
    RCLCPP_WARN(get_logger(), "CartesianVicServo is already running!");
    return true;
  }

  // Wait for joint state and planning scene
  size_t attempts = 0;
  size_t max_attemps = 10;
  while(!planning_scene_monitor_->getStateMonitor()->waitForCurrentState(
    rclcpp::Clock(RCL_ROS_TIME).now(), 1.0 /* seconds */))
  {
    RCLCPP_INFO(get_logger(), "Waiting for robot state...");
    ++attempts;
    if(attempts > max_attemps) {
      RCLCPP_ERROR(get_logger(), "Failed to get robot state after %d attempts!", attempts);
      return false;
    }
  }

  // Reset command queue
  auto current_state = servo_->getCurrentRobotState(false);
  moveit_servo::updateSlidingWindow(
    current_state, joint_cmd_rolling_window_, max_expected_latency_, this->now());

  // Start control loop
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(Ts_ * 1000)),
    std::bind(&CartesianVicServo::CartesianVicServo::update, this));
  if(timer_) {
    RCLCPP_INFO(get_logger(), "Control loop has started!");
    is_running_ = true;
    return true;
  }

  return false;
}

bool CartesianVicServo::stop()
{
  // TODO(dmeckes): stop timer
  timer_.reset();
  RCLCPP_ERROR(
        get_logger(),
        "Timer: stopped!");
  is_running_ = false;
  return true;
}

bool CartesianVicServo::update()
{
  // TODO(dmeckes): move to init() (+ make those node parameters?)
  double timeout = 0.01;
  is_running_ = true;

  // -----------------------
  // Prepare data for vic
  // -----------------------
  bool is_state_valid = true;
  // Get joint state
  is_state_valid &= get_joint_state(joint_state_msg_, timeout);
  // Get wrench
  is_state_valid &= get_wrench(wrench_msg_, timeout);

  if (is_state_valid) {
    is_state_valid &= update_measurement_data();
  } else {
    // Invalid state detected!
    RCLCPP_ERROR(
      get_logger(),
      "Failed to get state message!");
  }


  // Get vic trajectory
  bool is_vic_ref_valid = get_vic_trajectory(vic_trajectory_msg_, timeout);

  // -----------------------
  // Prepare fallback policy
  // -----------------------

  geometry_msgs::msg::TwistStamped twist_cmd;
  twist_cmd.header.stamp = this->now();
  twist_cmd.header.frame_id = vic_->get_input_data().base_frame;

  auto execute_fallback_policy = [this, &twist_cmd] () -> bool
    {
      RCLCPP_DEBUG(get_logger(), "Executing fallback policy...");
      // Send zero twist command
      twist_cmd.twist.linear.x = 0.0;
      twist_cmd.twist.linear.y = 0.0;
      twist_cmd.twist.linear.z = 0.0;
      twist_cmd.twist.angular.x = 0.0;
      twist_cmd.twist.angular.y = 0.0;
      twist_cmd.twist.angular.z = 0.0;
      if (!send_twist_command(twist_cmd)) {
        RCLCPP_ERROR(get_logger(), "Failed to send zero twist command!");
        return false;
      }
      return true;
    };

  // -----------------------
  // Run vic
  // -----------------------
  bool all_ok = is_state_valid;
  if (!is_vic_initialized_ && !is_state_valid) {
    // Exit and wait for valid data...
    auto clock = this->get_clock();
    RCLCPP_WARN_THROTTLE(
      get_logger(), *clock, 1000, "Waiting for valid data to init VIC plugin!");
    all_ok = false;
  } else if (!is_vic_initialized_ && is_state_valid) {
    RCLCPP_INFO(get_logger(), "Initializing VIC plugin...");
    ref_has_been_received_in_the_past_ = false;
    // Init current desired pose from current joint position
    is_vic_initialized_ = (vic_->init_reference_frame_trajectory(
        measurement_data_.get_joint_state()) == controller_interface::return_type::OK);
    if(!is_vic_initialized_) {
      all_ok = false;
      RCLCPP_ERROR(
      get_logger(),
      "Failed to initialize the reference compliance frame trajectory.");
    }
    RCLCPP_INFO(get_logger(), "VIC plugin is ready...");
  }

  // Send zero twist if not all OK
  if(!all_ok) {
    execute_fallback_policy();
    return false;
  }

  // Otherwise, proceed to control logic
  // 1) Update vic
  rclcpp::Duration period = rclcpp::Duration::from_seconds(Ts_);

  if (is_vic_ref_valid) {
    if (!ref_has_been_received_in_the_past_) {
      RCLCPP_INFO(get_logger(), "VIC ref has been received for the first time! Way to go :)");
    }
    ref_has_been_received_in_the_past_ = true;
    vic_->update_compliant_frame_trajectory(vic_trajectory_msg_);
  }
  if (!is_vic_ref_valid && !ref_has_been_received_in_the_past_) {
    // Cas 1: on attend de le recevoir pour la premier fois
    //    --> RCLCPP_WARN_THROTTLE avec msg "warning, didn't yet received..."
    // TODO(dmeckes): a faire
  }
  if (!is_vic_ref_valid && ref_has_been_received_in_the_past_) {
    // Cas 2: on a deja recu des refs avant!!! Donc erreur!!!
    RCLCPP_ERROR(get_logger(), "Invalid VIC ref trajectory!");
    execute_fallback_policy();
    return false;
  }

  auto ret_vic = vic_->update_input_data(
      period,
      measurement_data_
  );
  if (ret_vic != controller_interface::return_type::OK) {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to update VIC!");
    all_ok = false;
  }
  ret_vic = vic_->compute_controls(period, twist_cmd.twist);
  if (ret_vic != controller_interface::return_type::OK) {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to get commands from VIC rule!");
    all_ok = false;
  }

  // Send zero twist if not all OK
  if(!all_ok) {
    execute_fallback_policy();
    return false;
  }

  // 2) Compute and send twist command using moveit servo

  if (!send_twist_command(twist_cmd)) {
    RCLCPP_ERROR(get_logger(), "Failed to send twist command!");
    return false;
  }

  // 3) Send controller VIC state

  if(vic_->controller_state_to_msg(vic_state_) == controller_interface::return_type::OK) {
    rt_publisher_vic_state->lock();
    rt_publisher_vic_state->msg_ = vic_state_;
    rt_publisher_vic_state->unlockAndPublish();
  } else {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to retrieve state message");
  }

  return true;
}

// -----------------------------------------------------
//  Subscribers callbacks
// -----------------------------------------------------

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
      get_logger(),
      "Failed to get wrench message!");
    return false;
  }

  // TODO(dmeckes): check that the frame_id is correct

  // Check timeout
  double delay = (this->now() - wrench_msg.header.stamp).nanoseconds();
  if (delay > timeout * 1e9) {
    RCLCPP_ERROR(
      get_logger(),
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
    auto clock = this->get_clock();
    RCLCPP_WARN_THROTTLE(
      get_logger(), *clock, 1000, "Failed to get vic trajectory message!");
    return false;
  }

  // Check timeout
  double delay = (this->now() - vic_trajectory_msg.header.stamp).nanoseconds();
  if (delay > timeout * 1e9) {
    RCLCPP_ERROR(
      get_logger(),
      "Timeout on vic trajectory message!");
    return false;
  }

  return true;
}


// -----------------------------------------------------
// Utils
// -----------------------------------------------------

bool CartesianVicServo::reorder_joint_state(
  const sensor_msgs::msg::JointState & joint_state_msg,
  sensor_msgs::msg::JointState & joint_state_msg_reordered)
{
  if (joint_state_msg.name.size() != num_joints_) {
    RCLCPP_ERROR(
      get_logger(),
      "Number of joints in the joint state does not match that of the VIC controller!");
    return false;
  }

  // copy msg
  joint_state_msg_reordered = joint_state_msg;

  // reorder joint names and data
  for (size_t i = 0; i < num_joints_; ++i) {
    int idx_in_msg = std::find(
      joint_state_msg.name.begin(), joint_state_msg.name.end(),
        joint_names_[i]) - joint_state_msg.name.begin();
    joint_state_msg_reordered.name[i] = joint_state_msg.name[idx_in_msg];
    joint_state_msg_reordered.position[i] = joint_state_msg.position[idx_in_msg];
    joint_state_msg_reordered.velocity[i] = joint_state_msg.velocity[idx_in_msg];
    if (joint_state_msg.effort.size() > 0) {  // check if there is an effort field
      joint_state_msg_reordered.effort[i] = joint_state_msg.effort[idx_in_msg];
    }
    if (idx_in_msg != i) {
      RCLCPP_DEBUG(
        get_logger(),
        "Reordered joint state '%s' %lu (msg) -> %lu (internal model)",
        joint_state_msg.name[idx_in_msg], idx_in_msg, i);
    }
  }

  return true;
}

bool CartesianVicServo::get_joint_state(
  sensor_msgs::msg::JointState & joint_state_msg,
  double timeout /*seconds*/)
{
  const auto current_time = this->now();

  // Get joint state from planning scene monitor
  const moveit::core::JointModelGroup * joint_model_group = \
    robot_state_->getJointModelGroup(joint_model_group_name_);
  const auto joint_names = joint_model_group->getActiveJointModelNames();
  if (joint_names.size() != num_joints_) {
    RCLCPP_ERROR(
      get_logger(),
      "Number of joints in the joint model group does not match that of the VIC controller!");
    return false;
  }

  sensor_msgs::msg::JointState unordered_joint_state_msg;
  unordered_joint_state_msg.header.stamp =
    planning_scene_monitor_->getStateMonitor()->getCurrentStateTime();
  unordered_joint_state_msg.name = joint_names;
  robot_state_->copyJointGroupPositions(joint_model_group, unordered_joint_state_msg.position);
  robot_state_->copyJointGroupVelocities(joint_model_group, unordered_joint_state_msg.velocity);

  // Reorder joint state
  if (!reorder_joint_state(unordered_joint_state_msg, joint_state_msg)) {
    RCLCPP_ERROR(get_logger(), "Failed to reorder joint state!");
    return false;
  }

  // Check timeout
  double delay = (current_time - joint_state_msg.header.stamp).nanoseconds();
  auto clock = this->get_clock();
  if (delay > timeout * 1e9) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *clock,
      1000,
      "Timeout on joint state message!");
    return false;
  }

  return true;
}

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
      get_logger(),
      "NaN detected in state message!");
  }
  // Update ft sensor wrench
  all_ok &= measurement_data_.update_ft_sensor_wrench(wrench_msg_.wrench);

  return all_ok;
}


bool CartesianVicServo::send_twist_command(
  const geometry_msgs::msg::TwistStamped & twist_cmd)
{
  if (!servo_) {
    RCLCPP_ERROR(get_logger(), "Servo not initialized!");
    return false;
  }

  // Populate servo twist command
  moveit_servo::TwistCommand target_twist{base_frame_, {
      twist_cmd.twist.linear.x, twist_cmd.twist.linear.y, twist_cmd.twist.linear.z,
      twist_cmd.twist.angular.x, twist_cmd.twist.angular.y, twist_cmd.twist.angular.z}
  };

  // Get current joint state (actual state or current ref, depending on use_trajectory_cmd_)
  moveit_servo::KinematicState current_state;
  if (use_trajectory_cmd_ && !joint_cmd_rolling_window_.empty() &&
    joint_cmd_rolling_window_.back().time_stamp > now())
  {
    current_state = joint_cmd_rolling_window_.back();
  } else {
    // if joint_cmd_rolling_window_ is empty or all commands are outdated, use current robot state
    joint_cmd_rolling_window_.clear();
    current_state = servo_->getCurrentRobotState();
    current_state.velocities *= 0.0;
  }

  // update robot state values
  const moveit::core::JointModelGroup * joint_model_group = \
    robot_state_->getJointModelGroup(joint_model_group_name_);
  robot_state_->setJointGroupPositions(joint_model_group, current_state.positions);
  robot_state_->setJointGroupVelocities(joint_model_group, current_state.velocities);


  // Compute next joint state from twist command
  moveit_servo::KinematicState next_joint_state = servo_->getNextJointState(robot_state_,
      target_twist);
  const moveit_servo::StatusCode status = servo_->getStatus();

  if (status != moveit_servo::StatusCode::INVALID) {
    moveit_servo::updateSlidingWindow(
      next_joint_state, joint_cmd_rolling_window_, max_expected_latency_, this->now());
    if (const auto msg = moveit_servo::composeTrajectoryMessage(servo_params_,
        joint_cmd_rolling_window_))
    {
      rt_publisher_joint_cmd_->lock();
      rt_publisher_joint_cmd_->msg_ = msg.value();
      rt_publisher_joint_cmd_->unlockAndPublish();
    }
    if (!joint_cmd_rolling_window_.empty()) {
      const moveit::core::JointModelGroup * joint_model_group = \
        robot_state_->getJointModelGroup(joint_model_group_name_);
      robot_state_->setJointGroupPositions(joint_model_group,
          joint_cmd_rolling_window_.back().positions);
      robot_state_->setJointGroupVelocities(joint_model_group,
          joint_cmd_rolling_window_.back().velocities);
    }
  }

  return true;
}

std::string CartesianVicServo::getUrdfFromServer() const
{
  std::string urdf_string;

  using namespace std::chrono_literals;

  // Setup internal node for parameter client
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::Node::SharedPtr async_node;
  std::unique_ptr<std::thread> node_thread;

  RCLCPP_INFO(
    get_logger(),
    "Setting up internal parameters client... please wait...");
  rclcpp::NodeOptions options;
  std::string node_name =
    "cartesian_vic_controller_internal_parameters_client_" + std::to_string(std::rand());
  RCLCPP_INFO(get_logger(), "Internal node name: %s", node_name.c_str());
  options.arguments({"--ros-args", "-r", "__node:=" + node_name});
  async_node = rclcpp::Node::make_shared("_", options);
  node_thread = std::make_unique<std::thread>(
    [&]()
    {
      executor.add_node(async_node);
      executor.spin();
      executor.remove_node(async_node);
    });

  auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(
    async_node, robot_description_node_);
  while (!parameters_client->wait_for_service(0.5s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        get_logger(), "Interrupted while waiting for %s service. Exiting.",
        robot_description_node_.c_str());
      return 0;
    }
    RCLCPP_ERROR(
      get_logger(), "%s service not available, waiting again...",
      robot_description_node_.c_str());
  }

  std::string robot_description_param = "robot_description";
  RCLCPP_INFO(
    get_logger(), "connected to service (%s), asking for '%s'...",
    robot_description_node_.c_str(),
    robot_description_param.c_str());

  // search and wait for robot_description on param server
  while (urdf_string.empty()) {
    RCLCPP_DEBUG(
      get_logger(), "param_name = '%s'",
      robot_description_param.c_str());

    // get parameters
    try {
      auto results = parameters_client->get_parameters({robot_description_param});
      RCLCPP_INFO(
        get_logger(), "Waiting for and answer from param server...");
      if (results.wait_for(5s) != std::future_status::ready) {
        RCLCPP_INFO(get_logger(), "No answer from param server, trying again...");
        continue;
      }
      std::vector<rclcpp::Parameter> values = results.get();
      urdf_string = values[0].as_string();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "%s", e.what());
    }
    if (!urdf_string.empty()) {
      break;
    } else {
      RCLCPP_ERROR(
        get_logger(),
        "Waiting for model URDF in parameter [%s] on the ROS param server.",
        robot_description_param.c_str());
    }
    // std::this_thread::sleep_for(std::chrono::microseconds(100000));
  }
  RCLCPP_INFO(get_logger(), "Received URDF from param server");

  executor.cancel();
  node_thread->join();
  node_thread.reset();
  async_node.reset();

  return urdf_string;
}


}  // namespace cartesian_vic_servo

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cartesian_vic_servo::CartesianVicServo>();
  node->init();
  node->start();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
