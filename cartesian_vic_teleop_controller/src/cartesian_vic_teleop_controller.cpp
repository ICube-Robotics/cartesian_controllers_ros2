// Copyright 2023 ICUBE Laboratory, University of Strasbourg
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

#include "cartesian_vic_teleop_controller/cartesian_vic_teleop_controller.hpp"

#include "rclcpp/logging.hpp"
#include "rcutils/logging_macros.h"

namespace cartesian_vic_teleop_controller
{

PassiveVicTeleopController::PassiveVicTeleopController()
: Base::CartesianVicController()
{
    // TODO(tpoignonec) : check if this is needed
    // Actually, same for most overridden methods...
}

controller_interface::CallbackReturn
PassiveVicTeleopController::on_init()
{
  auto ret = Base::on_init();
  return ret;
}

controller_interface::CallbackReturn
PassiveVicTeleopController::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  auto ret = Base::on_configure(previous_state);
  if (ret != controller_interface::CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Parent Base::on_configure() failed!");
    return ret;
  }
  // Unsubscribe the existing subscriber (declare by base and only used in update)
  input_compliant_frame_trajectory_subscriber_.reset();

  // setup new subscribers and publishers used for VIC-based teleoperation
  auto follower_vic_state_callback =
    [this](const std::shared_ptr<cartesian_control_msgs::msg::VicControllerState> msg)
    {input_follower_vic_state_msg_.writeFromNonRT(msg);};
  follower_vic_state_subscriber_ =
    get_node()->create_subscription<cartesian_control_msgs::msg::VicControllerState>(
      "/vic_controller_follower/status",
      rclcpp::SystemDefaultsQoS(),
      follower_vic_state_callback);

  auto teleop_compliance_callback =
    [this](const std::shared_ptr<cartesian_control_msgs::msg::TeleopCompliance> msg)
    {input_teleop_compliance_msg_.writeFromNonRT(msg);};
  teleop_compliance_subscriber_ =
    get_node()->create_subscription<cartesian_control_msgs::msg::TeleopCompliance>(
      "~/teleoperation_compliance_reference",
      rclcpp::SystemDefaultsQoS(),
      teleop_compliance_callback);

  auto is_clutched_callback =
    [this](const std::shared_ptr<std_msgs::msg::Bool> msg)
    {input_is_clutched_msg_.writeFromNonRT(msg);};
  is_clutched_subscriber_ =
    get_node()->create_subscription<std_msgs::msg::Bool>(
      "fd_clutch",
      rclcpp::SystemDefaultsQoS(),
      is_clutched_callback);

  // Follower VIC ref publisher
  non_rt_follower_vic_ref_publisher_ = \
    get_node()->create_publisher<cartesian_control_msgs::msg::CompliantFrameTrajectory>(
      "/vic_controller_follower/reference_compliant_frame_trajectory",
      rclcpp::SystemDefaultsQoS());
  follower_vic_ref_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<
        cartesian_control_msgs::msg::CompliantFrameTrajectory>>(non_rt_follower_vic_ref_publisher_);

  // Teleop state publisher
  non_rt_teleop_state_publisher_ = \
    get_node()->create_publisher<cartesian_control_msgs::msg::TeleopControllerState>(
      "~/teleop_controller_state",
      rclcpp::SystemDefaultsQoS());
  teleop_state_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<
        cartesian_control_msgs::msg::TeleopControllerState>>(non_rt_teleop_state_publisher_);

  return ret;
}

controller_interface::return_type
PassiveVicTeleopController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void) time;

  auto execute_fallback_policy = [this] ()
    {
      RCLCPP_ERROR(
      get_node()->get_logger(),
      "Fallback policy triggered! Setting velocity / force controls to zero!");
      joint_command_ = last_commanded_joint_state_;
      std::fill(
      joint_command_.velocities.begin(), joint_command_.velocities.end(), 0.0);
      std::fill(
      joint_command_.accelerations.begin(), joint_command_.accelerations.end(), 0.0);
      std::fill(
      joint_command_.effort.begin(), joint_command_.effort.end(), 0.0);
      write_state_to_hardware(joint_command_);
    };

  // Realtime constraints are required in this function
  if (!vic_) {
    execute_fallback_policy();
    return controller_interface::return_type::ERROR;
  }

  // get all controller inputs
  bool is_state_valid = read_state_from_hardware(measurement_data_);

  // update compliant frame(s) reference from ros subscriber message
  follower_vic_state_msg_ptr_ = \
    *input_follower_vic_state_msg_.readFromRT();
  if (!follower_vic_state_msg_ptr_.get()) {
    auto clock = get_node()->get_clock();
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *clock,
      5000,
      "Waiting for follower VicStateMsg...");
    // Exit and wait for valid data...
    is_state_valid = false;
    // execute_fallback_policy();
    return controller_interface::return_type::OK;
  }

  // update compliance reference from ros subscriber message
  teleop_compliance_msg_ptr_ = \
    *input_teleop_compliance_msg_.readFromRT();
  if (!teleop_compliance_msg_ptr_.get()) {
    auto clock = get_node()->get_clock();
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *clock,
      1000,
      "WARNING: No compliance reference msg received...");
    is_state_valid = false;
    // execute_fallback_policy();
    return controller_interface::return_type::OK;
  }

  // retrieve clutch state
  bool workspace_clutch_disengaged = true;
  is_clutched_msg_ptr_ = \
    *input_is_clutched_msg_.readFromRT();
  if (is_clutched_msg_ptr_.get()) {
    // Only engage workspace if clutched
    workspace_clutch_disengaged = !is_clutched_msg_ptr_->data;
  } else {
    auto clock = get_node()->get_clock();
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *clock,
      1000,
      "Could not retrieve clutched state from topic, assuming false in the mean time...");
  }
  // TODO(tpoignonec): add subscriber to get clutch state

  // make sure impedance was initialized
  bool all_ok = true;
  if (!is_vic_initialized_ && !is_state_valid) {
    // Exit and wait for valid data...
    return controller_interface::return_type::OK;
  } else if (!is_vic_initialized_ && is_state_valid) {
    // Init current desired pose from current joint position
    if (!initialize_vic_rule(measurement_data_)) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Failed to initialize the VIC rule!");
      return controller_interface::return_type::ERROR;
    }
  } else if (!is_state_valid) {
    // TODO(tpoignonec): return ERROR?
    all_ok &= false;
    joint_command_ = last_commanded_joint_state_;
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Invalid data from hardware interface!");
    execute_fallback_policy();
    return controller_interface::return_type::ERROR;
  }

  // Control logic
  if (all_ok) {
    // Check follower VicStateMsg availability
    // TODO(tpoignonec) : check timestamps also!
    if (!follower_vic_state_msg_ptr_.get()) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Failed to retrieve follower VicStateMsg!");
      execute_fallback_policy();
      return controller_interface::return_type::ERROR;
    } else {
      follower_vic_state_msg_ = *follower_vic_state_msg_ptr_.get();
    }

    // Update vic input data
    RCLCPP_DEBUG(get_node()->get_logger(), "vic_->update_input_data()");
    auto ret_vic = vic_->update_input_data(period, measurement_data_);
    if (ret_vic != controller_interface::return_type::OK) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Failed to update VIC rule input data!");
      execute_fallback_policy();
      return controller_interface::return_type::ERROR;
    }

    // Update teleop logic
    if (!teleop_logic_.is_initialized()) {
      std::string teleop_parameters_namespace = "teleoperation";
      // Init teleop logic if not yet OK
      RCLCPP_INFO(
        get_node()->get_logger(),
        "Initializing the teleop logic...");
      try {
        if (!teleop_logic_.init(
          time,
          get_node()->get_node_parameters_interface(),
          vic_->get_input_data(),
          follower_vic_state_msg_,
          teleop_parameters_namespace))
        {
          RCLCPP_ERROR(
            get_node()->get_logger(),
            "Failed to initialize the teleop logic!");
          execute_fallback_policy();
          return controller_interface::return_type::ERROR;
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR(
          get_node()->get_logger(), "Exception thrown during update stage with message: %s \n",
          e.what());
        execute_fallback_policy();
        return controller_interface::return_type::ERROR;
      }
    }

    RCLCPP_DEBUG(get_node()->get_logger(), "teleop_logic_.setTeleoperationCompliance()");
    if (!teleop_logic_.setTeleoperationCompliance(
      *teleop_compliance_msg_ptr_.get()))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Failed to set the teleop compliance!");
      execute_fallback_policy();
      return controller_interface::return_type::ERROR;
    }

    RCLCPP_DEBUG(get_node()->get_logger(), "teleop_logic_.update()");
    if (!teleop_logic_.update(
      time, period, workspace_clutch_disengaged, vic_->get_input_data(), follower_vic_state_msg_))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Failed to update the teleop logic!");
      execute_fallback_policy();
      return controller_interface::return_type::ERROR;
    }

    RCLCPP_DEBUG(get_node()->get_logger(), "teleop_logic_.get_*_vic_ref()");
    all_ok &= teleop_logic_.get_leader_vic_ref(leader_vic_ref_);
    all_ok &= teleop_logic_.get_follower_vic_ref(follower_vic_ref_);

    if (all_ok) {
      RCLCPP_DEBUG(get_node()->get_logger(), "vic_->update_compliant_frame_trajectory()");
      // Update leader ref
      vic_->update_compliant_frame_trajectory(leader_vic_ref_);
      // Publish follower ref
      follower_vic_ref_publisher_->lock();
      follower_vic_ref_publisher_->msg_ = follower_vic_ref_;
      follower_vic_ref_publisher_->unlockAndPublish();
    } else {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Failed to retrieve compliant frame trajectories!");
      execute_fallback_policy();
      return controller_interface::return_type::ERROR;
    }

    // Get leader VIC commands
    RCLCPP_DEBUG(get_node()->get_logger(), "vic_->compute_controls()");
    ret_vic = vic_->compute_controls(period, joint_command_);
    if (ret_vic != controller_interface::return_type::OK) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Failed to compute VIC rule commands!");
      execute_fallback_policy();
      return controller_interface::return_type::ERROR;
    }

    if (ret_vic != controller_interface::return_type::OK) {
      return controller_interface::return_type::ERROR;
    }
  }

  // write calculated values to joint interfaces
  write_state_to_hardware(joint_command_);


  // Publish teleop controller state
  if (!teleop_logic_.get_current_state_msg(teleop_state_msg_)) {
    // NOT a critical error, but we want to know if it happens
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to retrieve teleop rule state as a ros2 msg!");
  } else {
    teleop_state_msg_.header.stamp = get_node()->get_clock()->now();
    teleop_state_publisher_->lock();
    teleop_state_publisher_->msg_ = teleop_state_msg_;
    teleop_state_publisher_->unlockAndPublish();
  }

  // Publish controller state
  if (vic_->controller_state_to_msg(controller_state_msg_) != \
    controller_interface::return_type::OK)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to retrieve VIC rule state!");
    execute_fallback_policy();
    return controller_interface::return_type::ERROR;
  } else {
    controller_state_msg_.header.stamp = get_node()->get_clock()->now();
    state_publisher_->lock();
    state_publisher_->msg_ = controller_state_msg_;
    state_publisher_->unlockAndPublish();
  }
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn
PassiveVicTeleopController::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  auto ret = Base::on_activate(previous_state);
  return ret;
}


controller_interface::CallbackReturn
PassiveVicTeleopController::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  auto ret = Base::on_deactivate(previous_state);
  return ret;
}

controller_interface::CallbackReturn
PassiveVicTeleopController::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
  auto ret = Base::on_cleanup(previous_state);
  return ret;
}


controller_interface::CallbackReturn
PassiveVicTeleopController::on_error(const rclcpp_lifecycle::State & previous_state)
{
  auto ret = Base::on_error(previous_state);
  return ret;
}

}  // namespace cartesian_vic_teleop_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  cartesian_vic_teleop_controller::PassiveVicTeleopController,
  controller_interface::ControllerInterface
)
