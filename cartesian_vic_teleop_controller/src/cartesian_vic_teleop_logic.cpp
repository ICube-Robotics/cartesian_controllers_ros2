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

#include "cartesian_vic_teleop_controller/cartesian_vic_teleop_logic.hpp"

#include "rclcpp/logging.hpp"
#include "rcutils/logging_macros.h"

#include "cartesian_vic_controller/utils.hpp"
#include "cartesian_vic_controller/vic_msgs_utils.hpp"

namespace cartesian_vic_teleop_controller
{

using cartesian_vic_controller::VicInputData;
using cartesian_vic_controller::matrixEigenToMsg;
using cartesian_vic_controller::AccelToMsg;
using cartesian_vic_controller::WrenchToMsg;


PassiveVicTeleopLogic::PassiveVicTeleopLogic()
: is_initialized_(false),
  has_update_been_called_(false),
  logger_(rclcpp::get_logger("cartesian_vic_teleop_logic"))
{
  // anything?
}

// Internal logic
// ---------------------

bool PassiveVicTeleopLogic::internal_init(
  const rclcpp::Time & time,
  std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface,
  const std::string & param_namespace)
{
  parameters_interface_ = parameters_interface;
  is_initialized_ = false;
  has_update_been_called_ = false;

  RCLCPP_INFO(logger_, "init(): initializing the mapping manager...");
  Eigen::Matrix3d master_base_to_follower_base_axis_mapping = \
    Eigen::Matrix3d::Identity();
  // Axis mapping is chosen so as to align the follower motion with the leader motion
  master_base_to_follower_base_axis_mapping = \
    Eigen::AngleAxis<double>(static_cast<double>(EIGEN_PI), Eigen::Vector3d::UnitZ());

  // Init the mapping manager
  bool all_ok = mapping_manager_.init(master_base_to_follower_base_axis_mapping);
  all_ok &= mapping_manager_.reset_mapping(
    follower_pose_in_follower_base_frame_,
    leader_pose_in_leader_base_frame_);
  all_ok &= mapping_manager_.update(leader_pose_in_leader_base_frame_);

  if (!all_ok) {
    RCLCPP_ERROR(logger_, "init(): Failed to initialize the mapping manager!");
    return false;
  }

  // Init the VIC references
  RCLCPP_INFO(logger_, "init(): initializing the data structures...");
  leader_vic_ref_ = CompliantFrameTrajectoryMsg();
  leader_vic_ref_.header.stamp = time;
  auto leader_cartesian_traj_point = cartesian_control_msgs::msg::CartesianTrajectoryPoint();
  auto leader_compliance = cartesian_control_msgs::msg::CartesianCompliance();
  leader_cartesian_traj_point.time_from_start = rclcpp::Duration::from_seconds(0.0);
  leader_cartesian_traj_point.pose = Eigen::toMsg(leader_pose_in_leader_base_frame_);
  leader_vic_ref_.cartesian_trajectory_points.push_back(leader_cartesian_traj_point);
  leader_vic_ref_.compliance_at_points.push_back(leader_compliance);

  follower_vic_ref_ = CompliantFrameTrajectoryMsg();
  follower_vic_ref_.header.stamp = time;
  auto follower_cartesian_traj_point = cartesian_control_msgs::msg::CartesianTrajectoryPoint();
  auto follower_compliance = cartesian_control_msgs::msg::CartesianCompliance();
  follower_cartesian_traj_point.time_from_start = rclcpp::Duration::from_seconds(0.0);
  follower_cartesian_traj_point.pose = Eigen::toMsg(follower_pose_in_follower_base_frame_);
  follower_vic_ref_.cartesian_trajectory_points.push_back(follower_cartesian_traj_point);
  follower_vic_ref_.compliance_at_points.push_back(follower_compliance);

  // Reset the teleoperation input data
  if(!mapping_manager_.map_master_tool_to_follower_tool(
    leader_pose_in_leader_base_frame_, teleop_data_input_.leader_pose))
  {
    RCLCPP_ERROR(logger_, "init(): Failed to map the leader pose!");
    all_ok = false;
  }
  teleop_data_input_.leader_velocity.setZero();
  teleop_data_input_.leader_acceleration.setZero();
  teleop_data_input_.leader_wrench.setZero();
  teleop_data_input_.leader_desired_inertia = leader_natural_inertia_in_leader_base_frame_;
  teleop_data_input_.leader_desired_stiffness = 50. * Eigen::Matrix<double, 6, 6>::Identity();
  teleop_data_input_.leader_desired_damping = 10. * Eigen::Matrix<double, 6, 6>::Identity();

  teleop_data_input_.follower_pose = follower_pose_in_follower_base_frame_;
  teleop_data_input_.follower_velocity.setZero();
  teleop_data_input_.follower_acceleration.setZero();
  teleop_data_input_.follower_wrench.setZero();
  teleop_data_input_.follower_desired_inertia = follower_natural_inertia_in_follower_base_frame_;
  teleop_data_input_.follower_desired_stiffness = 50. * Eigen::Matrix<double, 6, 6>::Identity();
  teleop_data_input_.follower_desired_damping = 10 * Eigen::Matrix<double, 6, 6>::Identity();

  // Reset the teleoperation output data
  set_default_safe_behavior(teleop_data_input_, teleop_data_output_);

  // Reset cache
  leader_velocity_in_leader_base_frame_.setZero();
  leader_acceleration_in_leader_base_frame_.setZero();
  leader_wrench_in_leader_base_frame_.setZero();

  follower_velocity_in_follower_base_frame_.setZero();
  follower_acceleration_in_follower_base_frame_.setZero();
  follower_wrench_in_follower_base_frame_.setZero();

  leader_reference_pose_in_leader_base_frame_ = leader_pose_in_leader_base_frame_;
  leader_reference_velocity_in_leader_base_frame_.setZero();
  leader_reference_acceleration_in_leader_base_frame_.setZero();
  leader_reference_wrench_in_leader_base_frame_.setZero();
  leader_reference_inertia_in_leader_base_frame_ = teleop_data_output_.leader_desired_inertia;
  leader_reference_stiffness_in_leader_base_frame_ = teleop_data_output_.leader_desired_stiffness;
  leader_reference_damping_in_leader_base_frame_ = teleop_data_output_.leader_desired_damping;

  // Initialize the teleoperation rule plugin
  RCLCPP_INFO(logger_, "init(): initializing the rule plugin...");

  auto rule_plugin_name_param_name = param_namespace + ".rule_plugin_name";
  auto rule_plugin_package_param_name = param_namespace + ".rule_plugin_package";

  rcl_interfaces::msg::ParameterDescriptor rule_plugin_param_desc;
  rule_plugin_param_desc.name = "rule_plugin_name";
  rule_plugin_param_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  rule_plugin_param_desc.description = "Name of the teleoperation rule plugin";

  rcl_interfaces::msg::ParameterDescriptor rule_plugin_package_param_desc;
  rule_plugin_package_param_desc.name = "rule_plugin_package";
  rule_plugin_package_param_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  rule_plugin_package_param_desc.description = "Package of the teleoperation rule plugin";

  parameters_interface->declare_parameter(
    rule_plugin_name_param_name, rclcpp::ParameterValue(""), rule_plugin_param_desc);
  parameters_interface->declare_parameter(
    rule_plugin_package_param_name, rclcpp::ParameterValue(""), rule_plugin_package_param_desc);

  auto rule_plugin_name_param = rclcpp::Parameter();
  auto rule_plugin_package_param = rclcpp::Parameter();
  if (!parameters_interface->get_parameter(
    rule_plugin_name_param_name, rule_plugin_name_param))
  {
    RCLCPP_ERROR(logger_, "parameter %s not set", rule_plugin_name_param_name.c_str());
    return false;
  }
  if (!parameters_interface->get_parameter(
    rule_plugin_package_param_name, rule_plugin_package_param))
  {
    RCLCPP_ERROR(logger_, "parameter %s not set", rule_plugin_package_param_name.c_str());
    return false;
  }

  std::string rule_plugin_name = rule_plugin_name_param.as_string();
  std::string rule_plugin_package = rule_plugin_package_param.as_string();

  RCLCPP_INFO(
    logger_, "init(): loading rule plugin '%s' from pkg '%s'...",
    rule_plugin_name.c_str(), rule_plugin_package.c_str());

  try {
    if (!rule_plugin_name.empty() && !rule_plugin_package.empty()) {
      teleop_rule_loader_ =
        std::make_shared<pluginlib::ClassLoader<
            cartesian_vic_teleop_controller::TeleopRule>>(
        rule_plugin_package,
        "cartesian_vic_teleop_controller::TeleopRule"
            );
      teleop_rule_ = std::unique_ptr<cartesian_vic_teleop_controller::TeleopRule>(
        teleop_rule_loader_->createUnmanagedInstance(rule_plugin_name));
    } else {
      RCLCPP_FATAL(
        logger_,
        "Please provide 'rule_plugin_package' and 'rule_plugin_name' parameters!");
      return false;
    }
    // Initialize teleop rule plugin
    is_teleop_rule_initialized_ = teleop_rule_->init(parameters_interface_, param_namespace);
    if (!is_teleop_rule_initialized_) {
      RCLCPP_ERROR(logger_, "Failed to initialize the teleop rule plugin!");
      all_ok = false;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Exception thrown during initialization with message: %s \n",
      e.what());
    return false;
  }

  // Set the initialized flag
  is_initialized_ = all_ok;
  RCLCPP_INFO(logger_, "init(): all done.");
  return is_initialized_;
}

bool PassiveVicTeleopLogic::internal_update(
  const rclcpp::Time & time,
  const rclcpp::Duration & period,
  bool workspace_clutch_disengaged)
{
  if (!teleop_rule_) {
    RCLCPP_ERROR(logger_, "update(): the teleop rule plugin is not loaded!");
    return false;
  }

  if (!is_teleop_rule_initialized_) {
    RCLCPP_ERROR(logger_, "update(): the teleop rule plugin is not initialized!");
    return false;
  }

  if (!mapping_manager_.is_initialized()) {
    RCLCPP_ERROR(logger_, "update(): Mapping manager is not initialized!");
    return false;
  }

  if (!mapping_manager_.update(
    leader_pose_in_leader_base_frame_,
    workspace_clutch_disengaged))
  {
    RCLCPP_ERROR(logger_, "update(): Failed to update the mapping manager!");
    return false;
  }

  // Copy the follower data
  teleop_data_input_.follower_pose = follower_pose_in_follower_base_frame_;
  teleop_data_input_.follower_velocity = follower_velocity_in_follower_base_frame_;
  teleop_data_input_.follower_acceleration = follower_acceleration_in_follower_base_frame_;
  teleop_data_input_.follower_wrench = follower_wrench_in_follower_base_frame_;
  teleop_data_input_.follower_natural_inertia = follower_natural_inertia_in_follower_base_frame_;

  // Map the leader data into the follower frame
  bool all_ok = mapping_manager_.map_master_tool_to_follower_tool(
    leader_pose_in_leader_base_frame_, teleop_data_input_.leader_pose);
  all_ok &= mapping_manager_.map_twist_or_wrench_from_master_to_follower(
    leader_velocity_in_leader_base_frame_, teleop_data_input_.leader_velocity);
  all_ok &= mapping_manager_.map_twist_or_wrench_from_master_to_follower(
    leader_acceleration_in_leader_base_frame_, teleop_data_input_.leader_acceleration);
  all_ok &= mapping_manager_.map_twist_or_wrench_from_master_to_follower(
    leader_wrench_in_leader_base_frame_, teleop_data_input_.leader_wrench);
  all_ok &= mapping_manager_.map_SDPD_from_master_to_follower(
    leader_natural_inertia_in_leader_base_frame_, teleop_data_input_.leader_natural_inertia);

  if (!all_ok) {
    RCLCPP_ERROR(logger_, "update(): Failed to map leader data into follower frame!");
    return false;
  }

  // Set the workspace clutch flag
  teleop_data_input_.workspace_is_engaged = (
    mapping_manager_.get_clutching_state() == MappingManager::CLUTCHING_STATE::ENGAGED);

  // Compute the reference trajectory using teleop rule plugin
  all_ok &= teleop_rule_->update(time, period, teleop_data_input_, teleop_data_output_);
  if (!all_ok) {
    RCLCPP_ERROR(logger_, "update(): Failed to compute reference VIC profiles!");
    return false;
  }

  // Disengage the follower motion forcefully if the workspace is disengaged
  if (!teleop_data_input_.workspace_is_engaged) {
    // Disengage the follower when needed
    teleop_data_output_.follower_desired_velocity.setZero();
    teleop_data_output_.follower_desired_acceleration.setZero();
    teleop_data_output_.follower_desired_wrench.setZero();
    // TODO(anyone): Is this right? Doesn't seem so...
  }

  // Map the leader reference trajectory into the leader frame
  all_ok &= mapping_manager_.inverse_map_follower_tool_to_master_tool(
    teleop_data_output_.leader_desired_pose, leader_reference_pose_in_leader_base_frame_);
  all_ok &= mapping_manager_.map_twist_or_wrench_from_follower_to_master(
    teleop_data_output_.leader_desired_velocity, leader_reference_velocity_in_leader_base_frame_);
  all_ok &= mapping_manager_.map_twist_or_wrench_from_follower_to_master(
    teleop_data_output_.leader_desired_acceleration,
    leader_reference_acceleration_in_leader_base_frame_);
  all_ok &= mapping_manager_.map_twist_or_wrench_from_follower_to_master(
    teleop_data_output_.leader_desired_wrench,
    leader_reference_wrench_in_leader_base_frame_);
  all_ok &= mapping_manager_.map_SDPD_from_follower_to_master(
    teleop_data_output_.leader_desired_inertia,
    leader_reference_inertia_in_leader_base_frame_);
  all_ok &= mapping_manager_.map_SDPD_from_follower_to_master(
    teleop_data_output_.leader_desired_stiffness,
    leader_reference_stiffness_in_leader_base_frame_);
  all_ok &= mapping_manager_.map_SDPD_from_follower_to_master(
    teleop_data_output_.leader_desired_damping,
    leader_reference_damping_in_leader_base_frame_);

  if (!all_ok) {
    RCLCPP_ERROR(logger_, "update(): Failed to map leader reference trajectory into leader frame!");
    return false;
  }

  // Fill the leader VIC ref trajectory msg
  leader_vic_ref_.header.stamp = time;
  auto & leader_cartesian_traj_point = leader_vic_ref_.cartesian_trajectory_points[0];
  auto & leader_compliance = leader_vic_ref_.compliance_at_points[0];

  leader_cartesian_traj_point.time_from_start = rclcpp::Duration::from_seconds(0.0);
  leader_cartesian_traj_point.pose = Eigen::toMsg(leader_reference_pose_in_leader_base_frame_);
  leader_cartesian_traj_point.velocity = \
    Eigen::toMsg(leader_reference_velocity_in_leader_base_frame_);
  leader_cartesian_traj_point.acceleration = \
    AccelToMsg(leader_reference_acceleration_in_leader_base_frame_);
  leader_cartesian_traj_point.wrench = \
    WrenchToMsg(leader_reference_wrench_in_leader_base_frame_);
  matrixEigenToMsg(
    leader_reference_inertia_in_leader_base_frame_, leader_compliance.inertia);
  matrixEigenToMsg(
    leader_reference_stiffness_in_leader_base_frame_, leader_compliance.stiffness);
  matrixEigenToMsg(
    leader_reference_damping_in_leader_base_frame_, leader_compliance.damping);

  // Fill the follower VIC ref trajectory msg
  follower_vic_ref_.header.stamp = time;
  auto & follower_cartesian_traj_point = follower_vic_ref_.cartesian_trajectory_points[0];
  auto & follower_compliance = follower_vic_ref_.compliance_at_points[0];
  follower_cartesian_traj_point.time_from_start = rclcpp::Duration::from_seconds(0.0);
  follower_cartesian_traj_point.pose = Eigen::toMsg(teleop_data_output_.follower_desired_pose);
  follower_cartesian_traj_point.velocity = \
    Eigen::toMsg(teleop_data_output_.follower_desired_velocity);
  follower_cartesian_traj_point.acceleration = \
    AccelToMsg(teleop_data_output_.follower_desired_acceleration);
  follower_cartesian_traj_point.wrench = \
    WrenchToMsg(teleop_data_output_.follower_desired_wrench);
  matrixEigenToMsg(
    teleop_data_output_.follower_desired_inertia, follower_compliance.inertia);
  matrixEigenToMsg(
    teleop_data_output_.follower_desired_stiffness, follower_compliance.stiffness);
  matrixEigenToMsg(
    teleop_data_output_.follower_desired_damping, follower_compliance.damping);

  has_update_been_called_ = all_ok;
  return all_ok;
}

// Convenience functions
// ---------------------
bool PassiveVicTeleopLogic::init(
  const rclcpp::Time & time,
  std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface,
  const VicStateMsg & leader_vic_state_msg,
  const VicStateMsg & follower_vic_state_msg,
  const std::string & param_namespace)
{
  RCLCPP_DEBUG(logger_, "Initializing the teleop logic...");

  bool success = extract_input_data(leader_vic_state_msg, follower_vic_state_msg);
  if (!success) {
    RCLCPP_ERROR(logger_, "init(): Failed to extract data from VicStateMsg!");
    return false;
  }
  RCLCPP_DEBUG(logger_, "Call to internal_init()...");
  return internal_init(time, parameters_interface, param_namespace);
}

bool PassiveVicTeleopLogic::init(
  const rclcpp::Time & time,
  std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface,
  const VicInputData & leader_input_data,
  const VicStateMsg & follower_vic_state_msg,
  const std::string & param_namespace)
{
  RCLCPP_DEBUG(logger_, "Initializing the teleop logic...");

  bool success = extract_input_data(leader_input_data, follower_vic_state_msg);
  if (!success) {
    RCLCPP_ERROR(
            logger_, "init(): Failed to extract data from VicInputData and VicStateMsg!");
    return false;
  }
  RCLCPP_DEBUG(logger_, "Call to internal_init()...");
  return internal_init(time, parameters_interface, param_namespace);
}

bool PassiveVicTeleopLogic::update(
  const rclcpp::Time & time,
  const rclcpp::Duration & period,
  bool workspace_clutch_disengaged,
  const VicStateMsg & leader_vic_state_msg,
  const VicStateMsg & follower_vic_state_msg)
{
  RCLCPP_DEBUG(logger_, "Update()");
  bool success = extract_input_data(leader_vic_state_msg, follower_vic_state_msg);
  if (!success) {
    RCLCPP_ERROR(logger_, "update(): Failed to extract data from VicStateMsg!");
    return false;
  }

  return internal_update(time, period, workspace_clutch_disengaged);
}

bool PassiveVicTeleopLogic::update(
  const rclcpp::Time & time,
  const rclcpp::Duration & period,
  bool workspace_clutch_disengaged,
  const VicInputData & leader_input_data,
  const VicStateMsg & follower_vic_state_msg)
{
  RCLCPP_DEBUG(logger_, "Update()");
  bool success = extract_input_data(leader_input_data, follower_vic_state_msg);
  if (!success) {
    RCLCPP_ERROR(
            logger_, "init(): Failed to extract data from VicInputData and VicStateMsg!");
    return false;
  }
  return internal_update(time, period, workspace_clutch_disengaged);
}

bool PassiveVicTeleopLogic::setTeleoperationCompliance(
  const cartesian_control_msgs::msg::TeleopCompliance & msg)
{
  bool all_ok = true;
  all_ok &= cartesian_vic_controller::fromMsg(
    msg.leader_desired_inertia, teleop_data_input_.leader_desired_inertia);
  all_ok &= cartesian_vic_controller::fromMsg(
    msg.leader_desired_stiffness, teleop_data_input_.leader_desired_stiffness);
  all_ok &= cartesian_vic_controller::fromMsg(
    msg.leader_desired_damping, teleop_data_input_.leader_desired_damping);

  all_ok &= cartesian_vic_controller::fromMsg(
    msg.follower_desired_inertia, teleop_data_input_.follower_desired_inertia);
  all_ok &= cartesian_vic_controller::fromMsg(
    msg.follower_desired_stiffness, teleop_data_input_.follower_desired_stiffness);
  all_ok &= cartesian_vic_controller::fromMsg(
    msg.follower_desired_damping, teleop_data_input_.follower_desired_damping);

  return all_ok;
}


bool
PassiveVicTeleopLogic::get_current_state_msg(
  cartesian_control_msgs::msg::TeleopControllerState & msg)
{
  if (!is_initialized_ || !has_update_been_called_) {
    RCLCPP_ERROR(logger_, "get_current_state_msg(): not yet initialized and/or updated!");
    return false;
  }
  return teleop_rule_->get_current_state_msg(teleop_data_input_, teleop_data_output_, msg);
}

// Utils
// ---------------------

bool PassiveVicTeleopLogic::extract_input_data(
  const VicStateMsg & leader_vic_state_msg,
  const VicStateMsg & follower_vic_state_msg)
{
  if (!extract_follower_input_data(follower_vic_state_msg)) {
    RCLCPP_ERROR(logger_, "extract_input_data(): failed to extract follower data!");
    return false;
  }
  bool all_ok = true;
  tf2::fromMsg(leader_vic_state_msg.pose, leader_pose_in_leader_base_frame_);
  all_ok &= cartesian_vic_controller::get_robot_velocity(
        leader_vic_state_msg, leader_velocity_in_leader_base_frame_);
  all_ok &= cartesian_vic_controller::get_robot_acceleration(
        leader_vic_state_msg, leader_acceleration_in_leader_base_frame_);

  if (follower_vic_state_msg.has_valid_wrench) {
    all_ok &= cartesian_vic_controller::get_robot_wrench(
          leader_vic_state_msg, leader_wrench_in_leader_base_frame_);
  } else {
    leader_wrench_in_leader_base_frame_.setZero();
    all_ok = false;
    RCLCPP_WARN_THROTTLE(
            logger_,
            internal_clock_,
            5000,
            "No leader force / torque sensor available!"
    );
  }
  all_ok &= cartesian_vic_controller::get_natural_robot_inertia(
        leader_vic_state_msg, leader_natural_inertia_in_leader_base_frame_);
  return all_ok;
}

bool PassiveVicTeleopLogic::extract_input_data(
  const cartesian_vic_controller::VicInputData & leader_input_data,
  const VicStateMsg & follower_vic_state_msg)
{
  if (!extract_follower_input_data(follower_vic_state_msg)) {
    RCLCPP_ERROR(logger_, "extract_input_data(): failed to extract follower data!");
    return false;
  }
  bool all_ok = true;
  leader_pose_in_leader_base_frame_ = leader_input_data.robot_current_pose;
  leader_velocity_in_leader_base_frame_ = leader_input_data.robot_current_velocity;
  leader_acceleration_in_leader_base_frame_ = leader_input_data.robot_estimated_acceleration;
  if (leader_input_data.has_external_torque_sensor()) {
    leader_wrench_in_leader_base_frame_ = leader_input_data.get_ft_sensor_wrench();
  } else {
    leader_wrench_in_leader_base_frame_.setZero();
    RCLCPP_WARN_THROTTLE(
            logger_,
            internal_clock_,
            5000,
            "No leader force / torque sensor available!"
    );
  }
  leader_natural_inertia_in_leader_base_frame_ = leader_input_data.natural_cartesian_inertia;
  return all_ok;
}

bool PassiveVicTeleopLogic::extract_follower_input_data(
  const VicStateMsg & follower_vic_state_msg)
{
  bool all_ok = true;
  tf2::fromMsg(follower_vic_state_msg.pose, follower_pose_in_follower_base_frame_);
  all_ok &= cartesian_vic_controller::get_robot_velocity(
        follower_vic_state_msg, follower_velocity_in_follower_base_frame_);
  all_ok &= cartesian_vic_controller::get_robot_acceleration(
        follower_vic_state_msg, follower_acceleration_in_follower_base_frame_);
  if (follower_vic_state_msg.has_valid_wrench) {
    all_ok &= cartesian_vic_controller::get_robot_wrench(
          follower_vic_state_msg, follower_wrench_in_follower_base_frame_);
  } else {
    follower_wrench_in_follower_base_frame_.setZero();
    all_ok = false;
    RCLCPP_WARN_THROTTLE(
            logger_,
            internal_clock_,
            5000,
            "No follower force / torque sensor available!"
    );
  }
  if (!cartesian_vic_controller::get_natural_robot_inertia(
        follower_vic_state_msg, follower_natural_inertia_in_follower_base_frame_))
  {
    RCLCPP_ERROR(logger_, "Failed to extract follower inertia!");
    all_ok = false;
  }
  return all_ok;
}

bool PassiveVicTeleopLogic::get_leader_vic_ref(
  CompliantFrameTrajectoryMsg & leader_vic_ref_msg)
{
  if(!is_initialized_ || !has_update_been_called_) {
    RCLCPP_ERROR(logger_, "get_leader_vic_ref(): not yet initialized and/or updated!");
    return false;
  }
  leader_vic_ref_msg = leader_vic_ref_;
  return true;
}

bool PassiveVicTeleopLogic::get_follower_vic_ref(
  CompliantFrameTrajectoryMsg & follower_vic_ref_msg)
{
  if(!is_initialized_ || !has_update_been_called_) {
    RCLCPP_ERROR(logger_, "get_follower_vic_ref(): not yet initialized and/or updated!");
    return false;
  }
  follower_vic_ref_msg = follower_vic_ref_;
  return true;
}
}  // namespace cartesian_vic_teleop_controller
