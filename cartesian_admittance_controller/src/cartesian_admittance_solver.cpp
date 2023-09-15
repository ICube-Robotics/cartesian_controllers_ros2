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
//
/// \authors: Thibault Poignonec, Maciej Bednarczyk

#include "cartesian_admittance_controller/cartesian_admittance_solver.hpp"

#include "rclcpp/duration.hpp"
//#include "rclcpp/utilities.hpp"
#include "tf2_ros/transform_listener.h"

namespace cartesian_admittance_controller
{
CartesianAdmittanceSolver::CartesianAdmittanceSolver(
    const std::shared_ptr<cartesian_admittance_controller::ParamListener> & parameter_handler)
{
    parameter_handler_ = parameter_handler;
    parameters_ = parameter_handler_->get_params();
    num_joints_ = parameters_.joints.size();
    admittance_state_ = AdmittanceState(num_joints_);
    use_streamed_interaction_parameters_ = false;
    reset(num_joints_);
}

controller_interface::return_type
CartesianAdmittanceSolver::configure(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node,
    const size_t num_joints)
{
    num_joints_ = num_joints;
    // reset admittance state
    reset(num_joints);
    // Load the differential IK plugin
    if (!parameters_.kinematics.plugin_name.empty())
    {
    try
    {
        kinematics_loader_ =
            std::make_shared<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>(
                parameters_.kinematics.plugin_package,
                "kinematics_interface::KinematicsInterface");
        kinematics_ = std::unique_ptr<kinematics_interface::KinematicsInterface>(
            kinematics_loader_->createUnmanagedInstance(parameters_.kinematics.plugin_name));
        if (!kinematics_->initialize(
            node->get_node_parameters_interface(), parameters_.kinematics.tip))
        {
            return controller_interface::return_type::ERROR;
        }
    }
    catch (pluginlib::PluginlibException & ex)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("CartesianAdmittanceSolver"),
            "Exception while loading the IK plugin '%s': '%s'",
            parameters_.kinematics.plugin_name.c_str(), ex.what()
        );
        return controller_interface::return_type::ERROR;
    }
    }
    else
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("CartesianAdmittanceSolver"),
            "A differential IK plugin name was not specified in the config file.");
            return controller_interface::return_type::ERROR;
    }

    return controller_interface::return_type::OK;
}


controller_interface::return_type
CartesianAdmittanceSolver::reset(const size_t num_joints)
{
  // Reset admittance state
  admittance_state_ = AdmittanceState(num_joints);

  // Load parameters
  apply_parameters_update();

  return controller_interface::return_type::OK;
}


void CartesianAdmittanceSolver::apply_parameters_update()
{
    if (parameter_handler_->is_old(parameters_))
    {
        parameters_ = parameter_handler_->get_params();
    }
    // update param values
    if(!use_streamed_interaction_parameters_)
    {
        admittance_state_.inertia.setZero();
        admittance_state_.stiffness.setZero();
        admittance_state_.damping.setZero();
        admittance_state_.inertia.diagonal() =
            Eigen::Matrix<double, 6, 1>(parameters_.admittance.inertia.data());
        admittance_state_.stiffness.diagonal() =
            Eigen::Matrix<double, 6, 1>(parameters_.admittance.stiffness.data());
        for (size_t i = 0; i < 6; ++i)
        {
            // Compute damping from damping ratio
            admittance_state_.damping.diagonal()[i] =
                parameters_.admittance.damping_ratio[i] * 2 \
                * sqrt(admittance_state_.inertia.diagonal()[i] * admittance_state_.stiffness.diagonal()[i]);
        }
    }
}

void CartesianAdmittanceSolver::set_interaction_parameters(
    const Eigen::Matrix<double, 6, 6> & desired_inertia,
    const Eigen::Matrix<double, 6, 6> & desired_stiffness,
    const Eigen::Matrix<double, 6, 6> & desired_damping)
{
    use_streamed_interaction_parameters_ = true;
    admittance_state_.inertia = desired_inertia;
    admittance_state_.stiffness = desired_stiffness;
    admittance_state_.damping = desired_damping;
}

controller_interface::return_type
CartesianAdmittanceSolver::update(
    const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state,
    const geometry_msgs::msg::Wrench & measured_wrench,
    const cartesian_control_msgs::msg::CartesianTrajectoryPoint & cartesian_reference,
    const rclcpp::Duration & period,
    trajectory_msgs::msg::JointTrajectoryPoint & joint_state_command)
{
    const double dt = period.seconds();

    if (parameters_.enable_parameter_update_without_reactivation)
    {
        apply_parameters_update();
    }
    // Update current robot state (measured AND desired)
    bool success = update_internal_state(
        current_joint_state,
        measured_wrench,
        cartesian_reference
    );
    // Compute controls
    success &= compute_controls(admittance_state_, dt);

    // If an error is detected, set commanded velocity to zero
    if (!success)
    {
        // Set commanded position to the previous one
        joint_state_command.positions = current_joint_state.positions;
        // Set commanded velocity/acc to zero
        std::fill(
            joint_state_command.velocities.begin(),
            joint_state_command.velocities.end(),
            0
        );
        std::fill(
            joint_state_command.accelerations.begin(),
            joint_state_command.accelerations.end(),
            0
        );
        return controller_interface::return_type::ERROR;
    }
    // Otherwise, set joint command and return
    for (size_t i = 0; i < parameters_.joints.size(); ++i)
    {
        joint_state_command.positions[i] =
            admittance_state_.joint_command_position[i];
        joint_state_command.velocities[i] =
            admittance_state_.joint_command_velocity[i];
        joint_state_command.accelerations[i] =
            admittance_state_.joint_command_acceleration[i];
    }
    return controller_interface::return_type::OK;
}

bool CartesianAdmittanceSolver::update_internal_state(
    const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state,
    const geometry_msgs::msg::Wrench & measured_wrench,
    const cartesian_control_msgs::msg::CartesianTrajectoryPoint & cartesian_reference)
{
    // Update kinematics

    // Update desired pose

    // Update tracking errors

    // Process wrench measurement
    // process_wrench_measurements(measured_wrench, ...)
    return true;
}

bool CartesianAdmittanceSolver::process_wrench_measurements(
    const geometry_msgs::msg::Wrench & measured_wrench,
    const Eigen::Matrix<double, 3, 3> & ft_sensor_world_rot,
    const Eigen::Matrix<double, 3, 3> & com_world_rot)
{
    /* TODO:
         - wrench registration
         - gravity compensation
         - interaction point computation
    */
    admittance_state_.robot_current_wrench.setZero();
    return true;
}


} // namespace cartesian_admittance_controller