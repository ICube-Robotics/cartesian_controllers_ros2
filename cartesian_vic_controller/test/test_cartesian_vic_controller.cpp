// Copyright (c) 2021, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#include "test_cartesian_vic_controller.hpp"

#include <limits>
#include <memory>
#include <utility>
#include <vector>

// Test on_configure returns ERROR when a required parameter is missing
TEST_P(VicControllerTestParameterizedMissingParameters, one_parameter_is_missing)
{
  ASSERT_EQ(SetUpController(GetParam()), controller_interface::return_type::ERROR);
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);
}

INSTANTIATE_TEST_SUITE_P(
  MissingMandatoryParameterDuringConfiguration,
  VicControllerTestParameterizedMissingParameters,
  ::testing::Values(
    "state_interfaces", "command_interfaces", "joints",
    "control.frame.id",
    // ft sensor parameters
    "ft_sensor.frame.id", "ft_sensor.name",
    // gravity compensation parameters
    "fixed_world_frame.frame.id", "gravity_compensation.frame.id",
    "gravity_compensation.CoG.pos", "gravity_compensation.CoG.force",
    // dynamics model parameters
    "dynamics.plugin_package", "dynamics.plugin_name", "dynamics.base", "dynamics.tip",
    // VIC rule parameters
    "vic.frame.id", "vic.plugin_name", "vic.plugin_package",
    "vic.selected_axes", "vic.inertia", "vic.stiffness", "vic.damping_ratio"
  ));

INSTANTIATE_TEST_SUITE_P(
  InvalidParameterDuringConfiguration, VicControllerTestParameterizedInvalidParameters,
  ::testing::Values(
    // wrong length COG
    std::make_tuple(
      std::string("gravity_compensation.CoG.pos"),
      rclcpp::ParameterValue(std::vector<double>() = {1, 2, 3, 4})),
    // wrong length stiffness
    std::make_tuple(
      std::string("vic.stiffness"),
      rclcpp::ParameterValue(std::vector<double>() = {1, 2, 3})),
    // negative stiffness
    std::make_tuple(
      std::string("vic.stiffness"),
      rclcpp::ParameterValue(std::vector<double>() = {-1, -2, 3, 4, 5, 6})),
    // wrong length inertia
    std::make_tuple(
      std::string("vic.inertia"), rclcpp::ParameterValue(std::vector<double>() = {1, 2, 3})),
    // negative inertia
    std::make_tuple(
      std::string("vic.inertia"),
      rclcpp::ParameterValue(std::vector<double>() = {-1, -2, 3, 4, 5, 6})),
    // wrong length damping ratio
    std::make_tuple(
      std::string("vic.damping_ratio"),
      rclcpp::ParameterValue(std::vector<double>() = {1, 2, 3})),
    // wrong length selected axes
    std::make_tuple(
      std::string("vic.selected_axes"),
      rclcpp::ParameterValue(std::vector<double>() = {1, 2, 3}))
    // invalid robot description.
    // TODO(anyone): deactivated, because SetUpController returns SUCCESS here?
    // ,std::make_tuple(
    //   std::string("robot_description"), rclcpp::ParameterValue(std::string() = "bad_robot")))
));

// Test on_init returns ERROR when a parameter is invalid
TEST_P(VicControllerTestParameterizedInvalidParameters, invalid_parameters)
{
  ASSERT_EQ(SetUpController(), controller_interface::return_type::ERROR);
}

TEST_F(CartesianVicControllerTest, all_parameters_set_configure_success)
{
  auto result = SetUpController();

  ASSERT_EQ(result, controller_interface::return_type::OK);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_TRUE(!controller_->vic_->parameters_.joints.empty());
  ASSERT_TRUE(controller_->vic_->parameters_.joints.size() == joint_names_.size());
  ASSERT_TRUE(
    std::equal(
      controller_->vic_->parameters_.joints.begin(),
      controller_->vic_->parameters_.joints.end(), joint_names_.begin(),
      joint_names_.end()));

  ASSERT_TRUE(!controller_->vic_->parameters_.command_interfaces.empty());
  ASSERT_TRUE(
    controller_->vic_->parameters_.command_interfaces.size() ==
    command_interface_types_.size());
  ASSERT_TRUE(
    std::equal(
      controller_->vic_->parameters_.command_interfaces.begin(),
      controller_->vic_->parameters_.command_interfaces.end(),
      command_interface_types_.begin(), command_interface_types_.end()));

  ASSERT_TRUE(!controller_->vic_->parameters_.state_interfaces.empty());
  ASSERT_TRUE(
    controller_->vic_->parameters_.state_interfaces.size() == state_interface_types_.size());
  ASSERT_TRUE(
    std::equal(
      controller_->vic_->parameters_.state_interfaces.begin(),
      controller_->vic_->parameters_.state_interfaces.end(), state_interface_types_.begin(),
      state_interface_types_.end()));

  ASSERT_EQ(controller_->vic_->parameters_.ft_sensor.name, ft_sensor_name_);
  ASSERT_EQ(controller_->vic_->parameters_.dynamics.base, ik_base_frame_);
  ASSERT_EQ(controller_->vic_->parameters_.ft_sensor.frame.id, sensor_frame_);

  ASSERT_TRUE(!controller_->vic_->parameters_.vic.selected_axes.empty());
  ASSERT_TRUE(
    controller_->vic_->parameters_.vic.selected_axes.size() ==
    vic_selected_axes_.size());
  ASSERT_TRUE(
    std::equal(
      controller_->vic_->parameters_.vic.selected_axes.begin(),
      controller_->vic_->parameters_.vic.selected_axes.end(),
      vic_selected_axes_.begin(), vic_selected_axes_.end()));

  ASSERT_TRUE(!controller_->vic_->parameters_.vic.inertia.empty());
  ASSERT_TRUE(
    controller_->vic_->parameters_.vic.inertia.size() == vic_inertia_.size());
  ASSERT_TRUE(
    std::equal(
      controller_->vic_->parameters_.vic.inertia.begin(),
      controller_->vic_->parameters_.vic.inertia.end(), vic_inertia_.begin(),
      vic_inertia_.end()));

  ASSERT_TRUE(!controller_->vic_->parameters_.vic.damping_ratio.empty());
  ASSERT_TRUE(
    controller_->vic_->parameters_.vic.damping_ratio.size() ==
    vic_damping_ratio_.size());
  ASSERT_TRUE(
    std::equal(
      controller_->vic_->parameters_.vic.damping_ratio.begin(),
      controller_->vic_->parameters_.vic.damping_ratio.end(),
      vic_damping_ratio_.begin(), vic_damping_ratio_.end()));

  ASSERT_TRUE(!controller_->vic_->parameters_.vic.stiffness.empty());
  ASSERT_TRUE(
    controller_->vic_->parameters_.vic.stiffness.size() ==
    vic_stiffness_.size());
  ASSERT_TRUE(
    std::equal(
      controller_->vic_->parameters_.vic.stiffness.begin(),
      controller_->vic_->parameters_.vic.stiffness.end(),
      vic_stiffness_.begin(),
      vic_stiffness_.end()));
}

TEST_F(CartesianVicControllerTest, check_interfaces)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  auto command_interfaces = controller_->command_interface_configuration();
  ASSERT_EQ(command_interfaces.names.size(), joint_command_values_.size());

  ASSERT_EQ(
    controller_->command_interfaces_.size(),
    command_interface_types_.size() * joint_names_.size());

  auto state_interfaces = controller_->state_interface_configuration();
  ASSERT_EQ(
    state_interfaces.names.size(),
    state_interface_types_.size() * joint_state_position_values_.size() + fts_state_values_.size());
  ASSERT_EQ(
    controller_->state_interfaces_.size(),
    state_interface_types_.size() * joint_names_.size() + fts_state_values_.size());
}

TEST_F(CartesianVicControllerTest, activate_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(
    controller_->command_interfaces_.size(), command_interface_types_.size() * joint_names_.size());
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
}

TEST_F(CartesianVicControllerTest, update_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  broadcast_tfs();
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
}

TEST_F(CartesianVicControllerTest, deactivate_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_deactivate(rclcpp_lifecycle::State()), NODE_SUCCESS);
}

TEST_F(CartesianVicControllerTest, reactivate_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_deactivate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  assign_interfaces();
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  broadcast_tfs();
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
}

TEST_F(CartesianVicControllerTest, publish_status_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  broadcast_tfs();
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  ControllerStateMsg msg;
  subscribe_and_get_messages(msg);

  //   // Check that wrench command are all zero since not used
  //   ASSERT_EQ(msg.wrench_base.header.frame_id, ik_base_frame_);
  //   ASSERT_EQ(msg.wrench_base.wrench.force.x, 0.0);
  //   ASSERT_EQ(msg.wrench_base.wrench.force.y, 0.0);
  //   ASSERT_TRUE(msg.wrench_base.wrench.force.z > 0.15);
  //   ASSERT_TRUE(msg.wrench_base.wrench.torque.x != 0.0);
  //   ASSERT_TRUE(msg.wrench_base.wrench.torque.y != 0.0);
  //   ASSERT_EQ(msg.wrench_base.wrench.torque.z, 0.0);

  //   // Check joint command message
  //   for (auto i = 0ul; i < joint_names_.size(); i++)
  //   {
  //     ASSERT_EQ(joint_names_[i], msg.joint_state.name[i]);
  //     ASSERT_FALSE(std::isnan(msg.joint_state.position[i]));
  //     ASSERT_FALSE(std::isnan(msg.joint_state.velocity[i]));
  //     ASSERT_FALSE(std::isnan(msg.joint_state.effort[i]));
  //   }
}

TEST_F(CartesianVicControllerTest, receive_message_and_publish_updated_status)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  broadcast_tfs();
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // After first update state, commanded position should be near the start state
  for (auto i = 0ul; i < joint_state_position_values_.size(); i++) {
    ASSERT_NEAR(joint_state_position_values_[i], joint_command_values_[i], COMMON_THRESHOLD);
  }
  /*
  //TODO(tpoignonec)
  ControllerStateMsg msg;
  subscribe_and_get_messages(msg);
  //   ASSERT_EQ(msg.wrench_base.header.frame_id, ik_base_frame_);
  //   ASSERT_EQ(msg.wrench_base.header.frame_id, ik_base_frame_);

  publish_commands();
  ASSERT_TRUE(controller_->wait_for_commands(executor));

  broadcast_tfs();
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_NEAR(joint_command_values_[0], joint_state_values_[0], COMMON_THRESHOLD);

  subscribe_and_get_messages(msg);
  */
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}

// Add test, wrong interfaces
