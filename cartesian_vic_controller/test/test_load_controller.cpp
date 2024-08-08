// Copyright (c) 2021, PickNik, Inc.
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

#include <gmock/gmock.h>
#include <memory>

#include "controller_manager/controller_manager.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/utilities.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

TEST(TestLoadVicController, load_controller)
{
  std::shared_ptr<rclcpp::Executor> executor =
    std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  // Warning: this is the constructor syntax for humble...
  // see https://github.com/ros-controls/ros2_control/blob/42107676e96d6517c258cb9c06e0adcfbb23fc3b/controller_manager/src/controller_manager.cpp#L206-L209
  controller_manager::ControllerManager cm(
    std::make_unique<hardware_interface::ResourceManager>(
      ros2_control_test_assets::valid_6d_robot_urdf),
    executor,
    "test_controller_manager"
  );

  ASSERT_EQ(
    cm.load_controller(
      "test_cartesian_vic_controller",
      "cartesian_vic_controller/CartesianVicController"),
    nullptr);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
