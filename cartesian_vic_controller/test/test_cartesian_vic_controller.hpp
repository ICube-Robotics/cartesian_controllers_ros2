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

#ifndef TEST_CARTESIAN_VIC_CONTROLLER_HPP_
#define TEST_CARTESIAN_VIC_CONTROLLER_HPP_

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "gmock/gmock.h"

#include "cartesian_vic_controller/cartesian_vic_controller.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "semantic_components/force_torque_sensor.hpp"
#include "test_asset_6d_robot_description.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_kdl/tf2_kdl.hpp"

#include "cartesian_control_msgs/msg/vic_controller_state.hpp"
#include "cartesian_control_msgs/msg/cartesian_trajectory.hpp"
#include "cartesian_control_msgs/msg/compliant_frame_trajectory.hpp"

// TODO(anyone): replace the state and command message types
using ControllerCommandWrenchMsg = geometry_msgs::msg::WrenchStamped;
using ControllerCommandPoseMsg = geometry_msgs::msg::PoseStamped;
using ControllerRefCompliantTrajectoryMsg = cartesian_control_msgs::msg::CompliantFrameTrajectory;
using ControllerStateMsg = cartesian_control_msgs::msg::VicControllerState;

namespace
{
const double COMMON_THRESHOLD = 0.001;

constexpr auto NODE_SUCCESS =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
constexpr auto NODE_ERROR =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;

rclcpp::WaitResultKind wait_for(rclcpp::SubscriptionBase::SharedPtr subscription)
{
  rclcpp::WaitSet wait_set;
  wait_set.add_subscription(subscription);
  const auto timeout = std::chrono::seconds(10);
  return wait_set.wait(timeout).kind();
}

}  // namespace

// subclassing and friending so we can access member variables
class TestableCartesianVicController : public cartesian_vic_controller::
  CartesianVicController
{
  FRIEND_TEST(CartesianVicControllerTest, joint_names_parameter_not_set);
  FRIEND_TEST(CartesianVicControllerTest, interface_parameter_not_set);
  FRIEND_TEST(CartesianVicControllerTest, all_parameters_set_configure_success);
  FRIEND_TEST(CartesianVicControllerTest, check_interfaces);
  FRIEND_TEST(CartesianVicControllerTest, activate_success);
  FRIEND_TEST(CartesianVicControllerTest, receive_message_and_publish_updated_status);

public:
  CallbackReturn on_init() override
  {
    get_node()->declare_parameter("robot_description", rclcpp::ParameterType::PARAMETER_STRING);
    get_node()->declare_parameter(
      "robot_description_semantic", rclcpp::ParameterType::PARAMETER_STRING);
    get_node()->set_parameter({"robot_description", robot_description_});
    get_node()->set_parameter({"robot_description_semantic", robot_description_semantic_});

    return cartesian_vic_controller::CartesianVicController::on_init();
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override
  {
    auto ret = cartesian_vic_controller::CartesianVicController::on_configure(
      previous_state);
    // Only if on_configure is successful create subscription
    if (ret == CallbackReturn::SUCCESS) {
      input_pose_command_subscriber_wait_set_.add_subscription(
        input_compliant_frame_trajectory_subscriber_);
    }
    return ret;
  }

  /**
   * @brief wait_for_commands blocks until a new ControllerCommandMsg is received.
   * Requires that the executor is not spinned elsewhere between the
   *  message publication and the call to this function.
   *
   * @return true if new ControllerCommandMsg msg was received, false if timeout.
   */
  bool wait_for_commands(
    rclcpp::Executor & executor,
    const std::chrono::milliseconds & timeout = std::chrono::milliseconds{500})
  {
    bool success =
      input_pose_command_subscriber_wait_set_.wait(timeout).kind() == rclcpp::WaitResultKind::Ready;

    if (success) {
      executor.spin_some();
    }
    return success;
  }

private:
  rclcpp::WaitSet input_pose_command_subscriber_wait_set_;
  const std::string robot_description_ = ros2_control_test_assets::valid_6d_robot_urdf;
  const std::string robot_description_semantic_ = ros2_control_test_assets::valid_6d_robot_srdf;
};

class CartesianVicControllerTest : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    //    rclcpp::init(0, nullptr);
  }

  void SetUp()
  {
    // initialize controller
    controller_ = std::make_unique<TestableCartesianVicController>();

    command_publisher_node_ = std::make_shared<rclcpp::Node>("command_publisher");

    compliant_frame_trajectory_publisher_ =
      command_publisher_node_->create_publisher<ControllerRefCompliantTrajectoryMsg>(
      "/test_cartesian_vic_controller/compliant_frame_trajectory",
      rclcpp::SystemDefaultsQoS());

    test_subscription_node_ = std::make_shared<rclcpp::Node>("test_subscription_node");
    test_broadcaster_node_ = std::make_shared<rclcpp::Node>("test_broadcaster_node");
  }

  static void TearDownTestCase()
  {
    //    rclcpp::shutdown();
  }

  void TearDown() {controller_.reset(nullptr);}

protected:
  controller_interface::return_type SetUpController(
    const std::string & controller_name, const std::vector<rclcpp::Parameter> & parameter_overrides)
  {
    auto options = rclcpp::NodeOptions()
      .allow_undeclared_parameters(false)
      .parameter_overrides(parameter_overrides)
      .automatically_declare_parameters_from_overrides(true);
    return SetUpControllerCommon(controller_name, options);
  }

  controller_interface::return_type SetUpController(
    const std::string & controller_name = "test_cartesian_vic_controller")
  {
    auto options = rclcpp::NodeOptions()
      .allow_undeclared_parameters(false)
      .automatically_declare_parameters_from_overrides(true);
    return SetUpControllerCommon(controller_name, options);
  }

  controller_interface::return_type SetUpControllerCommon(
    const std::string & controller_name, const rclcpp::NodeOptions & options)
  {
    auto result = controller_->init(controller_name, "", options);
    // auto result = controller_->init(controller_name, "", 0, "", options);

    controller_->export_reference_interfaces();
    assign_interfaces();

    return result;
  }

  void assign_interfaces()
  {
    std::vector<hardware_interface::LoanedCommandInterface> command_ifs;
    command_itfs_.reserve(joint_command_values_.size());
    command_ifs.reserve(joint_command_values_.size());

    for (auto i = 0u; i < joint_command_values_.size(); ++i) {
      command_itfs_.emplace_back(
        hardware_interface::CommandInterface(
          joint_names_[i], command_interface_types_[0], &joint_command_values_[i]));
      command_ifs.emplace_back(command_itfs_.back());
    }

    auto sc_fts = semantic_components::ForceTorqueSensor(ft_sensor_name_);
    fts_state_names_ = sc_fts.get_state_interface_names();
    std::vector<hardware_interface::LoanedStateInterface> state_ifs;

    const size_t num_state_ifs = 2 * joint_state_position_values_.size() + fts_state_names_.size();
    state_itfs_.reserve(num_state_ifs);
    state_ifs.reserve(num_state_ifs);

    for (auto i = 0u; i < joint_state_position_values_.size(); ++i) {
      state_itfs_.emplace_back(
        hardware_interface::StateInterface(
          joint_names_[i], state_interface_types_[0], &joint_state_position_values_[i]));
      state_ifs.emplace_back(state_itfs_.back());
    }

    for (auto i = 0u; i < joint_state_velocity_values_.size(); ++i) {
      state_itfs_.emplace_back(
        hardware_interface::StateInterface(
          joint_names_[i], state_interface_types_[1], &joint_state_velocity_values_[i]));
      state_ifs.emplace_back(state_itfs_.back());
    }

    std::vector<std::string> fts_itf_names =
    {"force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z"};

    for (auto i = 0u; i < fts_state_names_.size(); ++i) {
      state_itfs_.emplace_back(
        hardware_interface::StateInterface(
          ft_sensor_name_, fts_itf_names[i], &fts_state_values_[i]));
      state_ifs.emplace_back(state_itfs_.back());
    }

    controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));
  }

  void broadcast_tfs()
  {
    static tf2_ros::TransformBroadcaster br(test_broadcaster_node_);
    geometry_msgs::msg::TransformStamped transform_stamped;

    transform_stamped.header.stamp = test_broadcaster_node_->now();
    transform_stamped.header.frame_id = fixed_world_frame_;
    transform_stamped.transform.translation.x = 1.3;
    transform_stamped.transform.translation.y = 0.5;
    transform_stamped.transform.translation.z = 0.5;
    transform_stamped.transform.rotation.x = 0;
    transform_stamped.transform.rotation.y = 0;
    transform_stamped.transform.rotation.z = 0;
    transform_stamped.transform.rotation.w = 1;

    transform_stamped.child_frame_id = ik_base_frame_;
    br.sendTransform(transform_stamped);

    transform_stamped.child_frame_id = ik_tip_frame_;
    br.sendTransform(transform_stamped);

    transform_stamped.header.frame_id = ik_tip_frame_;
    transform_stamped.transform.translation.x = 0;
    transform_stamped.transform.translation.y = 0;
    transform_stamped.transform.translation.z = 0;
    transform_stamped.transform.rotation.x = 0;
    transform_stamped.transform.rotation.y = 0;
    transform_stamped.transform.rotation.z = 0;
    transform_stamped.transform.rotation.w = 1;

    transform_stamped.child_frame_id = control_frame_;
    br.sendTransform(transform_stamped);

    transform_stamped.transform.translation.z = 0.05;
    transform_stamped.child_frame_id = sensor_frame_;
    br.sendTransform(transform_stamped);

    transform_stamped.transform.translation.z = 0.2;
    transform_stamped.child_frame_id = endeffector_frame_;
    br.sendTransform(transform_stamped);
  }

  void subscribe_and_get_messages(ControllerStateMsg & msg)
  {
    // create a new subscriber
    auto subs_callback = [&](const ControllerStateMsg::SharedPtr) {};
    auto subscription = test_subscription_node_->create_subscription<ControllerStateMsg>(
      "/test_cartesian_vic_controller/status", 10, subs_callback);

    // call update to publish the test value
    ASSERT_EQ(
      controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
      controller_interface::return_type::OK);

    // wait for message to be passed
    ASSERT_EQ(wait_for(subscription), rclcpp::WaitResultKind::Ready);

    // take message from subscription
    rclcpp::MessageInfo msg_info;
    ASSERT_TRUE(subscription->take(msg, msg_info));
  }

  void publish_commands()
  {
    auto wait_for_topic = [&](const auto topic_name)
      {
        size_t wait_count = 0;
        while (command_publisher_node_->count_subscribers(topic_name) == 0) {
          if (wait_count >= 5) {
            auto error_msg =
              std::string("publishing to ") + topic_name + " but no node subscribes to it";
            throw std::runtime_error(error_msg);
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          ++wait_count;
        }
      };

    wait_for_topic(compliant_frame_trajectory_publisher_->get_topic_name());

    ControllerRefCompliantTrajectoryMsg ref_compliant_frame_traj_msg;
    int N = 1;
    ref_compliant_frame_traj_msg.cartesian_trajectory_points.reserve(N);
    ref_compliant_frame_traj_msg.compliance_at_points.reserve(N);

    // for (auto index = 0u; index < N; index++) {
    // }
    cartesian_control_msgs::msg::CartesianTrajectoryPoint cartesian_trajectory_point;

    try {
      kinematics_loader_ =
        std::make_shared<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>(
        "kinematics_interface",
        "kinematics_interface::KinematicsInterface"
        );
      kinematics_ = std::unique_ptr<kinematics_interface::KinematicsInterface>(
        kinematics_loader_->createUnmanagedInstance(
          "kinematics_interface_kdl/KinematicsInterfaceKDL"));

      kinematics_->initialize(
        controller_->get_node()->get_node_parameters_interface(), ik_tip_frame_);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        controller_->get_node()->get_logger(),
        "Exception thrown from 'publish_commands()' test util with message: %s \n",
        e.what()
      );
    }

    std::string tcp_frame = endeffector_frame_;
    Eigen::Isometry3d robot_reference_pose;
    Eigen::Matrix<double, 6, 1> robot_reference_velocity = Eigen::Matrix<double, 6, 1>::Zero();

    std::vector<double> joint_state_values(joint_state_position_values_.begin(),
      joint_state_position_values_.end());
    kinematics_->calculate_link_transform(
      joint_state_values,
      tcp_frame,
      robot_reference_pose
    );

    cartesian_trajectory_point.pose = Eigen::toMsg(robot_reference_pose);
    cartesian_trajectory_point.velocity = Eigen::toMsg(robot_reference_velocity);

    cartesian_control_msgs::msg::CartesianCompliance desired_compliance;

    auto eigen_to_multiarray = [](Eigen::Matrix<double, 6, 6> & mat)
      {
        // Now we can convert to a message
        std_msgs::msg::Float64MultiArray msg;
        msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
        msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
        msg.layout.dim[0].label = "rows";
        msg.layout.dim[0].size = 6;
        msg.layout.dim[0].stride = 6 * 6;
        msg.layout.dim[1].label = "cols";
        msg.layout.dim[1].size = 6;
        msg.layout.dim[1].stride = 6;
        msg.layout.data_offset = 0;
        std::vector<double> vec;
        vec.resize(mat.size());
        Eigen::Map<Eigen::VectorXd> mvec(mat.data(), mat.size());
        Eigen::VectorXd::Map(&vec[0], mvec.size()) = mvec;
        msg.data = vec;
        return msg;
      };

    Eigen::Matrix<double, 6, 6> M = Eigen::DiagonalMatrix<double, 6>(1, 2, 3, 0.1, 0.2, 0.3);
    Eigen::Matrix<double, 6, 6> K = Eigen::DiagonalMatrix<double, 6>(100, 200, 300, 10, 20, 30);
    Eigen::Matrix<double, 6, 6> D = Eigen::DiagonalMatrix<double, 6>(20, 30, 40, 5, 8, 12);

    desired_compliance.inertia = eigen_to_multiarray(M);
    desired_compliance.stiffness = eigen_to_multiarray(K);
    desired_compliance.damping = eigen_to_multiarray(D);

    ref_compliant_frame_traj_msg.cartesian_trajectory_points.emplace_back(
      cartesian_trajectory_point);
    ref_compliant_frame_traj_msg.compliance_at_points.emplace_back(desired_compliance);

    compliant_frame_trajectory_publisher_->publish(ref_compliant_frame_traj_msg);
  }

protected:
  // TODO(anyone): adjust the members as needed

  // Controller-related parameters
  const std::vector<std::string> joint_names_ = {
    "joint_a1", "joint_a2", "joint_a3",
    "joint_a4", "joint_a5", "joint_a6"
  };
  const std::vector<std::string> command_interface_types_ = {"position"};
  const std::vector<std::string> state_interface_types_ = {"position", "velocity"};
  const std::string ft_sensor_name_ = "ft_sensor_name";

  bool hardware_state_has_offset_ = false;

  const std::string ik_base_frame_ = "base_link";
  const std::string ik_tip_frame_ = "tool0";
  const std::string ik_group_name_ = "arm";
  //  const std::string robot_description_ = ros2_control_test_assets::valid_6d_robot_urdf;
  //  const std::string robot_description_semantic_ = ros2_control_test_assets::valid_6d_robot_srdf;

  const std::string control_frame_ = "tool0";
  const std::string endeffector_frame_ = "endeffector_frame";
  const std::string fixed_world_frame_ = "fixed_world_frame";
  const std::string sensor_frame_ = "tool0";

  std::array<bool, 6> vic_selected_axes_ = {true, true, true, true, true, true};
  std::array<double, 6> vic_inertia_ = {5.5, 6.6, 7.7, 8.8, 9.9, 10.10};
  std::array<double, 6> vic_damping_ratio_ = {2.828427, 2.828427, 2.828427,
    2.828427, 2.828427, 2.828427};
  std::array<double, 6> vic_stiffness_ = {214.1, 214.2, 214.3, 214.4, 214.5, 214.6};

  std::array<double, 6> joint_command_values_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 6> joint_state_position_values_ = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5};

  std::array<double, 6> joint_state_velocity_values_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 6> fts_state_values_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<std::string> fts_state_names_;

  std::vector<hardware_interface::StateInterface> state_itfs_;
  std::vector<hardware_interface::CommandInterface> command_itfs_;

  // Test related parameters
  std::unique_ptr<TestableCartesianVicController> controller_;
  rclcpp::Node::SharedPtr command_publisher_node_;
  rclcpp::Publisher<ControllerRefCompliantTrajectoryMsg>::SharedPtr
    compliant_frame_trajectory_publisher_;
  rclcpp::Node::SharedPtr test_subscription_node_;
  rclcpp::Node::SharedPtr test_broadcaster_node_;


  // Kinematics interface plugin loader
  std::shared_ptr<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>
  kinematics_loader_;

  /// Kinematics interface
  std::unique_ptr<kinematics_interface::KinematicsInterface> kinematics_;
};

// From the tutorial: https://www.sandordargo.com/blog/2019/04/24/parameterized-testing-with-gtest
class VicControllerTestParameterizedMissingParameters
  : public CartesianVicControllerTest,
  public ::testing::WithParamInterface<std::string>
{
public:
  virtual void SetUp()
  {
    CartesianVicControllerTest::SetUp();
    auto node = std::make_shared<rclcpp::Node>("test_cartesian_vic_controller");
    overrides_ = node->get_node_parameters_interface()->get_parameter_overrides();
  }

  static void TearDownTestCase() {CartesianVicControllerTest::TearDownTestCase();}

protected:
  controller_interface::return_type SetUpController(const std::string & remove_name)
  {
    std::vector<rclcpp::Parameter> parameter_overrides;
    for (const auto & override : overrides_) {
      if (override.first != remove_name) {
        rclcpp::Parameter param(override.first, override.second);
        parameter_overrides.push_back(param);
      }
    }

    return CartesianVicControllerTest::SetUpController(
      "test_vic_controller_no_overrides", parameter_overrides);
  }

  std::map<std::string, rclcpp::ParameterValue> overrides_;
};

// From the tutorial: https://www.sandordargo.com/blog/2019/04/24/parameterized-testing-with-gtest
class VicControllerTestParameterizedInvalidParameters
  : public CartesianVicControllerTest,
  public ::testing::WithParamInterface<std::tuple<std::string, rclcpp::ParameterValue>>
{
public:
  virtual void SetUp() {CartesianVicControllerTest::SetUp();}

  static void TearDownTestCase() {CartesianVicControllerTest::TearDownTestCase();}

protected:
  controller_interface::return_type SetUpController()
  {
    auto param_name = std::get<0>(GetParam());
    auto param_value = std::get<1>(GetParam());
    std::vector<rclcpp::Parameter> parameter_overrides;
    rclcpp::Parameter param(param_name, param_value);
    parameter_overrides.push_back(param);
    return CartesianVicControllerTest::SetUpController(
      "test_cartesian_vic_controller", parameter_overrides);
  }
};

#endif  // TEST_CARTESIAN_VIC_CONTROLLER_HPP_
