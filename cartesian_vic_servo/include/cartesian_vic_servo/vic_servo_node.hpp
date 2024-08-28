// TODO: add copyright

#ifndef CARTESIAN_VIC_SERVO__VIC_SERVO_NODE_HPP_
#define CARTESIAN_VIC_SERVO__VIC_SERVO_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/Twist.hpp"
#include "geometry_msgs/msg/Wrench.hpp"
#include "sensor_msgs/msg/JointState.hpp"

#include "cartesian_control_msgs/msg/compliant_frame_trajectory.hpp"
#include "cartesian_control_msgs/msg/vic_controller_state.hpp"

namespace cartesian_vic_servo
{

class CartesianVicServo : rclcpp::Node
{
public:
    explicit CartesianVicServo(std::string node_name = "cartesian_vic_servo_node");
    ~CartesianVicServo = default;

    bool init();

    bool start();

    bool stop();

    bool update();

    sensor_msgs::msg::JointState jointState;
    geometry_msgs::msg::Wrench wrench;
    cartesian_control_msgs::msg::ComplianceFrameTrajectory vicTrajectory;



protected:
    // void callback_new_joint_state_msg(...);
    // + wrench (StampedWrench ou Wrench)
    // + vic ref (voir cartesian_control_msgs)

    bool check_timeout_state();

    // bool send_servo_command(const Eigen::Vector<double, 6> & twist_cmd);

protected:
    /// @brief True if init() called sucessfully
    bool is_initialized_ = false;

    /// @brief True when ready to compute VIC commands
    bool is_ready_ = false;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_joint_state_;
    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr subscriber_wrench_;
    rclcpp::Subscription<cartesian_control_msgs::msg::ComplianceFrameTrajectory>::SharedPtr subscriber_vic_trajectory_;

    // PUblishers
    rclcpp::PUblisher<cartesian_control_msgs::msg::VicControllerState>::SharedPtr publisher_vic_state_;
    rclcpp::PUblisher<geometry_msgs::msg::Twist>::SharedPtr publisher_twist_;

    void jointState_callback(sensor_msgs::msg::JointState msg);


}




}

#endif  // CARTESIAN_VIC_SERVO__VIC_SERVO_NODE_HPP_
