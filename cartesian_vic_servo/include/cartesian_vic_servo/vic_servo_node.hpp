// TODO: add copyright

#ifndef CARTESIAN_VIC_SERVO__VIC_SERVO_NODE_HPP_
#define CARTESIAN_VIC_SERVO__VIC_SERVO_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

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


}




}

#endif  // CARTESIAN_VIC_SERVO__VIC_SERVO_NODE_HPP_
