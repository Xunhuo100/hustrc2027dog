#ifndef WHEEL_LEG_ROBOT_CONTROLLER_H
#define WHEEL_LEG_ROBOT_CONTROLLER_H
#include <std_msgs/msg/string.hpp>
#include "FSM/FSM.h"
#include "interface/RobotIO.h"
namespace wheel_leg
{
// ROS2 入口与全局保护；各状态的目标生成放在 FSM/State_*.cpp。
class RobotController : public rclcpp::Node
{
public:
    RobotController();
private:
    void update();
    bool changeMode(ControlMode mode);
    void modeCallback(std_msgs::msg::String::ConstSharedPtr message);
    void policyCallback(wheel_leg_interfaces::msg::RobotCommand::ConstSharedPtr message);
    RobotIO io_;
    ControllerConfig config_;
    FSM fsm_;
    double state_timeout_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
    rclcpp::Subscription<wheel_leg_interfaces::msg::RobotCommand>::SharedPtr policy_sub_;
};
}
#endif
