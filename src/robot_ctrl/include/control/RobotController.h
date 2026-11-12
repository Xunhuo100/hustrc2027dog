#ifndef WHEEL_LEG_ROBOT_CONTROLLER_H
#define WHEEL_LEG_ROBOT_CONTROLLER_H
#include <array>
#include <std_msgs/msg/string.hpp>
#include "FSM/State_RL.h"
#include "interface/RobotIO.h"
#include "common/RobotConfig.h"
namespace wheel_leg
{
class RobotController : public rclcpp::Node
{
public:
    RobotController();
private:
    void update();
    void changeMode(int mode);
    void modeCallback(std_msgs::msg::String::ConstSharedPtr message);
    void policyCallback(wheel_leg_interfaces::msg::RobotCommand::ConstSharedPtr message);
    void setStandCommand(wheel_leg_interfaces::msg::RobotCommand &command);
    RobotIO io_;
    State_RL rl_;
    int mode_ = PASSIVE;
    uint32_t session_ = 0;
    double state_timeout_, policy_timeout_, stand_duration_, leg_kp_, leg_kd_, wheel_kd_;
    std::vector<double> default_position_;
    std::array<double, MOTOR_COUNT> stand_start_;
    std::chrono::steady_clock::time_point stand_time_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
    rclcpp::Subscription<wheel_leg_interfaces::msg::RobotCommand>::SharedPtr policy_sub_;
};
}
#endif
