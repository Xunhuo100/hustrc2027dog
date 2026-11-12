#ifndef WHEEL_LEG_STATE_RL_H
#define WHEEL_LEG_STATE_RL_H
#include <chrono>
#include <wheel_leg_interfaces/msg/robot_command.hpp>
namespace wheel_leg
{
class State_RL
{
public:
    void enter(uint32_t session);
    bool receive(const wheel_leg_interfaces::msg::RobotCommand &command);
    bool timedOut(double timeout) const;
    bool hasCommand() const { return received_; }
    const wheel_leg_interfaces::msg::RobotCommand &command() const { return command_; }
private:
    uint32_t session_ = 0;
    bool received_ = false;
    std::chrono::steady_clock::time_point command_time_;
    wheel_leg_interfaces::msg::RobotCommand command_;
};
}
#endif
