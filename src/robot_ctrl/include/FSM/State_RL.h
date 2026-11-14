#ifndef WHEEL_LEG_STATE_RL_H
#define WHEEL_LEG_STATE_RL_H
#include <chrono>
#include "FSM/FSMState.h"
namespace wheel_leg
{
class State_RL : public FSMState
{
public:
    explicit State_RL(const ControllerConfig &config);
    void enter(const wheel_leg_interfaces::msg::RobotState &state, uint32_t session) override;
    void run(wheel_leg_interfaces::msg::RobotCommand &command) override;
    void exit() override;
    ControlMode checkChange(ControlMode requested) const override;
    bool receive(const wheel_leg_interfaces::msg::RobotCommand &command);
    bool timedOut() const;
private:
    const ControllerConfig &config_;
    uint32_t session_ = 0;
    bool received_ = false;
    std::chrono::steady_clock::time_point command_time_;
    wheel_leg_interfaces::msg::RobotCommand command_;
};
}
#endif
