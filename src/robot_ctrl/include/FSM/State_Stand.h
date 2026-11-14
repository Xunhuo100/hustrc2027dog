#ifndef WHEEL_LEG_STATE_STAND_H
#define WHEEL_LEG_STATE_STAND_H
#include <array>
#include <chrono>
#include "FSM/FSMState.h"
namespace wheel_leg
{
class State_Stand : public FSMState
{
public:
    explicit State_Stand(const ControllerConfig &config);
    void enter(const wheel_leg_interfaces::msg::RobotState &state, uint32_t session) override;
    void run(wheel_leg_interfaces::msg::RobotCommand &command) override;
    void exit() override;
    ControlMode checkChange(ControlMode requested) const override;
private:
    const ControllerConfig &config_;
    std::array<double, MOTOR_COUNT> start_position_;
    std::chrono::steady_clock::time_point start_time_;
};
}
#endif
