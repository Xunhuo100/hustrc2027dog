#ifndef WHEEL_LEG_STATE_PASSIVE_H
#define WHEEL_LEG_STATE_PASSIVE_H
#include "FSM/FSMState.h"
namespace wheel_leg
{
class State_Passive : public FSMState
{
public:
    explicit State_Passive(const ControllerConfig &config);
    void enter(const wheel_leg_interfaces::msg::RobotState &state, uint32_t session) override;
    void run(wheel_leg_interfaces::msg::RobotCommand &command) override;
    void exit() override;
    ControlMode checkChange(ControlMode requested) const override;
private:
    const ControllerConfig &config_;
};
}
#endif
