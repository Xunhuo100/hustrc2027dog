#ifndef WHEEL_LEG_FSM_H
#define WHEEL_LEG_FSM_H
#include "FSM/State_Passive.h"
#include "FSM/State_Stand.h"
#include "FSM/State_RL.h"
namespace wheel_leg
{
// 一个 FSM 管理三个常驻状态对象，无需动态创建状态。
// ROS2 模式回调和全局保护均通过 changeMode() 进入同一切换路径。
class FSM
{
public:
    explicit FSM(const ControllerConfig &config);
    bool changeMode(ControlMode requested, const wheel_leg_interfaces::msg::RobotState &state);
    void run(wheel_leg_interfaces::msg::RobotCommand &command);
    bool receivePolicy(const wheel_leg_interfaces::msg::RobotCommand &command);
    bool policyTimedOut() const;
    ControlMode mode() const { return mode_; }
    uint32_t session() const { return session_; }
private:
    State_Passive passive_;
    State_Stand stand_;
    State_RL rl_;
    FSMState *current_;
    ControlMode mode_ = PASSIVE;
    uint32_t session_ = 0;
};
}
#endif
