#include "FSM/State_Passive.h"
namespace wheel_leg
{
State_Passive::State_Passive(const ControllerConfig &config) : config_(config) {}
void State_Passive::enter(const wheel_leg_interfaces::msg::RobotState &, uint32_t) {}
void State_Passive::run(wheel_leg_interfaces::msg::RobotCommand &command)
{
    // 每周期从空指令开始：kp/q/dq/tau 均为零，只提供速度阻尼。
    // 反馈失效时，控制器会在最终发送前覆盖为 mode=0 的禁用指令。
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        command.motor_cmd[i].mode = 1;
        command.motor_cmd[i].kd = isWheel(i) ? config_.wheel_kd : 1.0;
    }
}
void State_Passive::exit() {}
ControlMode State_Passive::checkChange(ControlMode requested) const
{
    // 进入 RL 必须先经过 Stand。
    return requested == STAND ? STAND : PASSIVE;
}
}
