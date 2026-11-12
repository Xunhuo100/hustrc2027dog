#ifndef WHEEL_LEG_FSM_STATE_H
#define WHEEL_LEG_FSM_STATE_H
#include <cstdint>
#include <wheel_leg_interfaces/msg/robot_command.hpp>
#include <wheel_leg_interfaces/msg/robot_state.hpp>
#include "common/RobotConfig.h"
namespace wheel_leg
{
// 状态只负责生命周期、切换条件和统一电机目标，不直接收发 ROS2 消息。
// 切换时调用旧状态 exit()，再以当前反馈和新会话调用新状态 enter()。
class FSMState
{
public:
    virtual ~FSMState() = default;
    virtual void enter(const wheel_leg_interfaces::msg::RobotState &state, uint32_t session) = 0;
    virtual void run(wheel_leg_interfaces::msg::RobotCommand &command) = 0;
    virtual void exit() = 0;
    // 返回允许的下一状态；条件不满足时返回自身状态。
    virtual ControlMode checkChange(ControlMode requested) const = 0;
};
}
#endif
