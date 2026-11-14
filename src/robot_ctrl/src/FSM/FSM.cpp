#include "FSM/FSM.h"
namespace wheel_leg
{
FSM::FSM(const ControllerConfig &config)
    : passive_(config), stand_(config), rl_(config), current_(&passive_)
{
    current_->enter(wheel_leg_interfaces::msg::RobotState(), session_);
}
bool FSM::changeMode(ControlMode requested, const wheel_leg_interfaces::msg::RobotState &state)
{
    if (requested != PASSIVE && requested != STAND && requested != RL) return false;
    if (current_->checkChange(requested) != requested) return false;
    if (requested == mode_) return true;
    current_->exit();
    mode_ = requested;
    // 仅真正切换状态时更新会话；ROS 队列中上一会话的 RL 指令会被拒收。
    session_++;
    if (mode_ == PASSIVE) current_ = &passive_;
    else if (mode_ == STAND) current_ = &stand_;
    else current_ = &rl_;
    current_->enter(state, session_);
    return true;
}
void FSM::run(wheel_leg_interfaces::msg::RobotCommand &command)
{
    // 防止从上一周期或上一状态遗留 kp、目标速度、前馈力矩等字段。
    command = wheel_leg_interfaces::msg::RobotCommand();
    current_->run(command);
}
bool FSM::receivePolicy(const wheel_leg_interfaces::msg::RobotCommand &command)
{
    return mode_ == RL && rl_.receive(command);
}
bool FSM::policyTimedOut() const
{
    return mode_ == RL && rl_.timedOut();
}
}
