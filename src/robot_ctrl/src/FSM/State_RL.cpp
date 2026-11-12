#include "FSM/State_RL.h"
#include "common/RobotConfig.h"
#include <cmath>
namespace wheel_leg
{
void State_RL::enter(uint32_t session)
{
    // 清除上一会话的可用标记，禁止切回 RL 时继续执行旧动作。
    session_ = session;
    received_ = false;
    command_time_ = std::chrono::steady_clock::now();
}
bool State_RL::receive(const wheel_leg_interfaces::msg::RobotCommand &command)
{
    // 这里只做协议和数值范围检查，不进行模型推理，也不会把超限指令自动裁剪。
    // 校验失败由 RobotController 切到 Passive；这些上限不是电机额定参数。
    if (command.control_session != session_) return false;
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        const auto &m = command.motor_cmd[i];
        if (!std::isfinite(m.q) || !std::isfinite(m.dq) || !std::isfinite(m.kp) ||
            !std::isfinite(m.kd) || !std::isfinite(m.tau)) return false;
        if (m.mode != 1 || m.kp < 0 || m.kp > 100 || m.kd < 0 || m.kd > 10 ||
            std::abs(m.tau) > 40 || std::abs(m.dq) > 100 || std::abs(m.q) > 5) return false;
        // 当前轮腿协议固定轮为速度控制；新策略的轮指令同样需要 q=0、kp=0。
        if (isWheel(i) && (m.kp != 0 || m.q != 0)) return false;
    }
    command_ = command;
    received_ = true;
    command_time_ = std::chrono::steady_clock::now();
    return true;
}
bool State_RL::timedOut(double timeout) const
{
    // 从最近一次成功接收计时；刚进入 RL 尚无指令时从 enter() 计时。
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - command_time_).count() > timeout;
}
}
