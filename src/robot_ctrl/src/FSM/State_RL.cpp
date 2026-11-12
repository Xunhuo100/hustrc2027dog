#include "FSM/State_RL.h"
#include "common/RobotConfig.h"
#include <cmath>
namespace wheel_leg
{
State_RL::State_RL(const ControllerConfig &config) : config_(config) {}
void State_RL::enter(const wheel_leg_interfaces::msg::RobotState &, uint32_t session)
{
    // 清除上一会话的可用标记，禁止切回 RL 时继续执行旧动作。
    session_ = session;
    received_ = false;
    command_ = wheel_leg_interfaces::msg::RobotCommand();
    command_time_ = std::chrono::steady_clock::now();
}
void State_RL::run(wheel_leg_interfaces::msg::RobotCommand &command)
{
    if (received_)
    {
        // 保持策略消息中的全部目标和 PD，不用控制器的 Stand PD 覆盖。
        command = command_;
        return;
    }
    // 进入 RL 前已经完成 Stand；等待首条指令期间保持最终站姿，最多 policy_timeout。
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        auto &motor = command.motor_cmd[i];
        motor.mode = 1;
        motor.kd = isWheel(i) ? config_.wheel_kd : config_.leg_kd;
        if (!isWheel(i))
        {
            motor.kp = config_.leg_kp;
            motor.q = config_.default_position[i];
        }
    }
}
void State_RL::exit()
{
    received_ = false;
    command_ = wheel_leg_interfaces::msg::RobotCommand();
}
ControlMode State_RL::checkChange(ControlMode requested) const
{
    if (requested == PASSIVE || requested == STAND) return requested;
    return RL;
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
bool State_RL::timedOut() const
{
    // 从最近一次成功接收计时；刚进入 RL 尚无指令时从 enter() 计时。
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - command_time_).count() > config_.policy_timeout;
}
}
