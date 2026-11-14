#include "FSM/State_Stand.h"
#include <algorithm>
namespace wheel_leg
{
State_Stand::State_Stand(const ControllerConfig &config) : config_(config) {}
void State_Stand::enter(const wheel_leg_interfaces::msg::RobotState &state, uint32_t)
{
    // 每次真正进入 Stand 都重新记录起点；重复的 stand 请求不重新计时。
    start_time_ = std::chrono::steady_clock::now();
    for (int i = 0; i < MOTOR_COUNT; i++) start_position_[i] = state.motor_state[i].q;
}
void State_Stand::run(wheel_leg_interfaces::msg::RobotCommand &command)
{
    double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();
    double ratio = std::min(1.0, elapsed / config_.stand_duration);
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        auto &motor = command.motor_cmd[i];
        motor.mode = 1;
        motor.kd = isWheel(i) ? config_.wheel_kd : config_.leg_kd;
        if (!isWheel(i))
        {
            motor.kp = config_.leg_kp;
            motor.q = start_position_[i] + ratio * (config_.default_position[i] - start_position_[i]);
        }
    }
}
void State_Stand::exit() {}
ControlMode State_Stand::checkChange(ControlMode requested) const
{
    if (requested == PASSIVE) return PASSIVE;
    double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();
    if (requested == RL && elapsed >= config_.stand_duration) return RL;
    return STAND;
}
}
