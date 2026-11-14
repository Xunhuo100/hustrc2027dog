#ifndef WHEEL_LEG_ROBOT_CONFIG_H
#define WHEEL_LEG_ROBOT_CONFIG_H
#include <array>
#include <vector>
namespace wheel_leg
{
constexpr int MOTOR_COUNT = 16;
enum ControlMode { PASSIVE = 0, STAND = 1, RL = 2 };
const std::array<double, MOTOR_COUNT> DEFAULT_POSITION = {{
    0.1, 0.8, -1.5, 0.0, -0.1, 0.8, -1.5, 0.0,
    0.1, 1.0, -1.5, 0.0, -0.1, 1.0, -1.5, 0.0
}};
inline bool isWheel(int index) { return index % 4 == 3; }

// 状态共用的启动配置；由 RobotController 读取 ROS2 参数，运行时不修改。
struct ControllerConfig
{
    double stand_duration = 1.5;
    double policy_timeout = 0.2;
    double leg_kp = 20.0;
    double leg_kd = 0.5;
    double wheel_kd = 0.3;
    std::vector<double> default_position =
        std::vector<double>(DEFAULT_POSITION.begin(), DEFAULT_POSITION.end());
};
}
#endif
