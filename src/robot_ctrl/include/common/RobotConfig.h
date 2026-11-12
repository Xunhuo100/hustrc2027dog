#ifndef WHEEL_LEG_ROBOT_CONFIG_H
#define WHEEL_LEG_ROBOT_CONFIG_H
#include <array>
namespace wheel_leg
{
constexpr int MOTOR_COUNT = 16;
enum ControlMode { PASSIVE = 0, STAND = 1, RL = 2 };
const std::array<double, MOTOR_COUNT> DEFAULT_POSITION = {{
    0.1, 0.8, -1.5, 0.0, -0.1, 0.8, -1.5, 0.0,
    0.1, 1.0, -1.5, 0.0, -0.1, 1.0, -1.5, 0.0
}};
inline bool isWheel(int index) { return index % 4 == 3; }
}
#endif
