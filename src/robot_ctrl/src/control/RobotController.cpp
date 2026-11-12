#include "control/RobotController.h"
#include <cmath>
#include <functional>
#include <stdexcept>
namespace wheel_leg
{
RobotController::RobotController() : Node("robot_ctrl"), io_(this), fsm_(config_)
{
    using std::placeholders::_1;
    // 控制层默认 200 Hz；策略默认 50 Hz，状态对象复用最近一条有效策略指令。
    double period = declare_parameter("control_period", 0.005);
    state_timeout_ = declare_parameter("state_timeout", 0.1);
    config_.policy_timeout = declare_parameter("policy_timeout", config_.policy_timeout);
    config_.stand_duration = declare_parameter("stand_duration", config_.stand_duration);
    config_.leg_kp = declare_parameter("leg_kp", config_.leg_kp);
    config_.leg_kd = declare_parameter("leg_kd", config_.leg_kd);
    config_.wheel_kd = declare_parameter("wheel_kd", config_.wheel_kd);
    config_.default_position = declare_parameter<std::vector<double>>("default_position", config_.default_position);
    if (!std::isfinite(period) || period <= 0 || !std::isfinite(config_.stand_duration) ||
        config_.stand_duration <= 0 || !std::isfinite(state_timeout_) || state_timeout_ <= 0 ||
        !std::isfinite(config_.policy_timeout) || config_.policy_timeout <= 0 ||
        !std::isfinite(config_.leg_kp) || config_.leg_kp < 0 || config_.leg_kp > 100 ||
        !std::isfinite(config_.leg_kd) || config_.leg_kd < 0 || config_.leg_kd > 10 ||
        !std::isfinite(config_.wheel_kd) || config_.wheel_kd < 0 || config_.wheel_kd > 10 ||
        config_.default_position.size() != MOTOR_COUNT)
        throw std::runtime_error("Invalid controller periods, gains or default_position");
    for (double q : config_.default_position)
        if (!std::isfinite(q) || std::abs(q) > 5) throw std::runtime_error("Invalid default_position");
    mode_sub_ = create_subscription<std_msgs::msg::String>("/robot/mode", 1,
        std::bind(&RobotController::modeCallback, this, _1));
    policy_sub_ = create_subscription<wheel_leg_interfaces::msg::RobotCommand>("/robot/rl_cmd", 1,
        std::bind(&RobotController::policyCallback, this, _1));
    timer_ = create_wall_timer(std::chrono::duration<double>(period), std::bind(&RobotController::update, this));
    RCLCPP_INFO(get_logger(), "16 DOF ready: passive -> stand -> rl");
}
bool RobotController::changeMode(ControlMode mode)
{
    ControlMode previous = fsm_.mode();
    if (!fsm_.changeMode(mode, io_.getState())) return false;
    if (fsm_.mode() != previous)
        RCLCPP_INFO(get_logger(), "Control mode: %d, session: %u", fsm_.mode(), fsm_.session());
    return true;
}
void RobotController::modeCallback(std_msgs::msg::String::ConstSharedPtr message)
{
    // ROS2 回调只解析请求；状态自身通过 checkChange() 决定是否允许切换。
    if (message->data == "passive") { changeMode(PASSIVE); return; }
    if (!io_.stateReady(state_timeout_)) { RCLCPP_WARN(get_logger(), "No fresh motor/IMU feedback"); return; }
    if (message->data == "stand") changeMode(STAND);
    else if (message->data == "rl" && fsm_.mode() == STAND && changeMode(RL)) return;
    else RCLCPP_WARN(get_logger(), "Use passive/stand/rl; RL requires a completed stand");
}
void RobotController::policyCallback(wheel_leg_interfaces::msg::RobotCommand::ConstSharedPtr message)
{
    if (fsm_.mode() != RL) return;
    // 检查消息自身年龄；State_RL 另用单调时钟检测指令断流。
    // 本节点与策略节点需要一致的 ROS 时钟，允许最多 50 ms 的未来偏差。
    if (message->header.stamp.sec < 0 || message->header.stamp.nanosec >= 1000000000)
    {
        RCLCPP_WARN(get_logger(), "Rejected invalid RL timestamp");
        changeMode(PASSIVE);
        return;
    }
    double age = (now() - rclcpp::Time(message->header.stamp)).seconds();
    if (age < -0.05 || age > config_.policy_timeout || !fsm_.receivePolicy(*message))
    {
        RCLCPP_WARN(get_logger(), "Rejected stale, invalid or wrong-session RL command");
        changeMode(PASSIVE);
    }
}
void RobotController::update()
{
    // 全局保护先于状态执行；故障切换也经过同一 exit -> enter 路径。
    bool ready = io_.stateReady(state_timeout_);
    if (!ready) changeMode(PASSIVE);
    if (ready && fsm_.mode() != PASSIVE)
    {
        const auto &q = io_.getState().imu.orientation;
        // 机体 z 轴与世界 z 轴夹角的余弦，0.5 对应总倾角 60°。
        if (1.0 - 2.0 * (q.x * q.x + q.y * q.y) < 0.5)
        { RCLCPP_WARN(get_logger(), "Tilt exceeded 60 degrees"); changeMode(PASSIVE); }
    }
    if (fsm_.policyTimedOut())
    { RCLCPP_WARN(get_logger(), "RL command timed out"); changeMode(PASSIVE); }
    wheel_leg_interfaces::msg::RobotCommand command;
    fsm_.run(command);
    // 最终覆盖：无反馈时全部 mode=0，不能被 Passive 的常规阻尼输出重新使能。
    if (!ready) command = wheel_leg_interfaces::msg::RobotCommand();
    io_.sendMotorCommand(command);
    if (ready) io_.publishState(fsm_.mode(), fsm_.session());
}
}
