#include "control/RobotController.h"
#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
namespace wheel_leg
{
RobotController::RobotController() : Node("robot_ctrl"), io_(this)
{
    using std::placeholders::_1;
    // 控制层默认 200 Hz；策略默认 50 Hz，两次策略输出之间复用最近一条有效指令。
    // 此处 PD 用于 Stand/等待首条 RL 指令；正常 RL 的 PD 来自 RobotCommand。
    double period = declare_parameter("control_period", 0.005);
    state_timeout_ = declare_parameter("state_timeout", 0.1);
    policy_timeout_ = declare_parameter("policy_timeout", 0.2);
    stand_duration_ = declare_parameter("stand_duration", 1.5);
    leg_kp_ = declare_parameter("leg_kp", 20.0);
    leg_kd_ = declare_parameter("leg_kd", 0.5);
    wheel_kd_ = declare_parameter("wheel_kd", 0.3);
    default_position_ = declare_parameter<std::vector<double>>("default_position",
        std::vector<double>(DEFAULT_POSITION.begin(), DEFAULT_POSITION.end()));
    if (!std::isfinite(period) || period <= 0 || !std::isfinite(stand_duration_) ||
        stand_duration_ <= 0 || !std::isfinite(state_timeout_) || state_timeout_ <= 0 ||
        !std::isfinite(policy_timeout_) || policy_timeout_ <= 0 ||
        !std::isfinite(leg_kp_) || leg_kp_ < 0 || leg_kp_ > 100 ||
        !std::isfinite(leg_kd_) || leg_kd_ < 0 || leg_kd_ > 10 ||
        !std::isfinite(wheel_kd_) || wheel_kd_ < 0 || wheel_kd_ > 10 ||
        default_position_.size() != MOTOR_COUNT)
        throw std::runtime_error("Invalid controller periods, gains or default_position");
    for (double q : default_position_)
        if (!std::isfinite(q) || std::abs(q) > 5) throw std::runtime_error("Invalid default_position");
    mode_sub_ = create_subscription<std_msgs::msg::String>("/robot/mode", 1,
        std::bind(&RobotController::modeCallback, this, _1));
    policy_sub_ = create_subscription<wheel_leg_interfaces::msg::RobotCommand>("/robot/rl_cmd", 1,
        std::bind(&RobotController::policyCallback, this, _1));
    timer_ = create_wall_timer(std::chrono::duration<double>(period), std::bind(&RobotController::update, this));
    RCLCPP_INFO(get_logger(), "16 DOF ready: passive -> stand -> rl");
}
void RobotController::changeMode(int mode)
{
    if (mode_ == mode) return;
    mode_ = mode;
    // 每次真正切换模式都更换会话号；上一次 RL 留在 ROS 队列中的指令会被拒收。
    session_++;
    if (mode == STAND)
    {
        stand_time_ = std::chrono::steady_clock::now();
        for (int i = 0; i < MOTOR_COUNT; i++) stand_start_[i] = io_.getState().motor_state[i].q;
    }
    if (mode == RL) rl_.enter(session_);
    RCLCPP_INFO(get_logger(), "Control mode: %d, session: %u", mode_, session_);
}
void RobotController::modeCallback(std_msgs::msg::String::ConstSharedPtr message)
{
    // 必须先完成 Stand 才能进入 RL，外部策略也遵守同一启动顺序。
    if (message->data == "passive") { changeMode(PASSIVE); return; }
    if (!io_.stateReady(state_timeout_)) { RCLCPP_WARN(get_logger(), "No fresh motor/IMU feedback"); return; }
    if (message->data == "stand") changeMode(STAND);
    else if (message->data == "rl" && mode_ == STAND &&
        std::chrono::duration<double>(std::chrono::steady_clock::now() - stand_time_).count() >= stand_duration_)
        changeMode(RL);
    else RCLCPP_WARN(get_logger(), "Use passive/stand/rl; RL requires a completed stand");
}
void RobotController::policyCallback(wheel_leg_interfaces::msg::RobotCommand::ConstSharedPtr message)
{
    if (mode_ != RL) return;
    // header.stamp 检查消息本身的年龄；State_RL 另用单调时钟检测后续断流。
    // 本节点与策略节点需要一致的 ROS 时钟；允许最多 50 ms 的未来时间偏差。
    if (message->header.stamp.sec < 0 || message->header.stamp.nanosec >= 1000000000)
    {
        RCLCPP_WARN(get_logger(), "Rejected invalid RL timestamp");
        changeMode(PASSIVE);
        return;
    }
    double age = (now() - rclcpp::Time(message->header.stamp)).seconds();
    if (age < -0.05 || age > policy_timeout_ || !rl_.receive(*message))
    {
        RCLCPP_WARN(get_logger(), "Rejected stale, invalid or wrong-session RL command");
        changeMode(PASSIVE);
    }
}
void RobotController::setStandCommand(wheel_leg_interfaces::msg::RobotCommand &command)
{
    // 腿从反馈位置线性插值到默认站姿；轮保持 kp=0、dq=0，仅施加速度阻尼。
    double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - stand_time_).count();
    double ratio = std::min(1.0, elapsed / stand_duration_);
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        auto &motor = command.motor_cmd[i];
        motor.mode = 1;
        motor.kd = isWheel(i) ? wheel_kd_ : leg_kd_;
        if (!isWheel(i))
        {
            motor.kp = leg_kp_;
            motor.q = stand_start_[i] + ratio * (default_position_[i] - stand_start_[i]);
        }
    }
}
void RobotController::update()
{
    bool ready = io_.stateReady(state_timeout_);
    if (!ready) changeMode(PASSIVE);
    if (ready && mode_ != PASSIVE)
    {
        const auto &q = io_.getState().imu.orientation;
        // 机体 z 轴与世界 z 轴的夹角余弦；阈值 0.5 对应总倾角 60°。
        if (1.0 - 2.0 * (q.x * q.x + q.y * q.y) < 0.5)
        { RCLCPP_WARN(get_logger(), "Tilt exceeded 60 degrees"); changeMode(PASSIVE); }
    }
    if (mode_ == RL && rl_.timedOut(policy_timeout_))
    { RCLCPP_WARN(get_logger(), "RL command timed out"); changeMode(PASSIVE); }
    wheel_leg_interfaces::msg::RobotCommand command;
    // 有反馈的 Passive 是阻尼状态；反馈失效时 mode=0，关闭电机输出。
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        command.motor_cmd[i].mode = ready ? 1 : 0;
        command.motor_cmd[i].kd = ready ? (isWheel(i) ? wheel_kd_ : 1.0) : 0.0;
    }
    // 刚进入 RL 时先保持站姿，最多等待 policy_timeout；随后直接使用策略的整条指令。
    if (mode_ == STAND || (mode_ == RL && !rl_.hasCommand())) setStandCommand(command);
    else if (mode_ == RL) command = rl_.command();
    io_.sendMotorCommand(command);
    if (ready) io_.publishState(mode_, session_);
}
}
