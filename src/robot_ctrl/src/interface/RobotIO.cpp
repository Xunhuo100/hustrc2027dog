#include "interface/RobotIO.h"
#include "interface/motor_crc.h"
#include "common/RobotConfig.h"
#include <cmath>
#include <functional>
#include <set>
#include <stdexcept>
namespace wheel_leg
{
RobotIO::RobotIO(rclcpp::Node *node) : node_(node)
{
    using std::placeholders::_1;
    // mapping[i] 是统一第 i 个电机对应的底层 20 槽位索引，收发使用同一张表。
    // 仅重排槽位，不补偿关节方向、零点或减速比；这些转换应在驱动中完成。
    motor_mapping_ = node_->declare_parameter<std::vector<int64_t>>(
        "motor_mapping", {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15});
    std::set<int64_t> unique(motor_mapping_.begin(), motor_mapping_.end());
    if (motor_mapping_.size() != MOTOR_COUNT || unique.size() != MOTOR_COUNT ||
        *unique.begin() < 0 || *unique.rbegin() >= 20)
        throw std::runtime_error("motor_mapping needs 16 unique indices in [0, 19]");
    auto command_topic = node_->declare_parameter<std::string>("motor_command_topic", "/lowcmd");
    auto feedback_topic = node_->declare_parameter<std::string>("motor_state_topic", "/lowstate");
    auto depth_topic = node_->declare_parameter<std::string>("depth_topic", "/camera/depth/image_raw");
    auto info_topic = node_->declare_parameter<std::string>("camera_info_topic", "/camera/depth/camera_info");
    motor_pub_ = node_->create_publisher<unitree_go::msg::LowCmd>(command_topic, 1);
    state_pub_ = node_->create_publisher<wheel_leg_interfaces::msg::RobotState>("/robot/state", 1);
    imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("/robot/imu", rclcpp::SensorDataQoS());
    motor_sub_ = node_->create_subscription<unitree_go::msg::LowState>(feedback_topic,
        rclcpp::SensorDataQoS().keep_last(1), std::bind(&RobotIO::motorStateCallback, this, _1));
    depth_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(depth_topic,
        rclcpp::SensorDataQoS().keep_last(1), std::bind(&RobotIO::depthCallback, this, _1));
    camera_info_sub_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>(info_topic,
        rclcpp::SensorDataQoS(), std::bind(&RobotIO::cameraInfoCallback, this, _1));
    RCLCPP_INFO(node_->get_logger(), "ROS2 motor topics: publish %s, subscribe %s",
        command_topic.c_str(), feedback_topic.c_str());
}
void RobotIO::motorStateCallback(unitree_go::msg::LowState::ConstSharedPtr message)
{
    // 任意一个必要字段非法就使本次反馈不可用，控制循环会转入 Passive。
    received_ = false;
    double norm = 0.0;
    for (float value : message->imu_state.quaternion)
    {
        if (!std::isfinite(value)) return;
        norm += value * value;
    }
    if (norm < 0.25 || norm > 2.25) return;
    for (int i = 0; i < 3; i++)
        if (!std::isfinite(message->imu_state.gyroscope[i]) ||
            !std::isfinite(message->imu_state.accelerometer[i])) return;
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        const auto &motor = message->motor_state[motor_mapping_[i]];
        if (!std::isfinite(motor.q) || !std::isfinite(motor.dq) ||
            !std::isfinite(motor.tau_est)) return;
    }
    for (int i = 0; i < MOTOR_COUNT; i++)
        state_.motor_state[i] = message->motor_state[motor_mapping_[i]];
    // LowState 没有采样时间戳，此处记录接收时间；重复转发不会刷新这个时间。
    state_.header.stamp = node_->now();
    state_.header.frame_id = "base";
    state_.imu.header = state_.header;
    norm = std::sqrt(norm);
    // Unitree 数组为 [w,x,y,z]，ROS Imu 为具名 x/y/z/w 字段。
    // 此处只归一化/换排列，不旋转安装角，不执行 deg/s -> rad/s 转换或滤波。
    // 驱动必须提供与训练一致的机体坐标系姿态、rad/s 角速度和 m/s² 加速度。
    state_.imu.orientation.w = message->imu_state.quaternion[0] / norm;
    state_.imu.orientation.x = message->imu_state.quaternion[1] / norm;
    state_.imu.orientation.y = message->imu_state.quaternion[2] / norm;
    state_.imu.orientation.z = message->imu_state.quaternion[3] / norm;
    state_.imu.angular_velocity.x = message->imu_state.gyroscope[0];
    state_.imu.angular_velocity.y = message->imu_state.gyroscope[1];
    state_.imu.angular_velocity.z = message->imu_state.gyroscope[2];
    state_.imu.linear_acceleration.x = message->imu_state.accelerometer[0];
    state_.imu.linear_acceleration.y = message->imu_state.accelerometer[1];
    state_.imu.linear_acceleration.z = message->imu_state.accelerometer[2];
    state_time_ = std::chrono::steady_clock::now();
    received_ = true;
}
bool RobotIO::stateReady(double timeout) const
{
    return received_ && std::chrono::duration<double>(
        std::chrono::steady_clock::now() - state_time_).count() < timeout;
}
const wheel_leg_interfaces::msg::RobotState &RobotIO::getState() const { return state_; }
void RobotIO::sendMotorCommand(const wheel_leg_interfaces::msg::RobotCommand &command)
{
    // 本框架的底层输出入口：统一 16 电机指令 -> 映射后的 LowCmd -> CRC -> ROS2。
    // 本函数不计算 PD 力矩；仿真由 MujocoRobot.step() 计算，真机由兼容驱动/电机执行。
    unitree_go::msg::LowCmd output;
    output.head = {{0xFE, 0xEF}};
    output.level_flag = unitree_go_crc::LOWLEVEL;
    output.bms_cmd.off = 0xA5;
    for (int i = 0; i < MOTOR_COUNT; i++) output.motor_cmd[motor_mapping_[i]] = command.motor_cmd[i];
    unitree_go_crc::get_crc(output);
    motor_pub_->publish(output);  // 电机指令唯一输出入口。
}
void RobotIO::publishState(int mode, uint32_t session)
{
    // control_mode/session 随控制周期更新，反馈 stamp 保留最近有效 LowState 的接收时间。
    state_.control_mode = mode;
    state_.control_session = session;
    state_pub_->publish(state_);
    imu_pub_->publish(state_.imu);
}
// 相机入口仅缓存最近一帧及内参，不做时效/同步/编码处理，也不送进 HIMLoco。
// 外部视觉策略建议直接订阅配置中的相机 topic，并自己校验时间与预处理。
void RobotIO::depthCallback(sensor_msgs::msg::Image::ConstSharedPtr message) { depth_image_ = message; }
void RobotIO::cameraInfoCallback(sensor_msgs::msg::CameraInfo::ConstSharedPtr message) { camera_info_ = message; }
sensor_msgs::msg::Image::ConstSharedPtr RobotIO::getDepthImage() const { return depth_image_; }
sensor_msgs::msg::CameraInfo::ConstSharedPtr RobotIO::getDepthCameraInfo() const { return camera_info_; }
}
