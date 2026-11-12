#ifndef WHEEL_LEG_ROBOT_IO_H
#define WHEEL_LEG_ROBOT_IO_H
#include <chrono>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <unitree_go/msg/low_cmd.hpp>
#include <unitree_go/msg/low_state.hpp>
#include <wheel_leg_interfaces/msg/robot_state.hpp>
#include <wheel_leg_interfaces/msg/robot_command.hpp>
namespace wheel_leg
{
// 单线程 spin；底层协议、IMU 和相机入口集中在这里。
class RobotIO
{
public:
    explicit RobotIO(rclcpp::Node *node);
    bool stateReady(double timeout) const;
    const wheel_leg_interfaces::msg::RobotState &getState() const;
    // C++ 进程内的原始相机缓存，未接收时为空；Python 策略通过 ROS2 topic 获取。
    sensor_msgs::msg::Image::ConstSharedPtr getDepthImage() const;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr getDepthCameraInfo() const;
    void sendMotorCommand(const wheel_leg_interfaces::msg::RobotCommand &command);
    void publishState(int mode, uint32_t session);
private:
    void motorStateCallback(unitree_go::msg::LowState::ConstSharedPtr message);
    void depthCallback(sensor_msgs::msg::Image::ConstSharedPtr message);
    void cameraInfoCallback(sensor_msgs::msg::CameraInfo::ConstSharedPtr message);
    rclcpp::Node *node_;
    std::vector<int64_t> motor_mapping_;
    bool received_ = false;
    std::chrono::steady_clock::time_point state_time_;
    wheel_leg_interfaces::msg::RobotState state_;
    sensor_msgs::msg::Image::ConstSharedPtr depth_image_;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_;
    rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr motor_pub_;
    rclcpp::Publisher<wheel_leg_interfaces::msg::RobotState>::SharedPtr state_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr motor_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
};
}
#endif
