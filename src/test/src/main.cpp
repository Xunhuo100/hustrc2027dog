#include "rclcpp/rclcpp.hpp"
#include "iostream"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Hello, ROS 2!");
    rclcpp::shutdown();
    return 0;
}