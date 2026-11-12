#include "control/RobotController.h"
#include <exception>
#include <iostream>
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try
    {
        auto controller = std::make_shared<wheel_leg::RobotController>();
        rclcpp::spin(controller);
    }
    catch (const std::exception &error)
    {
        std::cerr << "robot_ctrl: " << error.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
