import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory("robot_ctrl"), "config", "go2w.yaml")
    defaults = {
        "motor_command_topic": "/lowcmd",
        "motor_state_topic": "/lowstate",
        "depth_topic": "/camera/depth/image_raw",
        "camera_info_topic": "/camera/depth/camera_info",
    }
    arguments = []
    for name, value in defaults.items():
        arguments.append(DeclareLaunchArgument(name, default_value=value))
    parameters = {name: LaunchConfiguration(name) for name in defaults}
    arguments.append(Node(package="robot_ctrl", executable="robot_ctrl", output="screen",
                          parameters=[config, parameters]))
    return LaunchDescription(arguments)
