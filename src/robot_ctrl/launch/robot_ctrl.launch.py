import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    package = get_package_share_directory("robot_ctrl")
    config = os.path.join(package, "config", "go2w.yaml")
    profile = ParameterFile(PathJoinSubstitution(
        [package, "config", [LaunchConfiguration("control_profile"), ".yaml"]]), allow_substs=False)
    defaults = {
        "motor_command_topic": "/lowcmd",
        "motor_state_topic": "/lowstate",
        "depth_topic": "/camera/depth/image_raw",
        "camera_info_topic": "/camera/depth/camera_info",
    }
    arguments = [DeclareLaunchArgument("control_profile", default_value="flat", choices=["flat", "parkour"])]
    for name, value in defaults.items():
        arguments.append(DeclareLaunchArgument(name, default_value=value))
    parameters = {name: LaunchConfiguration(name) for name in defaults}
    arguments.append(Node(package="robot_ctrl", executable="robot_ctrl", output="screen",
                          parameters=[config, profile, parameters]))
    return LaunchDescription(arguments)
