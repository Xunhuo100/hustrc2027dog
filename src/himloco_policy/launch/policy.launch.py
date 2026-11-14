import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue


def generate_launch_description():
    # 只启动策略；RobotState/RobotCommand 同时适用于仿真与真机。
    package = get_package_share_directory("himloco_policy")
    profile = LaunchConfiguration("policy_profile")
    profile_file = ParameterFile(PathJoinSubstitution([package, "config", [profile, ".yaml"]]),
                                 allow_substs=False)
    return LaunchDescription([
        DeclareLaunchArgument("policy_profile", default_value="parkour", choices=["parkour", "flat"]),
        DeclareLaunchArgument("model_path", default_value=PathJoinSubstitution(
            [package, "policies", [profile, ".pt"]])),
        DeclareLaunchArgument("velocity_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("autostart", default_value="false"),
        Node(package="himloco_policy", executable="himloco_policy", output="screen",
             parameters=[os.path.join(package, "config", "himloco.yaml"), profile_file,
                         {"policy_profile": profile, "model_path": LaunchConfiguration("model_path"),
                          "velocity_topic": LaunchConfiguration("velocity_topic"),
                          "autostart": ParameterValue(LaunchConfiguration("autostart"), value_type=bool)}]),
    ])
