import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # 只启动仿真驱动；不加载策略、不依赖 robot_ctrl 或 Torch。
    package = get_package_share_directory("wheel_leg_mujoco")
    defaults = {
        "config_file": os.path.join(package, "config", "flat.yaml"),
        "model_xml": os.path.join(package, "models", "go2w", "urdf", "scene_flat.xml"),
        "motor_command_topic": "/sim/lowcmd",
        "motor_state_topic": "/sim/lowstate",
        "depth_topic": "/sim/camera/depth/image_raw",
        "camera_info_topic": "/sim/camera/depth/camera_info",
        "viewer": "false",
        "follow_camera": "true",
        "camera_distance": "3.0",
        "enable_depth": "false",
    }
    arguments = [DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()]
    parameters = {name: LaunchConfiguration(name) for name in [
        "model_xml", "motor_command_topic", "motor_state_topic", "depth_topic", "camera_info_topic"]}
    for name in ["viewer", "follow_camera", "enable_depth"]:
        parameters[name] = ParameterValue(LaunchConfiguration(name), value_type=bool)
    parameters["camera_distance"] = ParameterValue(LaunchConfiguration("camera_distance"), value_type=float)
    arguments.append(Node(package="wheel_leg_mujoco", executable="go2w_mujoco", output="screen",
                          parameters=[LaunchConfiguration("config_file"), parameters]))
    return LaunchDescription(arguments)
