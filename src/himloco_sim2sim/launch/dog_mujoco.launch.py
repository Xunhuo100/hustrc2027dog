import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    # 复用本工作空间的 ROS2 MuJoCo 桥和参考项目的 XML，并非启动参考项目的原生 DDS 程序。
    package = get_package_share_directory("himloco_sim2sim")
    defaults = {
        "model_xml": os.path.expanduser("~/dogMujoco2027/unitree_robots/go2_w/urdf/scene_flat.xml"),
        "policy_profile": "parkour",
        "model_path": PathJoinSubstitution([package, "policies", [LaunchConfiguration("policy_profile"), ".pt"]]),
        "motor_command_topic": "/sim/lowcmd",
        "motor_state_topic": "/sim/lowstate",
        "depth_topic": "/sim/camera/depth/image_raw",
        "camera_info_topic": "/sim/camera/depth/camera_info",
        "velocity_topic": "/cmd_vel",
        "start_simulator": "true",
        "viewer": "false",
        "follow_camera": "true",
        "camera_distance": "3.0",
        "enable_depth": "false",
        "autostart": "true",
        "enable_keyboard": "false",
        "keyboard_linear_speed": "0.3",
    }
    arguments = []
    for name, value in defaults.items():
        arguments.append(DeclareLaunchArgument(name, default_value=value))
    arguments.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package, "launch", "sim2sim.launch.py")),
        launch_arguments={name: LaunchConfiguration(name) for name in defaults}.items()))
    return LaunchDescription(arguments)
