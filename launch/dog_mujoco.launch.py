import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    # 与 sim2sim 相同的组合，底层 topic 默认放在 /sim 下。
    simulator = get_package_share_directory("wheel_leg_mujoco")
    policy = get_package_share_directory("himloco_policy")
    defaults = {
        "model_xml": os.path.join(simulator, "models", "go2w", "urdf", "scene_flat.xml"),
        "policy_profile": "parkour",
        "model_path": PathJoinSubstitution([policy, "policies", [LaunchConfiguration("policy_profile"), ".pt"]]),
        "motor_command_topic": "/sim/lowcmd",
        "motor_state_topic": "/sim/lowstate",
        "depth_topic": "/sim/camera/depth/image_raw",
        "camera_info_topic": "/sim/camera/depth/camera_info",
        "velocity_topic": "/cmd_vel",
        "start_simulator": "true",
        "start_policy": "true",
        "viewer": "false",
        "follow_camera": "true",
        "camera_distance": "3.0",
        "enable_depth": "false",
        "autostart": "true",
        "enable_keyboard": "false",
        "keyboard_linear_speed": "0.3",
    }
    arguments = [DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()]
    arguments.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(os.path.dirname(__file__), "sim2sim.launch.py")),
        launch_arguments={name: LaunchConfiguration(name) for name in defaults}.items()))
    return LaunchDescription(arguments)
