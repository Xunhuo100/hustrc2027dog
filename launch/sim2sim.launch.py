import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue


def generate_launch_description():
    # 工作空间级组合启动；两个功能包各自也能独立运行。
    simulator = get_package_share_directory("wheel_leg_mujoco")
    policy = get_package_share_directory("himloco_policy")
    controller = get_package_share_directory("robot_ctrl")
    profile = LaunchConfiguration("policy_profile")
    controller_profile = ParameterFile(PathJoinSubstitution([controller, "config", [profile, ".yaml"]]),
                                       allow_substs=False)
    io_names = ["motor_command_topic", "motor_state_topic", "depth_topic", "camera_info_topic"]
    io_parameters = {name: LaunchConfiguration(name) for name in io_names}
    defaults = {
        "policy_profile": "parkour",
        "model_path": PathJoinSubstitution([policy, "policies", [profile, ".pt"]]),
        "model_xml": os.path.join(simulator, "models", "go2w", "urdf", "scene_flat.xml"),
        "start_simulator": "true",
        "start_policy": "true",
        "viewer": "false",
        "follow_camera": "true",
        "camera_distance": "3.0",
        "autostart": "true",
        "enable_keyboard": "false",
        "keyboard_linear_speed": "0.3",
        "motor_command_topic": "/lowcmd",
        "motor_state_topic": "/lowstate",
        "depth_topic": "/camera/depth/image_raw",
        "camera_info_topic": "/camera/depth/camera_info",
        "velocity_topic": "/cmd_vel",
        "enable_depth": "false",
    }
    arguments = []
    for name, value in defaults.items():
        arguments.append(DeclareLaunchArgument(name, default_value=value,
                         choices=["parkour", "flat"] if name == "policy_profile" else None))
    arguments.append(Node(package="robot_ctrl", executable="robot_ctrl", output="screen",
                          parameters=[os.path.join(controller, "config", "go2w.yaml"),
                                      controller_profile, io_parameters]))
    sim_arguments = {name: LaunchConfiguration(name) for name in io_names + [
        "model_xml", "viewer", "follow_camera", "camera_distance", "enable_depth"]}
    # 同一 profile 同步选择仿真初始姿态、控制器站姿与策略动作基准。
    sim_arguments["config_file"] = PathJoinSubstitution([simulator, "config", [profile, ".yaml"]])
    arguments.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(simulator, "launch", "mujoco.launch.py")),
        condition=IfCondition(LaunchConfiguration("start_simulator")),
        launch_arguments=sim_arguments.items()))
    arguments.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(policy, "launch", "policy.launch.py")),
        condition=IfCondition(LaunchConfiguration("start_policy")),
        launch_arguments={name: LaunchConfiguration(name) for name in [
            "policy_profile", "model_path", "velocity_topic", "autostart"]}.items()))
    arguments.append(Node(package="robot_ctrl", executable="keyboard_control", output="screen",
                          parameters=[os.path.join(controller, "config", "keyboard.yaml"),
                                      {"velocity_topic": LaunchConfiguration("velocity_topic"),
                                       "linear_speed": ParameterValue(
                                           LaunchConfiguration("keyboard_linear_speed"), value_type=float),
                                       "enable_keyboard": ParameterValue(
                                           LaunchConfiguration("enable_keyboard"), value_type=bool)}]))
    return LaunchDescription(arguments)
