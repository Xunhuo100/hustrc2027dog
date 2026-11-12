import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile


def generate_launch_description():
    package = get_package_share_directory("himloco_sim2sim")
    controller = get_package_share_directory("robot_ctrl")
    profile = LaunchConfiguration("policy_profile")
    # 同一个 profile 同时覆盖控制器站姿、策略 PD/动作基准和仿真初始姿态。
    # parameters 列表中后面的配置覆盖前面的同名值；最终显式 topic 参数优先。
    profile_file = ParameterFile(PathJoinSubstitution([package, "config", [profile, ".yaml"]]),
                                 allow_substs=False)
    io_parameters = {
        "motor_command_topic": LaunchConfiguration("motor_command_topic"),
        "motor_state_topic": LaunchConfiguration("motor_state_topic"),
        "depth_topic": LaunchConfiguration("depth_topic"),
        "camera_info_topic": LaunchConfiguration("camera_info_topic"),
    }
    return LaunchDescription([
        DeclareLaunchArgument("policy_profile", default_value="parkour", choices=["parkour", "flat"],
                              description="Select matching weights, stand pose and PD gains"),
        DeclareLaunchArgument("model_path", default_value=PathJoinSubstitution(
            [package, "policies", [profile, ".pt"]])),
        DeclareLaunchArgument("model_xml", default_value=os.path.expanduser(
            "~/dogMujoco2027/unitree_robots/go2_w/urdf/scene_flat.xml")),
        # false 只关闭内置仿真桥；此 launch 仍会启动 HIM 策略，接外部策略应分开启动节点。
        DeclareLaunchArgument("start_simulator", default_value="true"),
        DeclareLaunchArgument("viewer", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("enable_keyboard", default_value="false"),
        DeclareLaunchArgument("keyboard_linear_speed", default_value="0.3"),
        DeclareLaunchArgument("motor_command_topic", default_value="/lowcmd"),
        DeclareLaunchArgument("motor_state_topic", default_value="/lowstate"),
        DeclareLaunchArgument("depth_topic", default_value="/camera/depth/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/depth/camera_info"),
        DeclareLaunchArgument("velocity_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("enable_depth", default_value="false"),
        Node(package="robot_ctrl", executable="robot_ctrl", output="screen",
             parameters=[os.path.join(controller, "config", "go2w.yaml"), profile_file, io_parameters]),
        Node(package="himloco_sim2sim", executable="go2w_mujoco", output="screen",
             condition=IfCondition(LaunchConfiguration("start_simulator")),
             parameters=[profile_file, io_parameters, {"model_xml": LaunchConfiguration("model_xml"),
                          "enable_depth": ParameterValue(LaunchConfiguration("enable_depth"), value_type=bool),
                          "viewer": ParameterValue(LaunchConfiguration("viewer"), value_type=bool)}]),
        Node(package="himloco_sim2sim", executable="himloco_policy", output="screen",
             parameters=[os.path.join(package, "config", "himloco.yaml"), profile_file,
                         {"policy_profile": profile, "model_path": LaunchConfiguration("model_path"),
                          "velocity_topic": LaunchConfiguration("velocity_topic"),
                          "autostart": ParameterValue(LaunchConfiguration("autostart"), value_type=bool)}]),
        Node(package="himloco_sim2sim", executable="keyboard_control", output="screen",
             parameters=[os.path.join(package, "config", "keyboard.yaml"),
                         {"velocity_topic": LaunchConfiguration("velocity_topic"),
                          "linear_speed": ParameterValue(
                              LaunchConfiguration("keyboard_linear_speed"), value_type=float),
                          "enable_keyboard": ParameterValue(
                             LaunchConfiguration("enable_keyboard"), value_type=bool)}]),
    ])
