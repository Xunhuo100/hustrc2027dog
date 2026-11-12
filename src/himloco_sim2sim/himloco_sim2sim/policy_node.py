import os
import hashlib
import time
import numpy as np
import torch
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from wheel_leg_interfaces.msg import RobotState, RobotCommand
from .policy_core import DEFAULT_CONFIG, HimPolicyCore


class HimLocoPolicy(Node):
    # 策略层只发布统一 RobotCommand，不直接写 LowCmd；真机与仿真共用此 ROS2 边界。
    def __init__(self):
        super().__init__("himloco_policy")
        config = {}
        for name, default in DEFAULT_CONFIG.items():
            config[name] = self.declare_parameter(name, default).value
        self.core = HimPolicyCore(config)
        # profile 参数本身仅用于日志；实际 YAML/模型选择由 launch 完成。
        profile = self.declare_parameter("policy_profile", "flat").value
        default_model = os.path.join(get_package_share_directory("himloco_sim2sim"), "policies", "policy.pt")
        model_path = os.path.expanduser(self.declare_parameter("model_path", default_model).value)
        if not os.path.isfile(model_path):
            raise FileNotFoundError("Exported HIMLoco TorchScript model missing: " + model_path)
        torch.set_num_threads(1)
        self.model = torch.jit.load(model_path, map_location="cpu").eval()
        # 尽早检查 342 -> 16 接口；防止误用原始 checkpoint 或 12 电机模型。
        with torch.no_grad():
            result = self.model(torch.zeros(1, 342))
        if result.shape != (1, 16) or not torch.isfinite(result).all():
            raise ValueError("Policy must accept [1,342] and return finite [1,16]")
        self.autostart = self.declare_parameter("autostart", False).value
        self.stand_wait = self.declare_parameter("stand_wait", 1.7).value
        self.state_timeout = self.declare_parameter("state_timeout", 0.1).value
        self.command_timeout = self.declare_parameter("command_timeout", 0.5).value
        self.initial_command = np.asarray(self.declare_parameter("initial_command", [0.0, 0.0, 0.0]).value)
        if self.initial_command.shape != (3,) or not np.isfinite(self.initial_command).all():
            raise ValueError("initial_command must have three finite values")
        self.command = self.initial_command.copy()
        self.command_time = time.monotonic()
        self.state = None
        self.state_time = 0.0
        self.session = None
        self.auto_phase = 0
        self.stand_time = 0.0
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1)
        self.command_pub = self.create_publisher(RobotCommand, "/robot/rl_cmd", qos)
        self.mode_pub = self.create_publisher(String, "/robot/mode", qos)
        self.state_sub = self.create_subscription(RobotState, "/robot/state", self.state_callback, qos)
        velocity_topic = self.declare_parameter("velocity_topic", "/cmd_vel").value
        self.velocity_sub = self.create_subscription(Twist, velocity_topic, self.velocity_callback, qos)
        self.timer = self.create_timer(config["policy_period"], self.update)
        self.get_logger().info("HIMLoco loaded: 57 x 6 -> 16, " + model_path)
        with open(model_path, "rb") as policy_file:
            fingerprint = hashlib.sha256(policy_file.read()).hexdigest()[:16]
        self.get_logger().info(
            "Profile=%s, sha256=%s, leg PD=%.1f/%.2f, wheel kd=%.2f, gyro scale=%.2f" %
            (profile, fingerprint, config["leg_kp"], config["leg_kd"],
             config["wheel_kd"], config["angular_velocity_scale"]))

    def state_callback(self, message):
        # 接收时刻用于断流检查；update() 还会检查保留在消息中的反馈时间戳。
        self.state = message
        self.state_time = time.monotonic()

    def velocity_callback(self, message):
        # 约定机体坐标系 vx/vy/wz；Twist 无 frame_id，此处不做坐标变换或速度限幅。
        command = np.asarray([message.linear.x, message.linear.y, message.angular.z])
        if np.isfinite(command).all():
            self.command = command
            self.command_time = time.monotonic()

    def set_mode(self, mode):
        message = String()
        message.data = mode
        self.mode_pub.publish(message)

    def update(self):
        current = time.monotonic()
        if self.state is None or current - self.state_time > self.state_timeout:
            return
        state = self.state
        stamp = state.header.stamp.sec + state.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        if age < -0.05 or age > self.state_timeout:
            return
        # 自动启动只执行一次；故障退出后交还操作者。
        if self.autostart and self.auto_phase == 0:
            self.set_mode("stand")
            self.auto_phase = 1
        if self.autostart and self.auto_phase == 1 and state.control_mode == RobotState.STAND:
            self.stand_time = current
            self.auto_phase = 2
        if self.autostart and self.auto_phase == 2 and current - self.stand_time >= self.stand_wait:
            self.set_mode("rl")
            self.auto_phase = 3
        # 模式/会话变化后重置观测记忆；外部 RNN 策略也必须重置自己的 hidden state。
        session = (state.control_mode, state.control_session)
        if session != self.session:
            self.core.reset()
            self.session = session
        if state.control_mode != RobotState.RL:
            return
        # 速度命令断流只将目标速度归零，机器人仍在 RL；状态/策略断流则由控制器退出 RL。
        command = self.command if current - self.command_time < self.command_timeout else np.zeros(3)
        q = state.imu.orientation
        omega = state.imu.angular_velocity
        try:
            # 当前是纯本体感知策略：只使用 motor_state、姿态、陀螺仪和速度命令。
            # 没有图像订阅。接视觉模型需增加预处理/同步，并替换下面的模型调用契约。
            history = self.core.observe(
                [m.q for m in state.motor_state], [m.dq for m in state.motor_state],
                [q.w, q.x, q.y, q.z], [omega.x, omega.y, omega.z], command)
            with torch.no_grad():
                action = self.model(torch.from_numpy(history)).cpu().numpy()[0]
            position, velocity, kp, kd = self.core.action_to_targets(action)
            # 推理过慢时不把旧状态包装成一条新指令。
            if time.monotonic() - self.state_time > self.state_timeout:
                return
            output = RobotCommand()
            output.header.stamp = self.get_clock().now().to_msg()
            # 必须回传当前反馈中的会话号；不要用本地计数器代替。
            output.control_session = state.control_session
            for i in range(16):
                motor = output.motor_cmd[i]
                motor.mode = 1
                motor.q, motor.dq = float(position[i]), float(velocity[i])
                motor.kp, motor.kd = float(kp[i]), float(kd[i])
                motor.tau = 0.0
            self.command_pub.publish(output)
        except (ValueError, RuntimeError) as error:
            self.get_logger().error("Policy stopped: " + str(error))
            self.set_mode("passive")
            self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HimLocoPolicy()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
