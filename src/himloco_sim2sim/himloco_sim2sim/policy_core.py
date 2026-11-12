"""对照 HIMLoco/him_go2w_env.py；此文件不依赖 ROS 或 Torch。"""
import numpy as np

JOINT_NAMES = [leg + "_" + joint + "_joint"
               for leg in ["FL", "FR", "RL", "RR"]
               for joint in ["hip", "thigh", "calf", "foot"]]
DEFAULT_POSITION = [0.1, 0.8, -1.5, 0.0, -0.1, 0.8, -1.5, 0.0,
                    0.1, 1.0, -1.5, 0.0, -0.1, 1.0, -1.5, 0.0]
DEFAULT_CONFIG = {
    "default_position": DEFAULT_POSITION,
    "history_length": 6, "policy_period": 0.02,
    "command_scale": [2.0, 2.0, 0.25], "angular_velocity_scale": 0.25,
    "joint_position_scale": 1.0, "joint_velocity_scale": 0.05,
    "clip_observations": 100.0, "clip_actions": 100.0,
    "leg_action_scale": 0.25, "wheel_action_scale": 10.0,
    "leg_kp": 20.0, "leg_kd": 0.5, "wheel_kd": 0.3,
    "wheel_velocity_limit": 100.0,
}


def finite_vector(value, size):
    value = np.asarray(value, dtype=np.float32)
    if value.shape != (size,) or not np.isfinite(value).all():
        raise ValueError("Expected finite vector of size " + str(size))
    return value


def projected_gravity(quaternion):
    # 输入是机体相对世界的 [w,x,y,z]；将世界重力单位向量 [0,0,-1] 旋到机体。
    # 水平静止应为 [0,0,-1]；不使用欧拉角，也不使用加速度计估计重力。
    w, x, y, z = finite_vector(quaternion, 4)
    norm = float(np.linalg.norm([w, x, y, z]))
    if norm < 0.5 or norm > 1.5:
        raise ValueError("Invalid IMU quaternion norm")
    w, x, y, z = np.asarray([w, x, y, z]) / norm
    return np.asarray([2 * (w * y - x * z), -2 * (w * x + y * z),
                       2 * (x * x + y * y) - 1], dtype=np.float32)


class HimPolicyCore:
    # HIM Go2W 专用适配器：关节顺序、57 维单帧、6 帧历史和混合动作语义均已固定。
    # 外部策略若观测/动作不同，应替换对应逻辑，不能只替换 model_path。
    def __init__(self, config=None):
        self.config = DEFAULT_CONFIG.copy()
        if config:
            self.config.update(config)
        self.default_position = finite_vector(self.config["default_position"], 16)
        self.command_scale = finite_vector(self.config["command_scale"], 3)
        self.history_length = int(self.config["history_length"])
        if self.history_length != 6:
            raise ValueError("This Go2W HIM policy requires six 57-dimensional frames")
        for key in ["policy_period", "clip_observations", "clip_actions",
                    "leg_action_scale", "wheel_action_scale", "wheel_velocity_limit"]:
            if not np.isfinite(self.config[key]) or self.config[key] <= 0:
                raise ValueError("Invalid parameter: " + key)
        self.history = np.zeros((self.history_length, 57), dtype=np.float32)
        self.last_action = np.zeros(16, dtype=np.float32)

    def reset(self):
        # 每次 control_mode/session 变化清空历史和上一动作；循环网络还需清空其隐藏状态。
        self.history.fill(0)
        self.last_action.fill(0)

    def observe(self, position, velocity, quaternion, angular_velocity, command):
        position = finite_vector(position, 16)
        velocity = finite_vector(velocity, 16)
        angular_velocity = finite_vector(angular_velocity, 3)
        command = finite_vector(command, 3)
        relative_position = (position - self.default_position) * self.config["joint_position_scale"]
        relative_position[3::4] = 0.0  # 轮角累计值不进入策略。
        # 57 维分段: [0:3]速度命令、[3:6]角速度、[6:9]重力、
        # [9:25]关节位置差、[25:41]关节速度、[41:57]上一次裁剪后的原始动作。
        frame = np.concatenate([
            command * self.command_scale,
            angular_velocity * self.config["angular_velocity_scale"],
            projected_gravity(quaternion), relative_position,
            velocity * self.config["joint_velocity_scale"], self.last_action,
        ])
        bound = self.config["clip_observations"]
        frame = np.clip(frame, -bound, bound)
        # 训练端把最新一帧放在最前；首次进入 RL 时旧帧为零。
        self.history[1:] = self.history[:-1].copy()
        self.history[0] = frame
        return self.history.reshape(1, -1).copy()

    def action_to_targets(self, action):
        # last_action 对齐训练：保存按 clip_actions 裁剪的网络输出，
        # 不用关节限位之后的目标反算；腿输出位置偏移，轮输出目标角速度。
        action = finite_vector(action, 16)
        bound = self.config["clip_actions"]
        self.last_action = np.clip(action, -bound, bound)
        position = self.default_position + self.last_action * self.config["leg_action_scale"]
        # 电机目标限制采用当前 Go2W XML 的关节范围。
        # 这些范围硬编码于当前 Go2W；换机器人需要同步修改，motor_mapping 不会改限位。
        lower = np.asarray([-1.0472, -1.570796, -2.7227, 0.0] * 4)
        lower[[9, 13]] = -0.5236
        upper = np.asarray([1.0472, 3.4907, -0.83776, 0.0] * 4)
        upper[[9, 13]] = 4.5379
        position = np.clip(position, lower, upper)
        position[3::4] = 0.0
        velocity = np.zeros(16, dtype=np.float32)
        limit = self.config["wheel_velocity_limit"]
        velocity[3::4] = np.clip(self.last_action[3::4] * self.config["wheel_action_scale"], -limit, limit)
        kp = np.full(16, self.config["leg_kp"], dtype=np.float32)
        kd = np.full(16, self.config["leg_kd"], dtype=np.float32)
        kp[3::4] = 0.0
        kd[3::4] = self.config["wheel_kd"]
        return position, velocity, kp, kd
