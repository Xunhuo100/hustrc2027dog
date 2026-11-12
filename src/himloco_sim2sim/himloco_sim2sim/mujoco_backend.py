"""按关节名映射 MuJoCo 索引，可脱离 ROS 运行和检查。"""
import os
import numpy as np
import mujoco
from .policy_core import JOINT_NAMES, DEFAULT_POSITION, finite_vector


class MujocoRobot:
    # 仿真后端不依赖策略：任何满足 16 路 mode/q/dq/kp/kd/tau 约定的指令都能执行。
    # 当前机器人结构仍固定为 Go2W，并要求名为 base 的机体和相同的 16 个关节名。
    def __init__(self, model_path, timestep=0.005, default_position=None, initial_height=None):
        if not os.path.isfile(model_path):
            raise FileNotFoundError("MuJoCo XML missing: " + model_path)
        if not np.isfinite(timestep) or timestep <= 0:
            raise ValueError("simulation_period must be positive")
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.model.opt.timestep = timestep
        if self.model.nu != 16:
            raise ValueError("Expected a 16 actuator Go2W model")
        self.data = mujoco.MjData(self.model)
        self.joint_ids, self.actuator_ids = [], []
        # 通过名称寻找 qpos/qvel/actuator 地址，不假设 MJCF 的数组排列等于策略顺序。
        for name in JOINT_NAMES:
            joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint < 0:
                raise ValueError("Missing joint: " + name)
            matches = np.flatnonzero(self.model.actuator_trnid[:, 0] == joint)
            if len(matches) != 1:
                raise ValueError("Joint needs one torque actuator: " + name)
            self.joint_ids.append(joint)
            self.actuator_ids.append(int(matches[0]))
        self.qpos_ids = self.model.jnt_qposadr[self.joint_ids]
        self.qvel_ids = self.model.jnt_dofadr[self.joint_ids]
        self.base_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "base")
        if self.base_id < 0:
            raise ValueError("Model needs a base body")
        if self.model.nkey > 0:
            mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        position = DEFAULT_POSITION if default_position is None else default_position
        self.data.qpos[self.qpos_ids] = finite_vector(position, 16)
        if initial_height is not None:
            if not np.isfinite(initial_height) or initial_height <= 0:
                raise ValueError("initial_height must be positive")
            self.data.qpos[2] = initial_height
        mujoco.mj_forward(self.model, self.data)

    def step(self, command):
        # command 列: mode, q, dq, kp, kd, tau。无命令时只保留阻尼。
        position, velocity = self.joint_state()[:2]
        if command is None:
            torque = -velocity.copy()
            torque[3::4] *= 0.3
        else:
            command = np.asarray(command, dtype=float)
            if command.shape != (16, 6) or not np.isfinite(command).all():
                raise ValueError("Invalid 16 motor command")
            # 腿: kp*(q_des-q)-kd*dq；轮: kp=0，kd*(dq_des-dq)。tau 是额外前馈力矩。
            # 若换成非单位 gear 的执行器，需要重新检查 ctrl 与关节力矩的对应关系。
            torque = command[:, 3] * (command[:, 1] - position)
            torque += command[:, 4] * (command[:, 2] - velocity) + command[:, 5]
            torque[command[:, 0] == 0] = 0.0
        # 先在 Python 算 PD，再按 XML 的 actuator ctrlrange 限制，最后执行一个物理步。
        # XML 自带的 joint damping/frictionloss 仍参与动力学，不能当作此处 kd 的别名。
        for i, actuator in enumerate(self.actuator_ids):
            value = torque[i]
            if self.model.actuator_ctrllimited[actuator]:
                low, high = self.model.actuator_ctrlrange[actuator]
                value = np.clip(value, low, high)
            self.data.ctrl[actuator] = value
        mujoco.mj_step(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)

    def joint_state(self):
        return (self.data.qpos[self.qpos_ids].copy(), self.data.qvel[self.qvel_ids].copy(),
                self.data.actuator_force[self.actuator_ids].copy())

    def imu_state(self):
        velocity, acceleration = np.zeros(6), np.zeros(6)
        # 姿态为 [w,x,y,z]，local=1 的速度/加速度位于 base 机体坐标系。
        # BODY 使用惯性主轴；XBODY 才与 xquat 和训练端 base 坐标系一致。
        # 本实现未添加 IMU 噪声、偏置、安装角和滤波；观测缩放在策略层处理。
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_XBODY,
                                self.base_id, velocity, 1)
        mujoco.mj_rnePostConstraint(self.model, self.data)
        mujoco.mj_objectAcceleration(self.model, self.data, mujoco.mjtObj.mjOBJ_XBODY,
                                    self.base_id, acceleration, 1)
        # rnePostConstraint 的 cacc 已包含重力补偿；自由落体时比力为零。
        return self.data.xquat[self.base_id].copy(), velocity[:3], acceleration[3:]
