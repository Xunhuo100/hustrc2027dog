#!/usr/bin/env python3
"""量化 parkour 的前进、左右侧移和首个高墙；使用固定仿真步长排除 ROS 调度抖动。"""
import argparse
import sys
from pathlib import Path

import mujoco
import numpy as np
import torch
import yaml

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src/himloco_sim2sim"
sys.path.insert(0, str(PACKAGE))
from himloco_sim2sim.mujoco_backend import MujocoRobot
from himloco_sim2sim.policy_core import HimPolicyCore


def run_case(model, command, fields=False):
    profile = yaml.safe_load((PACKAGE / "config/parkour.yaml").read_text())
    parameters = profile["himloco_policy"]["ros__parameters"]
    core = HimPolicyCore(parameters)
    scene = "scene_fields.xml" if fields else "scene_flat.xml"
    xml = Path.home() / "dogMujoco2027/unitree_robots/go2_w/urdf" / scene
    robot = MujocoRobot(str(xml), default_position=core.default_position, initial_height=0.45)
    q, dq, kp, kd = core.action_to_targets(np.zeros(16))
    output = np.column_stack([np.ones(16), q, dq, kp, kd, np.zeros(16)])
    for _ in range(300):
        robot.step(output)
    angles, speeds = [], []
    wall_contact = False
    wall_crossing = None
    for step in range(1500 if fields else 600):
        position, velocity, torque = robot.joint_state()
        quaternion, gyro, acceleration = robot.imu_state()
        history = core.observe(position, velocity, quaternion, gyro, command)
        with torch.no_grad():
            action = model(torch.from_numpy(history)).numpy()[0]
        q, dq, kp, kd = core.action_to_targets(action)
        output = np.column_stack([np.ones(16), q, dq, kp, kd, np.zeros(16)])
        for _ in range(4):
            robot.step(output)
        assert np.isfinite(robot.data.qpos).all() and np.isfinite(robot.data.qvel).all()
        rotation = robot.data.xmat[robot.base_id].reshape(3, 3)
        assert rotation[2, 2] > 0.5, "Robot exceeded 60 degrees of tilt"
        roll = np.arctan2(rotation[2, 1], rotation[2, 2])
        pitch = np.arcsin(np.clip(-rotation[2, 0], -1, 1))
        if step >= 100:
            angles.append(np.rad2deg([roll, pitch]))
            speeds.append(rotation.T @ robot.data.qvel[:3])
        if fields:
            for contact in robot.data.contact:
                for geom in [contact.geom1, contact.geom2]:
                    body = int(robot.model.geom_bodyid[geom])
                    name = mujoco.mj_id2name(robot.model, mujoco.mjtObj.mjOBJ_BODY, body) or ""
                    if name == "field_high_wall":
                        wall_contact = True
            if wall_crossing is None and robot.data.qpos[0] >= 3.0:
                wall_crossing = robot.data.qpos[:3].copy()
            if robot.data.qpos[0] >= 4.0:
                break
    angles = np.asarray(angles)
    rms = np.sqrt(np.mean(angles ** 2, axis=0))
    peak = np.max(np.abs(angles), axis=0)
    speed = np.mean(speeds, axis=0)
    if fields:
        assert wall_contact, "Did not contact the high wall"
        assert wall_crossing is not None and abs(wall_crossing[1]) < 0.25, "Did not cross wall centre"
        assert robot.data.qpos[0] >= 4.0, "Did not reach the other side of the wall"
        assert abs(robot.data.qpos[1]) < 0.25, "Robot went around the wall"
        print("PASS high wall: crossing xyz=", np.round(wall_crossing, 3),
              "end xyz=", np.round(robot.data.qpos[:3], 3),
              "time=", round((step + 1) * 0.02, 2), flush=True)
    else:
        assert np.max(peak) < 15.0, "Excessive roll/pitch oscillation"
        axis = 0 if command[0] else 1
        assert speed[axis] * np.sign(command[axis]) > 0.1, "Did not follow commanded direction"
        print("PASS command=", command, "body speed=", np.round(speed, 3),
              "roll/pitch RMS(deg)=", np.round(rms, 2),
              "peak(deg)=", np.round(peak, 2), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fields", action="store_true", help="同时检查原障碍场景的首个 0.3m 高墙")
    args = parser.parse_args()
    torch.set_num_threads(1)
    model = torch.jit.load(str(PACKAGE / "policies/parkour.pt"), map_location="cpu").eval()
    for command in [[0.3, 0.0, 0.0], [0.0, 0.3, 0.0], [0.0, -0.3, 0.0]]:
        run_case(model, command)
    if args.fields:
        run_case(model, [1.0, 0.0, 0.0], fields=True)


if __name__ == "__main__":
    main()
