"""MuJoCo 启动姿态、被动连杆闭合与物理子步回归检查。"""
import sys
import tempfile
import unittest
from pathlib import Path

import mujoco
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src/wheel_leg_mujoco"
sys.path.insert(0, str(PACKAGE))
from wheel_leg_mujoco.mujoco_backend import MujocoRobot
from wheel_leg_mujoco.robot_config import DEFAULT_POSITION, JOINT_NAMES

CONFIG = yaml.safe_load((PACKAGE / "config/hustdog.yaml").read_text())
POSE = CONFIG["go2w_mujoco"]["ros__parameters"]
STAND = CONFIG["robot_ctrl"]["ros__parameters"]["default_position"]
SCENES = ["scene_flat.xml", "scene_svproj.xml"]


def hustdog(scene):
    return MujocoRobot(str(PACKAGE / "models/hustdog_v11/urdf" / scene),
                       default_position=POSE["default_position"],
                       initial_height=POSE["initial_height"],
                       physics_substeps=POSE["physics_substeps"])


def equality_error(robot):
    data = robot.data
    rows = data.efc_type[:data.nefc] == mujoco.mjtConstraint.mjCNSTR_EQUALITY
    return float(np.max(np.abs(data.efc_pos[:data.nefc][rows]), initial=0))


def linkage_gap(robot):
    model, data = robot.model, robot.data
    gaps = []
    for prefix in ["left_3", "right_3", "left_4", "right_4"]:
        shank, crank, rod = [model.body(prefix + "_" + name).id
                             for name in ["shank", "crank", "rod"]]
        # 平行四边形：连杆远端与小腿销轴在各自连杆坐标中的位置由零位装配确定。
        tip = data.xpos[rod] + data.xmat[rod].reshape(3, 3) @ (
            model.body_pos[shank] - model.body_pos[crank])
        pin = data.xpos[shank] + data.xmat[shank].reshape(3, 3) @ model.body_pos[rod]
        gaps.append(np.linalg.norm(tip - pin))
    return max(gaps)


class MujocoInitializationTests(unittest.TestCase):
    def test_initial_pose_and_linkage(self):
        self.assertNotEqual(POSE["default_position"], STAND)
        for scene in SCENES:
            with self.subTest(scene=scene):
                robot = hustdog(scene)
                model, data = robot.model, robot.data
                np.testing.assert_allclose(robot.joint_state()[0], POSE["default_position"], atol=1e-7)
                self.assertEqual(model.neq, 8)
                base_geom = model.geom("base_link_collision0")
                clearance = data.geom(base_geom.id).xpos[2] - base_geom.size[2]
                self.assertGreaterEqual(clearance, 0.0)
                self.assertLess(clearance, 0.01)
                for index, joint in enumerate(robot.joint_ids):
                    if model.jnt_limited[joint]:
                        low, high = model.jnt_range[joint]
                        self.assertGreaterEqual(STAND[index], low)
                        self.assertLessEqual(STAND[index], high)
                self.assertLess(equality_error(robot), 1e-12)
                self.assertLess(linkage_gap(robot), 1e-9)
                for joint in range(model.njnt):
                    if model.jnt_limited[joint]:
                        q = data.qpos[model.jnt_qposadr[joint]]
                        low, high = model.jnt_range[joint]
                        self.assertGreaterEqual(q, low)
                        self.assertLessEqual(q, high)
                print(f"{scene}: initial equality error={equality_error(robot):.3g}, "
                      f"linkage gap={linkage_gap(robot):.3g} m", flush=True)

    def test_crouch_remains_stable_under_passive_damping(self):
        # 不依赖节点等待逻辑：实际推进物理，验证趴姿自然落地后不会翻倒或弹起。
        command = np.zeros((16, 6))
        command[:, 0] = 1
        command[:, 4] = 1
        command[3::4, 4] = 0.5
        for scene in SCENES:
            with self.subTest(scene=scene):
                robot = hustdog(scene)
                maximum_tilt = maximum_gap = 0.0
                for _ in range(1000):
                    robot.step(command)
                    vertical = robot.data.xmat[robot.base_id].reshape(3, 3)[2, 2]
                    maximum_tilt = max(maximum_tilt, np.arccos(np.clip(vertical, -1, 1)))
                    maximum_gap = max(maximum_gap, linkage_gap(robot))
                self.assertAlmostEqual(robot.data.time, 5.0, places=9)
                self.assertTrue(np.isfinite(robot.data.qpos).all())
                self.assertTrue(np.isfinite(robot.data.qvel).all())
                self.assertFalse(np.any(robot.data.warning.number))
                self.assertLess(maximum_tilt, np.deg2rad(5))
                self.assertLess(maximum_gap, 0.005)
                self.assertGreater(robot.data.qpos[2], 0.08)
                self.assertLess(robot.data.qpos[2], 0.11)
                self.assertLess(np.linalg.norm(robot.data.qvel[:6]), 0.02)
                print(f"{scene}: 5 s passive crouch, maximum tilt={np.rad2deg(maximum_tilt):.3f} deg, "
                      f"base height={robot.data.qpos[2]:.4f} m", flush=True)

    def test_stand_commands_preserve_linkage(self):
        command = np.zeros((16, 6))
        command[:, 0] = 1
        command[:, 1] = POSE["default_position"]
        command[:, 3] = 40
        command[:, 4] = 1
        command[3::4, 3] = 0
        command[3::4, 4] = 0.5
        for scene in SCENES:
            with self.subTest(scene=scene):
                robot = hustdog(scene)
                maximum_gap = 0.0
                for tick in range(600):
                    # 与控制器相同：前 1.5 秒从趴下姿态插值到 Stand，随后保持。
                    ratio = min((tick + 1) * 0.005 / 1.5, 1.0)
                    command[:, 1] = np.asarray(POSE["default_position"]) + ratio * (
                        np.asarray(STAND) - POSE["default_position"])
                    robot.step(command)
                    maximum_gap = max(maximum_gap, linkage_gap(robot))
                    self.assertTrue(np.isfinite(robot.data.qpos).all())
                    self.assertTrue(np.isfinite(robot.data.qvel).all())
                self.assertAlmostEqual(robot.data.time, 3.0, places=9)
                self.assertFalse(np.any(robot.data.warning.number))
                self.assertLess(maximum_gap, 0.005)
                print(f"{scene}: 3 s joint motion, maximum linkage gap={maximum_gap * 1000:.3f} mm",
                      flush=True)

    def test_go2w_defaults_are_preserved(self):
        path = str(PACKAGE / "models/go2w/urdf/scene_flat.xml")
        robot = MujocoRobot(path)
        np.testing.assert_array_equal(robot.joint_state()[0], np.asarray(DEFAULT_POSITION, dtype=np.float32))
        self.assertEqual(robot.physics_substeps, 1)
        self.assertEqual(robot.model.opt.timestep, 0.005)
        self.assertEqual(robot.model.opt.integrator, mujoco.mjtIntegrator.mjINT_EULER)
        robot.step(None)
        self.assertAlmostEqual(robot.data.time, 0.005)

    def test_reference_angles_and_dependency_order(self):
        # 由 MuJoCo 自己计算约束残差，验证非零 ref、非线性多项式、倒序依赖及固定关节。
        bodies = []
        motors = []
        for index, name in enumerate(JOINT_NAMES):
            ref = 0.2 if name == "FL_calf_joint" else 0.0
            bodies.append(f'<body pos="{index * 0.04} 0 0"><joint name="{name}" ref="{ref}"/>'
                          '<geom type="sphere" size="0.01"/></body>')
            motors.append(f'<motor joint="{name}"/>')
        for name, ref in [("passive_a", 0.4), ("passive_b", -0.1),
                          ("fixed_passive", 0.3), ("inactive_passive", 0.6)]:
            bodies.append(f'<body><joint name="{name}" ref="{ref}"/>'
                          '<geom type="sphere" size="0.01"/></body>')
        xml = ('<mujoco><compiler angle="radian"/><default><geom contype="0" conaffinity="0"/>'
               '</default><worldbody><body name="base"><freejoint/>'
               '<geom type="box" size="0.1 0.1 0.1"/>' + ''.join(bodies) +
               '</body></worldbody><actuator>' + ''.join(motors) + '</actuator><equality>'
               '<joint joint1="passive_b" joint2="passive_a" polycoef="-0.2 -0.5 0 0 0"/>'
               '<joint joint1="passive_a" joint2="FL_calf_joint" polycoef="0.2 1.5 0.3 0 0"/>'
               '<joint joint1="fixed_passive" polycoef="0.05 0 0 0 0"/>'
               '<joint joint1="inactive_passive" polycoef="2 0 0 0 0" active="false"/>'
               '</equality></mujoco>')
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "reference.xml"
            path.write_text(xml)
            robot = MujocoRobot(str(path), default_position=[0.7] * 16)
            self.assertLess(equality_error(robot), 1e-12)
            address = robot.model.joint("inactive_passive").qposadr[0]
            self.assertEqual(robot.data.qpos[address], robot.model.qpos0[address])

    def test_invalid_substeps(self):
        path = str(PACKAGE / "models/go2w/urdf/scene_flat.xml")
        for invalid in [0, -1, True, 1.5]:
            with self.subTest(value=invalid), self.assertRaisesRegex(ValueError, "physics_substeps"):
                MujocoRobot(path, physics_substeps=invalid)


if __name__ == "__main__":
    unittest.main(verbosity=2)
