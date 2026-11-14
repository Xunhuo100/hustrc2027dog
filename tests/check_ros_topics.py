#!/usr/bin/env python3
"""检查同一控制器通过 ROS2 topic 切换仿真和外部驱动。"""
import os
import math
import pty
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from unitree_go.msg import LowCmd, LowState
from wheel_leg_interfaces.msg import RobotState
from check_keyboard import controlling_terminal

ROOT = Path(__file__).resolve().parents[1]


class Monitor(Node):
    def __init__(self, command_topic, state_topic, velocity_topic):
        super().__init__("ros_topic_check")
        self.state, self.command, self.feedback, self.velocity = None, None, None, None
        self.state_sub = self.create_subscription(RobotState, "/robot/state", self.on_state, 1)
        self.command_sub = self.create_subscription(LowCmd, command_topic, self.on_command, 1)
        self.feedback_sub = self.create_subscription(LowState, state_topic, self.on_feedback,
                                                     qos_profile_sensor_data)
        self.velocity_sub = self.create_subscription(Twist, velocity_topic, self.on_velocity, 1)
        self.client = self.create_client(SetParameters, "/keyboard_control/set_parameters")

    def on_state(self, message):
        self.state = message

    def on_command(self, message):
        self.command = message

    def on_feedback(self, message):
        self.feedback = message

    def on_velocity(self, message):
        self.velocity = message

    def until(self, predicate, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)
            if predicate():
                return
        raise AssertionError("ROS topic check timed out; see /tmp/hustrc2027dog_topics_*.log")


def run_case(external, profile="parkour"):
    os.environ["ROS_DOMAIN_ID"] = "79" if profile == "flat" else ("77" if external else "76")
    prefix = "/test_driver" if external else "/sim"
    command_topic, state_topic = prefix + "/lowcmd", prefix + "/lowstate"
    velocity_topic = "/navigation/cmd_vel" if external else "/cmd_vel"
    rclpy.init()
    node = Monitor(command_topic, state_topic, velocity_topic)
    master, slave = pty.openpty()
    processes, logs = [], []

    def start(name, command, **kwargs):
        log = open("/tmp/hustrc2027dog_topics_" + name + ".log", "w")
        logs.append(log)
        process = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, **kwargs)
        processes.append(process)
        return process

    try:
        arguments = ["policy_profile:=" + profile]
        if external:
            # 用独立 ROS2 MuJoCo 进程模拟已有的硬件驱动，不连接真实电机。
            start("driver", [sys.executable,
                str(ROOT / "install/wheel_leg_mujoco/lib/wheel_leg_mujoco/go2w_mujoco"),
                "--ros-args", "-p", "motor_command_topic:=" + command_topic,
                "-p", "motor_state_topic:=" + state_topic])
            arguments += ["start_simulator:=false", "motor_command_topic:=" + command_topic,
                          "motor_state_topic:=" + state_topic, "velocity_topic:=" + velocity_topic]
        launch = start("launch", ["ros2", "launch", str(ROOT / "launch/dog_mujoco.launch.py")] + arguments,
                       stdin=slave, preexec_fn=controlling_terminal)
        node.until(lambda: node.state is not None and node.state.control_mode == RobotState.RL, 20)
        assert node.command is not None and node.feedback is not None
        assert node.velocity is None, "Default disabled keyboard published velocity"
        assert len(node.feedback.motor_state) == 20
        assert sum(v * v for v in node.feedback.imu_state.quaternion) > 0.5
        assert node.client.wait_for_service(timeout_sec=2)
        request = SetParameters.Request()
        request.parameters = [Parameter("enable_keyboard", value=True).to_parameter_msg()]
        future = node.client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=3)
        assert future.done() and future.result().results[0].successful
        speed = 0.0
        end = time.monotonic() + 3.0
        while time.monotonic() < end:
            os.write(master, b"\x1b[A")
            rclpy.spin_once(node, timeout_sec=0.02)
            assert node.state.control_mode == RobotState.RL
            assert launch.poll() is None
            speed = max(speed, max(abs(node.state.motor_state[i].dq) for i in [3, 7, 11, 15]))
            assert all(node.command.motor_cmd[i].kp == 0 for i in [3, 7, 11, 15])
            assert node.command.motor_cmd[0].kp == (40.0 if profile == "parkour" else 20.0)
        assert node.velocity is not None and node.velocity.linear.x == 0.3
        assert speed > 0.1
        os.write(master, b" ")
        node.until(lambda: node.velocity.linear.x == 0.0, 2)
        if not external and profile == "parkour":
            # 完整 ROS2 链路下检查两侧平移，避免只验证轮子有转动。
            for key, desired in [(b"\x1b[D", 0.3), (b"\x1b[C", -0.3)]:
                peak_tilt = 0.0
                end = time.monotonic() + 6.0
                while time.monotonic() < end:
                    os.write(master, key)
                    rclpy.spin_once(node, timeout_sec=0.02)
                    assert node.state.control_mode == RobotState.RL
                    q = node.state.imu.orientation
                    cosine = max(-1.0, min(1.0, 1.0 - 2.0 * (q.x * q.x + q.y * q.y)))
                    peak_tilt = max(peak_tilt, math.degrees(math.acos(cosine)))
                    assert peak_tilt < 20.0, "Excessive tilt during ROS2 lateral motion"
                    assert node.command.motor_cmd[0].kp == 40.0, "Parkour profile was not applied"
                assert node.velocity.linear.y == desired
                os.write(master, b" ")
                node.until(lambda: node.velocity.linear.y == 0.0, 2)
                print("PASS ROS2 parkour lateral vy=", desired, "peak tilt(deg)=", peak_tilt, flush=True)
        print("PASS", profile, "external driver topics" if external else "default /sim topics",
              ": keyboard -> desired velocity -> RL -> motor feedback, max wheel speed", speed, flush=True)
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
        for process in reversed(processes):
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        os.close(master)
        os.close(slave)
        for log in logs:
            log.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    run_case(False)
    run_case(True)
    run_case(False, "flat")
