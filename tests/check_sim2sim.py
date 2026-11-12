#!/usr/bin/env python3
"""真实 ROS2 + TorchScript + MuJoCo 闭环，以及策略/反馈断连检查。"""
import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from unitree_go.msg import LowCmd
from wheel_leg_interfaces.msg import RobotState

ROOT = Path(__file__).resolve().parents[1]


class Monitor(Node):
    def __init__(self):
        super().__init__("sim2sim_check")
        self.state, self.command = None, None
        self.state_sub = self.create_subscription(RobotState, "/robot/state", self.on_state, 1)
        self.command_sub = self.create_subscription(LowCmd, "/lowcmd", self.on_command, 1)
        self.mode_pub = self.create_publisher(String, "/robot/mode", 1)
        self.velocity_pub = self.create_publisher(Twist, "/cmd_vel", 1)

    def on_state(self, message):
        self.state = message

    def on_command(self, message):
        self.command = message

    def until(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)
            if predicate():
                print("PASS", description, flush=True)
                return
        raise AssertionError(description)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-xml", help="MuJoCo 场景路径；不指定时使用默认平地")
    args = parser.parse_args()
    os.environ.setdefault("ROS_DOMAIN_ID", "73")
    rclpy.init()
    node = Monitor()
    processes, logs = [], []

    def start(name, command):
        log = open("/tmp/hustrc2027dog_" + name + ".log", "w")
        logs.append(log)
        process = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
        processes.append(process)
        return process

    try:
        controller = start("controller", [str(ROOT / "install/robot_ctrl/lib/robot_ctrl/robot_ctrl")])
        simulator_command = [sys.executable, str(ROOT / "src/himloco_sim2sim/scripts/go2w_mujoco")]
        if args.model_xml:
            simulator_command += ["--ros-args", "-p", "model_xml:=" + str(Path(args.model_xml).expanduser())]
        simulator = start("mujoco", simulator_command)
        policy = start("policy", [sys.executable, str(ROOT / "src/himloco_sim2sim/scripts/himloco_policy"),
                                  "--ros-args", "-p", "autostart:=true"])
        node.until(lambda: node.state is not None and node.state.control_mode == RobotState.RL,
                   20, "Passive -> Stand -> RL, real policy and MuJoCo connected")
        maximum_wheel_speed = 0.0
        end = time.monotonic() + 6.0
        while time.monotonic() < end:
            message = Twist()
            message.linear.x = 0.3
            node.velocity_pub.publish(message)
            rclpy.spin_once(node, timeout_sec=0.02)
            assert node.state.control_mode == RobotState.RL, "RL exited during closed loop"
            maximum_wheel_speed = max(maximum_wheel_speed, max(
                abs(node.state.motor_state[i].dq) for i in [3, 7, 11, 15]))
            assert all(node.command.motor_cmd[i].kp == 0 for i in [3, 7, 11, 15])
        assert maximum_wheel_speed > 0.1, "Wheel feedback did not respond to motion command"
        print("PASS 6 s closed loop; maximum wheel speed:", maximum_wheel_speed, flush=True)
        policy.send_signal(signal.SIGINT)
        policy.wait(timeout=5)
        node.until(lambda: node.state.control_mode == RobotState.PASSIVE and
                   all(node.command.motor_cmd[i].kp == 0 for i in range(16)), 2,
                   "Policy process stopped -> Passive")
        simulator.send_signal(signal.SIGINT)
        simulator.wait(timeout=5)
        node.until(lambda: node.command is not None and
                   all(m.mode == 0 for m in node.command.motor_cmd), 2,
                   "Motor feedback stopped -> all 20 motor message slots disabled")
        assert controller.poll() is None
    finally:
        for process in processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        for log in logs:
            log.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
