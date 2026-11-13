#!/usr/bin/env python3
"""检查显式 FSM 生命周期、站立插值、映射、会话隔离、坏数据和超时。"""
import os
import signal
import subprocess
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from unitree_go.msg import LowState, LowCmd
from wheel_leg_interfaces.msg import RobotState, RobotCommand

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_Q = [0.1, 0.8, -1.5, 0.0, -0.1, 0.8, -1.5, 0.0,
             0.1, 1.0, -1.5, 0.0, -0.1, 1.0, -1.5, 0.0]


class FakeRobot(Node):
    def __init__(self):
        super().__init__("controller_interface_check")
        self.state, self.command = None, None
        self.feedback = True
        self.state_sub = self.create_subscription(RobotState, "/robot/state", self.on_state, 1)
        self.command_sub = self.create_subscription(LowCmd, "/lowcmd", self.on_command, 1)
        self.feedback_pub = self.create_publisher(LowState, "/lowstate", qos_profile_sensor_data)
        self.policy_pub = self.create_publisher(RobotCommand, "/robot/rl_cmd", 1)
        self.mode_pub = self.create_publisher(String, "/robot/mode", 1)
        self.feedback_msg = LowState()
        self.feedback_msg.imu_state.quaternion = [1.0, 0.0, 0.0, 0.0]
        self.feedback_msg.imu_state.gyroscope = [1.0, 2.0, 3.0]
        self.feedback_msg.imu_state.accelerometer = [0.0, 0.0, 9.81]
        for i in range(20):
            self.feedback_msg.motor_state[i].q = i * 0.01

    def on_state(self, message):
        self.state = message

    def on_command(self, message):
        self.command = message

    def pump(self, seconds, predicate=None, policy=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if self.feedback:
                self.feedback_pub.publish(self.feedback_msg)
            if policy is not None:
                message = self.policy_command()
                policy(message)
                self.policy_pub.publish(message)
            rclpy.spin_once(self, timeout_sec=0.005)
            if predicate is not None and predicate():
                return
        if predicate is not None:
            raise AssertionError("Expected controller condition was not reached")

    def mode(self, name):
        message = String()
        message.data = name
        self.mode_pub.publish(message)

    def enter_rl(self):
        self.mode("stand")
        self.pump(1, lambda: self.state.control_mode == RobotState.STAND)
        self.pump(0.3)
        self.mode("rl")
        self.pump(1, lambda: self.state.control_mode == RobotState.RL)

    def policy_command(self):
        message = RobotCommand()
        message.header.stamp = self.get_clock().now().to_msg()
        message.control_session = self.state.control_session
        for i in range(16):
            motor = message.motor_cmd[i]
            motor.mode = 1
            motor.q = DEFAULT_Q[i]
            motor.kp, motor.kd = 20.0, 0.5
            if i % 4 == 3:
                motor.q, motor.kp, motor.kd = 0.0, 0.0, 0.3
                motor.dq = float((i + 1) // 4)
        return message


def main():
    os.environ.setdefault("ROS_DOMAIN_ID", "74")
    rclpy.init()
    robot = FakeRobot()
    log = open("/tmp/hustrc2027dog_controller_check.log", "w")
    process = subprocess.Popen([
        str(ROOT / "install/robot_ctrl/lib/robot_ctrl/robot_ctrl"), "--ros-args",
        "-p", "stand_duration:=0.25", "-p", "policy_timeout:=0.15",
        "-p", "motor_mapping:=[15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0]"],
        stdout=log, stderr=subprocess.STDOUT)
    try:
        robot.pump(5, lambda: robot.state is not None and robot.command is not None)
        assert abs(robot.state.motor_state[0].q - 0.15) < 1e-6
        assert abs(robot.state.motor_state[15].q) < 1e-6
        assert robot.state.imu.orientation.w == 1
        assert robot.state.imu.angular_velocity.y == 2
        print("PASS all 16 feedback slots and IMU conversion", flush=True)
        assert robot.state.control_mode == RobotState.PASSIVE
        assert robot.state.control_session == 0
        for i in range(16):
            motor = robot.command.motor_cmd[15 - i]
            assert motor.mode == 1 and motor.kp == 0 and motor.dq == 0 and motor.tau == 0
            assert abs(motor.kd - (0.3 if i % 4 == 3 else 1.0)) < 1e-6
        print("PASS Passive generates damping commands", flush=True)
        robot.mode("rl")
        robot.pump(0.05)
        assert robot.state.control_mode == RobotState.PASSIVE
        assert robot.state.control_session == 0
        robot.mode("stand")
        robot.pump(1, lambda: robot.state.control_mode == RobotState.STAND and
                   0.14 < robot.command.motor_cmd[14].q < 0.8)
        stand_session = robot.state.control_session
        assert stand_session == 1
        assert robot.command.motor_cmd[14].kp == 20.0
        assert robot.command.motor_cmd[14].kd == 0.5
        for i in [3, 7, 11, 15]:
            motor = robot.command.motor_cmd[15 - i]
            assert motor.q == 0 and motor.kp == 0 and motor.dq == 0
        robot.mode("rl")
        robot.pump(0.03)
        assert robot.state.control_mode == RobotState.STAND
        assert robot.state.control_session == stand_session
        robot.mode("stand")
        robot.pump(0.04)
        # 重复请求不重启 Stand 插值，且不会产生新会话。
        assert robot.state.control_session == stand_session
        robot.pump(0.3)
        for i in range(16):
            assert abs(robot.command.motor_cmd[15 - i].q - DEFAULT_Q[i]) < 1e-6
        robot.mode("rl")
        robot.pump(1, lambda: robot.state.control_mode == RobotState.RL)
        assert robot.state.control_session == stand_session + 1
        robot.pump(0.02)
        assert all(robot.command.motor_cmd[15 - i].dq == 0 for i in [3, 7, 11, 15])
        assert abs(robot.command.motor_cmd[14].q - DEFAULT_Q[1]) < 1e-6
        print("PASS Stand interpolation, RL entry gate, sessions and initial stand hold", flush=True)
        robot.pump(1, lambda: robot.command.motor_cmd[0].dq == 4.0, lambda m: None)
        assert robot.command.motor_cmd[12].dq == 1.0
        assert all(robot.command.motor_cmd[i].mode == 0 for i in range(16, 20))
        assert robot.command.crc != 0
        print("PASS 16th motor output, reverse mapping, reserved slots and CRC", flush=True)
        # RL 的 PD 直接来自策略，不能被 Stand 参数覆盖。
        def different_pd(message):
            message.motor_cmd[1].kp = 31.0
            message.motor_cmd[1].kd = 0.7
            message.motor_cmd[1].tau = 0.2
        robot.pump(1, lambda: robot.command.motor_cmd[14].kp == 31.0, different_pd)
        assert abs(robot.command.motor_cmd[14].kd - 0.7) < 1e-6
        assert abs(robot.command.motor_cmd[14].tau - 0.2) < 1e-6
        previous_session = robot.state.control_session
        robot.mode("stand")
        robot.pump(1, lambda: robot.state.control_mode == RobotState.STAND and
                   robot.command.motor_cmd[14].kp == 20.0 and
                   all(robot.command.motor_cmd[15 - i].dq == 0 for i in [3, 7, 11, 15]))
        assert robot.state.control_session == previous_session + 1
        assert robot.command.motor_cmd[14].tau == 0
        robot.pump(0.3)
        robot.mode("rl")
        robot.pump(1, lambda: robot.state.control_mode == RobotState.RL)
        robot.pump(0.02)
        assert robot.state.control_session == previous_session + 2
        assert robot.command.motor_cmd[14].kp == 20.0
        assert all(robot.command.motor_cmd[15 - i].dq == 0 for i in [3, 7, 11, 15])
        print("PASS policy PD preserved, RL exit clears targets and re-entry cannot replay actions", flush=True)
        def bad_value(message):
            message.motor_cmd[15].dq = float("nan")
        robot.pump(1, lambda: robot.state.control_mode == RobotState.PASSIVE, bad_value)
        print("PASS nonfinite action -> Passive", flush=True)
        robot.enter_rl()
        def bad_session(message):
            message.control_session -= 1
        robot.pump(1, lambda: robot.state.control_mode == RobotState.PASSIVE, bad_session)
        print("PASS previous RL session rejected", flush=True)
        robot.enter_rl()
        def bad_timestamp(message):
            message.header.stamp.sec = -1
        robot.pump(1, lambda: robot.state.control_mode == RobotState.PASSIVE, bad_timestamp)
        print("PASS invalid timestamp rejected without crashing", flush=True)
        robot.enter_rl()
        robot.pump(1, lambda: robot.state.control_mode == RobotState.PASSIVE)
        print("PASS missing first policy command -> Passive", flush=True)
        robot.enter_rl()
        robot.pump(1, lambda: robot.command.motor_cmd[0].dq == 4.0, lambda m: None)
        robot.pump(1, lambda: robot.state.control_mode == RobotState.PASSIVE and
                   robot.command.motor_cmd[0].dq == 0 and robot.command.motor_cmd[14].kp == 0)
        print("PASS active policy timeout clears motor targets", flush=True)
        robot.enter_rl()
        # roll=90°，测试全局保护能在状态执行前抢占 RL 输出。
        robot.feedback_msg.imu_state.quaternion = [0.70710678, 0.70710678, 0.0, 0.0]
        robot.pump(1, lambda: robot.state.control_mode == RobotState.PASSIVE and
                   all(m.kp == 0 and m.dq == 0 and m.tau == 0 for m in robot.command.motor_cmd))
        print("PASS tilt protection preempts RL and outputs Passive damping", flush=True)
        robot.feedback = False
        robot.pump(1, lambda: all(m.mode == 0 for m in robot.command.motor_cmd))
        print("PASS feedback timeout disables all motor slots", flush=True)
        assert process.poll() is None
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        log.close()
        robot.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
