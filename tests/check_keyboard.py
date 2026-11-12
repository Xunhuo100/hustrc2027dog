#!/usr/bin/env python3
"""用真实终端、ROS2 参数服务及 Twist 话题检查键盘节点。"""
import fcntl
import os
import pty
import signal
import subprocess
import sys
import termios
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter

ROOT = Path(__file__).resolve().parents[1]


class Monitor(Node):
    def __init__(self):
        super().__init__("keyboard_check")
        self.samples = []
        self.subscription = self.create_subscription(Twist, "/cmd_vel", self.receive, 10)
        self.client = self.create_client(SetParameters, "/keyboard_control/set_parameters")

    def receive(self, message):
        self.samples.append([message.linear.x, message.linear.y, message.angular.z])

    def spin_for(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def enable(self, value):
        request = SetParameters.Request()
        request.parameters = [Parameter("enable_keyboard", value=value).to_parameter_msg()]
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3)
        assert future.done(), "Parameter service timed out"
        result = future.result().results[0]
        assert result.successful, result.reason

    def expect(self, velocity, timeout=1.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.samples and self.samples[-1] == velocity:
                return
        raise AssertionError("Expected velocity %s, got %s" % (velocity, self.samples[-3:]))


def controlling_terminal():
    os.setsid()
    fcntl.ioctl(0, termios.TIOCSCTTY, 0)


def main():
    os.environ.setdefault("ROS_DOMAIN_ID", "75")
    rclpy.init()
    node = Monitor()
    master, slave = pty.openpty()
    original = termios.tcgetattr(slave)
    log = open("/tmp/hustrc2027dog_keyboard.log", "w")
    process = subprocess.Popen(
        [sys.executable, str(ROOT / "install/himloco_sim2sim/lib/himloco_sim2sim/keyboard_control"),
         "--ros-args", "-p", "key_timeout:=0.3"],
        stdin=slave, stdout=log, stderr=subprocess.STDOUT, preexec_fn=controlling_terminal)
    try:
        assert node.client.wait_for_service(timeout_sec=5), "Keyboard node did not start"
        node.spin_for(0.3)
        assert not node.samples, "Disabled keyboard published /cmd_vel"
        assert termios.tcgetattr(slave) == original, "Disabled keyboard modified terminal"
        print("PASS default disabled: no velocity publication or terminal changes", flush=True)

        node.enable(True)
        node.expect([0.0, 0.0, 0.0])
        assert not termios.tcgetattr(slave)[3] & termios.ICANON
        for key, velocity in [
                (b"\x1b[A", [0.3, 0.0, 0.0]),
                (b"\x1b[B", [-0.3, 0.0, 0.0]),
                (b"\x1b[D", [0.0, 0.3, 0.0]),
                (b"\x1b[C", [0.0, -0.3, 0.0]),
                (b",", [0.0, 0.0, 0.6]),
                (b".", [0.0, 0.0, -0.6]),
                (b"\x1bOA", [0.3, 0.0, 0.0]),
                (b"\x1b[1;5D", [0.0, 0.3, 0.0])]:
            node.samples.clear()
            os.write(master, b" " + key)
            node.expect(velocity)
        print("PASS four arrows, comma/period and terminal escape modes", flush=True)

        os.write(master, b" \x1b")
        node.spin_for(0.08)
        node.samples.clear()
        os.write(master, b"[A,")
        node.expect([0.3, 0.0, 0.6])
        node.spin_for(0.18)
        os.write(master, b",")
        node.spin_for(0.17)
        node.expect([0.0, 0.0, 0.6])
        node.samples.clear()
        node.expect([0.0, 0.0, 0.0])
        print("PASS fragmented arrows, combined motion and independent key timeouts", flush=True)

        os.write(master, b"\x1b[A")
        node.expect([0.3, 0.0, 0.0])
        os.write(master, b" ")
        node.samples.clear()
        node.expect([0.0, 0.0, 0.0])
        node.enable(False)
        node.spin_for(0.15)
        node.samples.clear()
        node.spin_for(0.2)
        assert not node.samples, "Disabled keyboard kept publishing"
        assert termios.tcgetattr(slave) == original, "Disable did not restore terminal"
        node.enable(True)
        process.send_signal(signal.SIGINT)
        process.wait(timeout=5)
        assert process.returncode == 0, "Keyboard shutdown failed; see keyboard log"
        assert termios.tcgetattr(slave) == original, "Exit did not restore terminal"
        print("PASS space stop, runtime disable/re-enable and terminal restore on exit", flush=True)
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        termios.tcsetattr(slave, termios.TCSANOW, original)
        os.close(master)
        os.close(slave)
        log.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
