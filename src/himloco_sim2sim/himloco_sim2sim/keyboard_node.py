import math
import os
import select
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter


class KeyboardControl(Node):
    # 只发布期望速度，不发送电机命令。enable_keyboard 默认关闭且支持运行时切换。
    # 启用时每周期都会发布 Twist，使用导航速度源前应关闭键盘，避免两个源互相覆盖。
    def __init__(self):
        super().__init__("keyboard_control")
        enabled = self.declare_parameter("enable_keyboard", False).value
        fixed = ParameterDescriptor(read_only=True)
        # 速度/周期为只读启动参数；launch 的 keyboard_linear_speed 对应这里的 linear_speed。
        self.linear_speed = self.declare_parameter("linear_speed", 0.3, fixed).value
        self.angular_speed = self.declare_parameter("angular_speed", 0.6, fixed).value
        self.key_timeout = self.declare_parameter("key_timeout", 0.5, fixed).value
        period = self.declare_parameter("publish_period", 0.05, fixed).value
        for value in [self.linear_speed, self.angular_speed, self.key_timeout, period]:
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError("Keyboard speeds, key_timeout and publish_period must be positive")

        velocity_topic = self.declare_parameter("velocity_topic", "/cmd_vel", fixed).value
        self.publisher = self.create_publisher(Twist, velocity_topic, 1)
        self.enabled = False
        self.fd = None
        self.terminal_settings = None
        self.buffer = ""
        self.velocity = [0.0, 0.0, 0.0]
        self.key_time = [0.0, 0.0, 0.0]
        self.last_printed = None
        self.timer = self.create_timer(period, self.update)
        self.add_on_set_parameters_callback(self.parameter_callback)
        if enabled:
            self.start_keyboard()
        else:
            self.get_logger().info("Keyboard disabled (enable_keyboard=false)")

    def start_keyboard(self):
        # ros2 launch 不转发 stdin；直接打开启动终端，ros2 run 也使用同一入口。
        fd = os.open("/dev/tty", os.O_RDONLY | os.O_NONBLOCK | os.O_NOCTTY)
        try:
            settings = termios.tcgetattr(fd)
            tty.setcbreak(fd, termios.TCSANOW)
        except (OSError, termios.error):
            os.close(fd)
            raise
        self.fd = fd
        self.terminal_settings = settings
        self.buffer = ""
        self.velocity = [0.0, 0.0, 0.0]
        self.key_time = [0.0, 0.0, 0.0]
        self.last_printed = None
        self.enabled = True
        self.get_logger().info(
            "Keyboard enabled: arrows=translation, ','=left turn, '.'=right turn, "
            "space=stop, Ctrl+C=exit. Hold a key to keep moving.")

    def stop_keyboard(self):
        if self.enabled and rclpy.ok():
            self.publisher.publish(Twist())
        self.enabled = False
        self.velocity = [0.0, 0.0, 0.0]
        if self.fd is not None:
            try:
                termios.tcsetattr(self.fd, termios.TCSANOW, self.terminal_settings)
            finally:
                os.close(self.fd)
                self.fd = None

    def parameter_callback(self, parameters):
        for parameter in parameters:
            if parameter.name != "enable_keyboard" or parameter.value == self.enabled:
                continue
            try:
                if parameter.value:
                    self.start_keyboard()
                else:
                    self.stop_keyboard()
            except (OSError, termios.error) as error:
                return SetParametersResult(
                    successful=False, reason="Keyboard needs a controlling terminal: " + str(error))
        return SetParametersResult(successful=True)

    def read_keys(self):
        if select.select([self.fd], [], [], 0)[0]:
            data = os.read(self.fd, 1024)
            if not data:
                raise OSError("Keyboard terminal closed")
            self.buffer += data.decode("ascii", errors="ignore")
        keys = []
        while self.buffer:
            if self.buffer[0] != "\x1b":
                keys.append(self.buffer[0])
                self.buffer = self.buffer[1:]
                continue
            if len(self.buffer) < 2:
                break
            if self.buffer[1] not in ["[", "O"]:
                self.buffer = self.buffer[1:]
                continue
            # 兼容普通方向键、应用光标模式及修饰键，保留未收完的字节。
            end = 2
            while end < len(self.buffer) and not ("@" <= self.buffer[end] <= "~"):
                end += 1
            if end == len(self.buffer):
                if len(self.buffer) > 32:
                    self.buffer = ""
                break
            if self.buffer[end] in "ABCD":
                keys.append("arrow_" + self.buffer[end])
            self.buffer = self.buffer[end + 1:]
        return keys

    def update(self):
        if not self.enabled:
            return
        try:
            keys = self.read_keys()
        except (OSError, termios.error) as error:
            self.get_logger().error("Keyboard stopped: " + str(error))
            self.set_parameters([Parameter("enable_keyboard", value=False)])
            return
        now = time.monotonic()
        for key in keys:
            if key == " ":
                self.velocity = [0.0, 0.0, 0.0]
                self.key_time = [0.0, 0.0, 0.0]
                continue
            if key == "arrow_A":
                axis, value = 0, self.linear_speed
            elif key == "arrow_B":
                axis, value = 0, -self.linear_speed
            elif key == "arrow_D":
                axis, value = 1, self.linear_speed
            elif key == "arrow_C":
                axis, value = 1, -self.linear_speed
            elif key == ",":
                axis, value = 2, self.angular_speed
            elif key == ".":
                axis, value = 2, -self.angular_speed
            else:
                continue
            self.velocity[axis] = value
            self.key_time[axis] = now
        # 终端没有松键事件，每个方向独立超时；允许平移和转向组合。
        for axis in range(3):
            if now - self.key_time[axis] >= self.key_timeout:
                self.velocity[axis] = 0.0
        message = Twist()
        message.linear.x, message.linear.y, message.angular.z = self.velocity
        self.publisher.publish(message)
        if self.velocity != self.last_printed:
            self.get_logger().info(
                "Desired velocity: vx=%.2f m/s, vy=%.2f m/s, wz=%.2f rad/s" %
                tuple(self.velocity))
            self.last_printed = self.velocity.copy()

    def destroy_node(self):
        self.stop_keyboard()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = KeyboardControl()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
