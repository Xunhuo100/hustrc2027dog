import os
import time
import numpy as np
import mujoco
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from unitree_go.msg import LowCmd, LowState
from sensor_msgs.msg import Image, CameraInfo
from .mujoco_backend import MujocoRobot
from .policy_core import DEFAULT_POSITION


class Go2WMujoco(Node):
    # ROS2 仿真驱动：LowCmd 输入、LowState 输出，策略不需要知道内部使用 MuJoCo。
    def __init__(self):
        super().__init__("go2w_mujoco")
        default_xml = os.path.expanduser("~/dogMujoco2027/unitree_robots/go2_w/urdf/scene_flat.xml")
        path = os.path.expanduser(self.declare_parameter("model_xml", default_xml).value)
        period = self.declare_parameter("simulation_period", 0.005).value
        position = self.declare_parameter("default_position", DEFAULT_POSITION).value
        height = self.declare_parameter("initial_height", 0.42).value
        self.robot = MujocoRobot(path, period, position, height)
        self.command_timeout = self.declare_parameter("command_timeout", 0.1).value
        self.waiting = self.declare_parameter("wait_for_controller", True).value
        self.command, self.command_time = None, 0.0
        self.viewer, self.renderer = None, None
        self.viewer_step = 0
        self.follow_camera = self.declare_parameter("follow_camera", True).value
        camera_distance = self.declare_parameter("camera_distance", 3.0).value
        if not np.isfinite(camera_distance) or camera_distance <= 0.0:
            raise ValueError("camera_distance must be positive and finite")
        if self.declare_parameter("viewer", False).value:
            from mujoco import viewer
            self.viewer = viewer.launch_passive(self.robot.model, self.robot.data)
            if self.follow_camera:
                # 观察窗口跟随 base；此视角与发布深度图的机载相机独立。
                with self.viewer.lock():
                    self.viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
                    self.viewer.cam.lookat[:] = self.robot.data.xpos[self.robot.base_id]
                    self.viewer.cam.distance = camera_distance
                    self.viewer.cam.azimuth = 135.0
                    self.viewer.cam.elevation = -20.0
        command_topic = self.declare_parameter("motor_command_topic", "/lowcmd").value
        state_topic = self.declare_parameter("motor_state_topic", "/lowstate").value
        self.depth_topic = self.declare_parameter("depth_topic", "/camera/depth/image_raw").value
        self.info_topic = self.declare_parameter("camera_info_topic", "/camera/depth/camera_info").value
        self.state_pub = self.create_publisher(LowState, state_topic, qos_profile_sensor_data)
        self.command_sub = self.create_subscription(LowCmd, command_topic, self.command_callback, 1)
        if self.declare_parameter("enable_depth", False).value:
            self.setup_depth()
        self.timer = self.create_timer(period, self.update)
        self.get_logger().info("Go2W MuJoCo loaded: " + path)
        self.get_logger().info("ROS2 motor topics: subscribe " + command_topic + ", publish " + state_topic)

    def command_callback(self, message):
        # 此桥固定使用 LowCmd/LowState 前 16 个槽位；默认 motor_mapping 为恒等映射。
        # 控制器若配置了非恒等映射，需要驱动同步支持，不能只改控制器的一侧。
        command = np.asarray([[m.mode, m.q, m.dq, m.kp, m.kd, m.tau]
                              for m in message.motor_cmd[:16]], dtype=float)
        if not np.isfinite(command).all():
            self.command = None
            return
        self.command = command
        self.command_time = time.monotonic()
        if np.any(command[:, 3] > 0):
            self.waiting = False

    def update(self):
        # wall timer 每次推进一个固定物理步；当前没有发布 /clock，也没有追赶丢失的步。
        # 渲染/推理变慢会影响实时率，改变 simulation_period 不能替代恢复调度性能。
        if not self.waiting:
            command = self.command if time.monotonic() - self.command_time < self.command_timeout else None
            self.robot.step(command)
        position, velocity, torque = self.robot.joint_state()
        quaternion, gyro, acceleration = self.robot.imu_state()
        state = LowState()
        state.head = [0xFE, 0xEF]
        state.level_flag = 0xFF
        state.tick = int(self.robot.data.time * 1000) % (2 ** 32)
        for i in range(16):
            motor = state.motor_state[i]
            motor.mode = 1
            motor.q, motor.dq, motor.tau_est = float(position[i]), float(velocity[i]), float(torque[i])
        state.imu_state.quaternion = quaternion.astype(np.float32).tolist()
        state.imu_state.gyroscope = gyro.astype(np.float32).tolist()
        state.imu_state.accelerometer = acceleration.astype(np.float32).tolist()
        self.state_pub.publish(state)
        self.viewer_step += 1
        if self.viewer is not None and self.viewer_step % 6 == 0:
            if self.viewer.is_running():
                if self.follow_camera:
                    # 仅更新注视点；保留鼠标调整的角度和缩放，不跟随机身翻滚。
                    with self.viewer.lock():
                        self.viewer.cam.lookat[:] = self.robot.data.xpos[self.robot.base_id]
                self.viewer.sync()
            else:
                self.timer.cancel()

    def setup_depth(self):
        # 必须由 XML 提供与训练安装方式一致的机载相机；场地观察相机不等于机载相机。
        # 发布图像和内参，不发布 base -> camera 的 TF；外参需由接入者另行提供。
        self.camera_name = self.declare_parameter("depth_camera", "front_depth").value
        camera_id = mujoco.mj_name2id(self.robot.model, mujoco.mjtObj.mjOBJ_CAMERA, self.camera_name)
        if camera_id < 0:
            raise ValueError("enable_depth needs a named MJCF camera: " + self.camera_name)
        self.width = self.declare_parameter("depth_width", 320).value
        self.height = self.declare_parameter("depth_height", 240).value
        self.renderer = mujoco.Renderer(self.robot.model, height=self.height, width=self.width)
        self.renderer.enable_depth_rendering()
        self.depth_pub = self.create_publisher(Image, self.depth_topic, qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, self.info_topic, qos_profile_sensor_data)
        fovy = self.robot.model.cam_fovy[camera_id] * np.pi / 180
        self.focal = self.height / (2 * np.tan(fovy / 2))
        self.depth_timer = self.create_timer(1.0 / 15.0, self.publish_depth)

    def publish_depth(self):
        # 原始深度 320x240（默认）、32FC1 米、15 Hz；未执行模型需要的裁剪/缩放。
        # 图像 stamp 是渲染发布的 ROS 时间，当前不做相机与 RobotState 的同步。
        self.renderer.update_scene(self.robot.data, camera=self.camera_name)
        depth = self.renderer.render().astype("<f4")
        image = Image()
        image.header.stamp = self.get_clock().now().to_msg()
        # 此 frame_id 当前固定；更换命名相机时需同时核对 frame_id 和外参。
        image.header.frame_id = "front_depth_optical"
        image.height, image.width = self.height, self.width
        image.encoding = "32FC1"  # 米，未把 RGB 或空白图像当作深度。
        image.is_bigendian = False
        image.step, image.data = self.width * 4, depth.tobytes()
        info = CameraInfo()
        info.header = image.header
        info.height, info.width = self.height, self.width
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        f, cx, cy = float(self.focal), (self.width - 1) / 2, (self.height - 1) / 2
        info.k = [f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [f, 0.0, cx, 0.0, 0.0, f, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.depth_pub.publish(image)
        self.info_pub.publish(info)

    def destroy_node(self):
        if self.viewer is not None:
            self.viewer.close()
        if self.renderer is not None:
            self.renderer.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Go2WMujoco()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
