# 2027 轮腿控制框架

当前控制对象为 Go2W 的 **16 个电机：12 个腿关节 + 4 个轮电机**。
ROS2 Humble，C++ 控制器，Python TorchScript 推理，默认 MuJoCo 仿真。

核心实现、外部盲走/视觉策略接入、IMU 坐标与单位、电机 PD 及控制周期的详细笔记：
[控制框架与策略接入说明](docs/control_and_policy_integration.md)。
控制层可复用统一电机接口接入不同策略；当前自带推理节点专用于 HIM Go2W，
视觉推理与运行中多策略选择仍需要额外适配。

## 代码结构

- `src/wheel_leg_interfaces`：固定 16 电机的 `RobotState` / `RobotCommand`。
- `src/robot_ctrl`：200 Hz 底层收发与 Passive / Stand / RL 状态机。
- `src/himloco_sim2sim`：50 Hz HIMLoco 推理、模型导出脚本和 MuJoCo ROS2 桥。
- `src/robot_ctrl/legacy`：原来 12 自由度框架的完整源文件和构建配置，不参与编译。

参考本机 `2026dogs/12fdog/unitree_him_w` 的 RL 状态收发方式；
模型和观测来自 `_PaperWorks/HIMLoco`，机器人来自
`dogMujoco2027/unitree_robots/go2_w/urdf`。
ROS2 节点采用类似 `Robocon/2026/Volleyball` 的直接回调和普通成员函数写法。

## 构建和启动

```bash
cd ~/hustrc2027dog
source /opt/ros/humble/setup.bash
colcon build --packages-up-to robot_ctrl himloco_sim2sim --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

Python 节点需使用 **Python 3.10**，与 Humble 的 rclpy 一致。
本工作空间已配置 `.venv` 作为验证环境。新机器可以这样准备：

```bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install torch==2.5.1 --index-url https://download.pytorch.org/whl/cpu
python -m pip install 'numpy<2' mujoco==3.2.7
```

若系统提示缺少 ensurepip，先安装发行版的 `python3.10-venv`。
保留 `.venv/bin` 在 PATH 前面，因为 Python 可执行脚本使用 `#!/usr/bin/env python3`。
已有的 HIMLoco Python 3.8 conda 环境可以导出模型，但不能运行 Humble 的 Python 节点。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
source .venv/bin/activate
ros2 launch himloco_sim2sim sim2sim.launch.py
```

启动文件默认使用本机最新 parkour 导出权重，已核对与 `model_13500.pt` 的 actor 和 estimator
全部参数一致；`policy_profile:=parkour` 同时选择权重、Stand 站姿、策略 PD 和仿真初始姿态。
原 flat 配置可用 `policy_profile:=flat` 显式选择。直接运行旧的独立节点入口仍使用原 flat 默认值，
建议通过 launch 统一选择配置。启动后等待模型加载完成，自动执行
Stand 插值，再进入 RL；初始速度为零。默认无窗口；显示仿真加 `viewer:=true`。
MuJoCo 在首次收到站立位置指令前保持初始姿态，避免加载模型期间机器人掉落。
进入控制后发生断连会继续物理仿真并施加阻尼。

```bash
# 持续发送机体坐标系下的 vx、vy、wz；停止发送 0.5 秒后目标速度归零。
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.3, y: 0.0}, angular: {z: 0.0}}'

# 指定其他导出模型或场景。
ros2 launch himloco_sim2sim sim2sim.launch.py \
  policy_profile:=parkour model_path:=/absolute/path/policy.pt model_xml:=/absolute/path/scene.xml
```

控制器和仿真器通过 ROS2 topic 收发 `LowCmd` / `LowState`。
`start_simulator:=false` 时只启动控制器、策略和默认关闭的键盘节点，
由外部 ROS2 仿真节点或真机驱动提供相同消息接口。

## 可选键盘控制

`/keyboard_control` 节点的 ROS2 参数 `enable_keyboard` **默认 false**。
关闭时不读取键盘、不发布 `/cmd_vel`，可继续由其他节点发送速度。
启动参数会传入同名 ROS2 参数：

```bash
# 内置 MuJoCo，启动终端接收按键。
ros2 launch himloco_sim2sim sim2sim.launch.py enable_keyboard:=true viewer:=true

# 如需提高前进/侧移按键的速度，在启动时设置，单位 m/s。
ros2 launch himloco_sim2sim dog_mujoco.launch.py \
  enable_keyboard:=true keyboard_linear_speed:=0.6 viewer:=true

# 单独运行键盘节点，也必须显式启用。
ros2 run himloco_sim2sim keyboard_control --ros-args -p enable_keyboard:=true

# 运行中切换；终端与节点需处于同一 ROS_DOMAIN_ID 和 RMW。
ros2 param set /keyboard_control enable_keyboard true
ros2 param set /keyboard_control enable_keyboard false
```

| 按键 | 期望速度（机体坐标系） |
| --- | --- |
| ↑ / ↓ | `linear.x`：前进 / 后退，默认 ±0.3 m/s |
| ← / → | `linear.y`：左移 / 右移，默认 ±0.3 m/s |
| `,` / `.` | `angular.z`：左转 / 右转，默认 ±0.6 rad/s |
| 空格 | vx、vy、wz 立即归零 |

输出为 `/cmd_vel` 的 `geometry_msgs/msg/Twist`，默认 20 Hz，并在速度变化时打印 vx、vy、wz。
参数位于 `src/himloco_sim2sim/config/keyboard.yaml`；速度和周期在启动时配置。
长按方向键维持运动，可组合平移和转向；终端不提供松键事件，每个方向默认 0.5 秒无重复输入归零。
使用启动该节点的终端输入，Ctrl+C 退出时恢复终端设置。
`ros2 launch` 下直接读取 `/dev/tty`；无终端环境保持默认关闭。
启用键盘时由它持续发布期望速度，外部速度发布者应先停用。

## ROS2 仿真与真机切换

`dog_mujoco.launch.py` 使用本包的 ROS2 MuJoCo 节点，直接加载
`dogMujoco2027/unitree_robots/go2_w/urdf/scene_flat.xml`，默认使用 `/sim` 下的电机和相机话题。
它复用参考工程的模型和场景；运行入口是 `go2w_mujoco`，不启动参考工程的原生 DDS 可执行程序。
仿真桥直接从 MuJoCo 读取关节和 IMU 数据，不要求原始 XML 额外添加传感器。

```bash
# 仿真：控制器 + HIMLoco + ROS2 MuJoCo 桥，可选键盘。
ros2 launch himloco_sim2sim dog_mujoco.launch.py enable_keyboard:=true viewer:=true

# 真机或外部 ROS2 仿真：保留同一控制器和策略，只切换 topic 并关闭内置仿真。
# 以下 /hardware 话题名是示例，替换为实际驱动话题。
ros2 launch himloco_sim2sim dog_mujoco.launch.py \
  start_simulator:=false autostart:=false \
  motor_command_topic:=/hardware/lowcmd \
  motor_state_topic:=/hardware/lowstate \
  depth_topic:=/hardware/camera/depth/image_raw \
  camera_info_topic:=/hardware/camera/depth/camera_info
```

| 启动参数 / ROS2 参数 | dog_mujoco 默认 topic | 控制器方向 | 消息类型 |
| --- | --- | --- | --- |
| `motor_command_topic` | `/sim/lowcmd` | 发布电机命令；驱动订阅 | `unitree_go/msg/LowCmd` |
| `motor_state_topic` | `/sim/lowstate` | 订阅电机与 IMU；驱动发布 | `unitree_go/msg/LowState` |
| `depth_topic` | `/sim/camera/depth/image_raw` | 订阅深度；相机驱动发布 | `sensor_msgs/msg/Image` |
| `camera_info_topic` | `/sim/camera/depth/camera_info` | 订阅内参；相机驱动发布 | `sensor_msgs/msg/CameraInfo` |
| `velocity_topic` | `/cmd_vel` | 策略订阅；键盘或导航发布 | `geometry_msgs/msg/Twist` |

`sim2sim.launch.py` 和单独的 `robot_ctrl.launch.py` 也暴露电机和相机 topic 参数，
默认电机话题为 `/lowcmd` / `/lowstate`。在一处指定后，启动文件会把相同 topic
传给控制器和仿真桥；`velocity_topic` 同时传给策略和键盘。
直接运行节点也可通过 `--ros-args -p motor_command_topic:=... -p motor_state_topic:=...` 配置。
这些 topic 参数在启动时设置，切换后重启节点。

实机驱动需要提供同样的消息类型、单位与电机顺序；不同槽位用 `go2w.yaml` 的
`motor_mapping` 配置，不同协议在硬件驱动层转换。IMU 随 LowState 返回，
策略继续使用 `/robot/state`，控制状态和模型代码不需要修改。
如果已有外部 ROS2 仿真节点，也使用 `start_simulator:=false` 并填写它的收发 topic。

参考工程的障碍场景也可以直接通过 `model_xml` 加载，无需修改控制器或话题：

```bash
ros2 launch himloco_sim2sim dog_mujoco.launch.py \
  model_xml:="$HOME/dogMujoco2027/unitree_robots/go2_w/urdf/scene_fields.xml" \
  enable_keyboard:=true viewer:=true
```

此场景包含高墙、单木桥、轮胎阵等 8 组固定障碍，机器人从原点附近开始，第一处障碍位于
世界坐标 X=3 m。场景仍使用同一 Go2W 模型的 16 个执行器，电机、IMU 和速度接口保持一致。
已验证最新版 parkour 的左右侧移，以及以 vx=1.0 m/s 从中央通过首个 0.3 m 高墙。
这项高墙检查使用固定步长的策略/物理闭环；ROS2 完整链路另行验证了前进和左右侧移。
vx=0.3 m/s 时仍可能停在墙前，不能把键盘默认慢速下的结果视为任意速度均可越障。
未验证整条场地所有障碍；详细定位和量化结果见 `docs/parkour_deployment_check.md`。
当前策略不读取深度图，也不包含障碍识别或路径规划。
原场景只有 `course_overview`、`course_top` 两个场地相机，没有机载 `front_depth`，
因此保持 `enable_depth:=false`；如需机载深度，先在机器人 base 下配置对应相机。

## 电机顺序与控制方式

所有内部消息、默认策略和内置仿真器使用以下固定顺序：

| 索引 | 腿 | 关节顺序 |
| --- | --- | --- |
| 0–3 | FL 左前 | hip, thigh, calf, wheel |
| 4–7 | FR 右前 | hip, thigh, calf, wheel |
| 8–11 | RL 左后 | hip, thigh, calf, wheel |
| 12–15 | RR 右后 | hip, thigh, calf, wheel |

腿输出 `q_target = q_default + 0.25 * action`，`dq=0`；parkour 的 `kp=40, kd=1.0`。
轮输出 `dq_target = 10 * action` rad/s，`q=0, kp=0`；parkour 的 `kd=0.5`。
对应训练站姿为四条腿 hip=0、thigh=0.8、calf=-1.5。
flat 仍为原来的站姿、腿 `kp=20, kd=0.5`、轮 `kd=0.3`。
前馈 `tau=0`。仿真器计算 `tau + kp*(q_target-q) + kd*(dq_target-dq)` 并按 XML 力矩范围限幅。
腿目标角按 XML 关节范围限幅，轮速限制为 ±100 rad/s；训练 action 限幅保留为 ±100。

`robot_ctrl/config/go2w.yaml` 的 `motor_mapping[i]` 表示内部第 i 个电机对应的
LowCmd/LowState 槽位。当前 XML 和现有 MuJoCo 桥都是 FL/FR/RL/RR 交错顺序，默认恒等映射。
接硬件时在此配置实际电机编号和驱动话题；同一映射同时用于输出和反馈。
框架保留 Unitree 20 槽报文及 CRC，未映射槽位的 mode 恒为 0。
当前没有实现自定义 CAN/串口驱动；如果底层协议不同，在下面的 IO 入口完成适配。

## 电机、IMU、深度相机接口

| 功能 | ROS2 接口 | 代码入口 |
| --- | --- | --- |
| 输出电机指令 | `/lowcmd`，`unitree_go/msg/LowCmd` | `RobotIO::sendMotorCommand()` |
| 接收电机反馈 | `/lowstate`，`unitree_go/msg/LowState` | `RobotIO::motorStateCallback()` |
| 获取 IMU | LowState 内的 `imu_state` | 同一回调转换为 `getState().imu` |
| 统一状态输出 | `/robot/state`，`wheel_leg_interfaces/msg/RobotState` | `RobotIO::publishState()` |
| IMU 单独输出 | `/robot/imu`，`sensor_msgs/msg/Imu` | `RobotIO::publishState()` |
| RL 目标输入 | `/robot/rl_cmd`，`wheel_leg_interfaces/msg/RobotCommand` | `RobotController::policyCallback()` → `State_RL::receive()` |
| 深度图输入 | `/camera/depth/image_raw`，`sensor_msgs/msg/Image` | `RobotIO::depthCallback()` / `getDepthImage()` |
| 深度相机内参 | `/camera/depth/camera_info`，`sensor_msgs/msg/CameraInfo` | `RobotIO::cameraInfoCallback()` / `getDepthCameraInfo()` |
| 速度命令 | `/cmd_vel`，`geometry_msgs/msg/Twist` | `HimLocoPolicy.velocity_callback()` |
| 状态切换 | `/robot/mode`，`std_msgs/msg/String` | `RobotController::modeCallback()` |

实现位置为 `src/robot_ctrl/include/interface/RobotIO.h` 与 `src/robot_ctrl/src/interface/RobotIO.cpp`。
`getState().motor_state[i]` 提供位置 rad、速度 rad/s、估计力矩 N·m。
Unitree IMU 原始四元数为 **w,x,y,z**，转换到 ROS 消息的具名 x/y/z/w 字段后再发布；
角速度 rad/s、加速度 m/s²，均在机体坐标系。内置 MuJoCo 桥输出加速度计比力，自由落体为零。
仿真角速度和加速度使用 `mjOBJ_XBODY` 的 base 坐标系；不能用 `mjOBJ_BODY` 的惯性主轴
局部坐标，否则会与姿态四元数的轴不一致。策略角速度缩放保持训练值 0.25。

深度图保留驱动原始 encoding、step、时间戳和字节序；常见 `16UC1` 单位由驱动约定，
`32FC1` 通常为米。使用前检查指针、时间戳并按 encoding 解码。
接口只缓存最新图像与内参，当前 HIMLoco 是本体感知模型，没有把深度图拼进 57 维观测。
实机启动相机驱动并配置对应话题即可。内置 MuJoCo 桥可在 XML 已有名为 `front_depth`
的相机时使用 `enable_depth:=true`，输出 32FC1 米制深度与相机内参；默认场景没有该相机，
因此默认关闭。深度参数可通过单独运行桥的 `--ros-args -p enable_depth:=true` 设置。

## RL 状态与保护

Passive 对腿施加阻尼、轮目标速度为零；Stand 用 1.5 秒从反馈姿态插值到默认站姿；
只有 Stand 完成且反馈有效后才接受 RL 状态切换。
RL 首帧命令到达前保持站姿，进入 RL 后 0.2 秒未收到有效指令就退回 Passive。
模型推理节点退出也会触发这个超时。
NaN/Inf、非法增益、超过指令范围、过期时间戳或不匹配的 control_session 会拒绝指令并退回 Passive。
反馈超时 0.1 秒或 IMU 无效时停用输出；倾角超过 60° 退回 Passive。
每次切换状态递增会话号，策略历史及上一次 action 随会话清零。
自动启动只执行一次，故障退出后需要手动重新站立。

```bash
# 单独启动控制器，初始为 Passive。
ros2 launch robot_ctrl robot_ctrl.launch.py
ros2 topic pub --once /robot/mode std_msgs/msg/String '{data: stand}'
# 等待站立插值结束，再进入 RL；策略节点需已启动。
ros2 topic pub --once /robot/mode std_msgs/msg/String '{data: rl}'
ros2 topic pub --once /robot/mode std_msgs/msg/String '{data: passive}'
```

控制器进程本身退出后，需要底层接收端实施独立命令看门狗；内置 MuJoCo 桥已实现 0.1 秒超时阻尼。

## HIMLoco 模型约定与导出

`himloco_sim2sim/policy_core.py` 对照本机 HIMLoco 的 `him_go2w_env.py` 构建每帧观测：

| 区间 | 内容 | 缩放 |
| --- | --- | --- |
| 0:3 | vx, vy, wz 命令 | 2, 2, 0.25 |
| 3:6 | IMU 角速度 | 0.25 |
| 6:9 | 重力方向在机体坐标系中的投影 | 单位重力，水平为 [0,0,-1] |
| 9:25 | 16 关节相对默认角度 | 1；4 个轮角强制为零 |
| 25:41 | 16 关节速度 | 0.05 |
| 41:57 | 上一次裁剪后的 action | 1 |

共 57 维，6 帧历史拼为 `[1,342]`，**最新帧在前**，初始历史其余帧为零。
策略必须是 estimator + actor 一起导出的 TorchScript，输出 `[1,16]`。
只导出 actor 会丢失估计速度和 latent，不能直接部署。

```bash
# 可以在已有的 HIMLoco Python 3.8 环境或当前 .venv 中执行。
python src/himloco_sim2sim/scripts/export_policy \
  --himloco-root "$HOME/_PaperWorks/HIMLoco" \
  --checkpoint /absolute/path/model_2000.pt \
  --output /absolute/path/policy.pt
```

导出脚本直接加载训练端 `HIMActorCritic`，严格加载权重，并比较训练推理与导出结果。
替换模型时一并核对站姿、PD、缩放和训练周期；rough/obstacle 的配置不能直接套用 flat 默认参数。
示例权重的来源和原项目许可保存在 `policies/SOURCE.md`、`policies/LICENSE_HIMLoco.txt`。

## 验证

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
source .venv/bin/activate
python -m pytest -q tests/test_policy_core.py
python tests/check_parkour.py --fields
ROS_DOMAIN_ID=74 python tests/check_controller.py
ROS_DOMAIN_ID=73 python tests/check_sim2sim.py
ROS_DOMAIN_ID=78 python tests/check_sim2sim.py \
  --model-xml "$HOME/dogMujoco2027/unitree_robots/go2_w/urdf/scene_fields.xml"
ROS_DOMAIN_ID=75 python tests/check_keyboard.py
```

前者检查观测布局、轮角屏蔽、四元数方向、历史帧顺序、动作分流、非有限值、真实 XML 和权重。
还检查非零姿态下 IMU 轴向，以及 parkour 权重的全部推理参数与训练 checkpoint 完全相同。
`check_parkour.py` 使用 0.005 s 物理步长和 0.02 s 策略周期，量化前进、左右侧移与首个高墙接触。
`check_controller.py` 检查非恒等映射、会话号、非法动作和超时。
`check_sim2sim.py` 启动三个真实进程，验证进入 RL、速度输入下 6 秒闭环、策略退出回 Passive、反馈中断禁用全部槽位。
进程日志位于 `/tmp/hustrc2027dog_{controller,policy,mujoco}.log`。

`check_keyboard.py` 使用真实伪终端、ROS2 参数服务和 Twist 订阅，检查默认关闭、全部按键、
分段方向键序列、组合运动、超时归零、运行时开关和退出恢复终端。
`check_ros_topics.py` 启动完整 launch，验证 `/sim` 和外部驱动话题两种接线，
检查默认键盘关闭、运行时启用、期望速度和真实 MuJoCo 电机反馈闭环。

```bash
python tests/check_ros_topics.py
```
