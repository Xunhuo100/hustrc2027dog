# HUSTDOG v11 MuJoCo 场景

完整地图入口为 `urdf/scene_svproj.xml`，引用 `../../terrains/svproj_course.xml`（相对于入口文件所在目录）。该入口从 `urdf/hustdog_v11.xml` 生成，已将网格目录修正为 `../assets`，去掉源文件自带的地面与灯光，并将浮动基座及 home keyframe 的 XY 出生位置设为 `(0.5,0.5)`。机器人原有零位姿态、惯性和被动关节约束予以保留；执行器已改为与 ROS 后端匹配的力矩执行器。

在工作空间根目录执行：

```bash
.venv/bin/python -m mujoco.viewer \
  --mjcf=src/wheel_leg_mujoco/models/hustdog_v11/urdf/scene_svproj.xml
```

本入口用于 MuJoCo 模型和地图预览，没有给 HUSTDOG 添加运动策略。当前 home 姿态是基座贴地的原始零位预览姿态，不是站立姿态。

主动关节名与基座名称已和现有 ROS 后端统一；ROS 启动已配置贴地的趴下初始姿态和单独的 Stand 目标；策略的观测动作映射仍需适配，Go2W 已训练策略不能直接复用。完整地图说明见 `../terrains/README.md`。

修改机器人源模型或重新导出地图后，执行 `python3 tools/build_workspace_map.py` 更新生成场景，再重新构建 ROS 包以更新安装资源。

## 关节命名

以当前模型坐标的 +X 为前、+Y 为左，根据髋关节安装位置统一四条腿的前缀：

| 原前缀 | 新前缀 | 腿的位置 |
| --- | --- | --- |
| `left_3` | `FL` | 左前，X > 0、Y > 0 |
| `right_4` | `FR` | 右前，X > 0、Y < 0 |
| `left_4` | `RL` | 左后，X < 0、Y > 0 |
| `right_3` | `RR` | 右后，X < 0、Y < 0 |

16 个主动关节采用 `FL/FR/RL/RR` + `_hip_joint`、`_thigh_joint`、`_calf_joint`、`_foot_joint`。
其中 `calf` 对应原来的主动 `crank` 曲柄关节，`foot` 对应原来的 `wheel` 轮关节。
8 个被动 `shank/rod` 关节保留原名；执行器名称、执行器关节引用以及 equality 引用同步更新。
浮动根机体从 `base_link` 更名为 `base`，`root_angmom` 传感器的机体引用同步更新。

此次修改仅统一名称，关节轴、限位、惯量、闭环约束参数、关节和执行器排列均保持原值。
执行器现已改为力矩控制，具体约定见下节。
`FL/FR/RL/RR` 的命名完成不代表原 Go2W 的 PD 参数、默认站姿或 `parkour.pt` 已适配 HUSTDOG。

## 力矩执行器

`hustdog_v11.xml` 和完整地图入口 `scene_svproj.xml` 的 16 个执行器均使用 `<motor gear="1">`。
对于这些转动关节，`ctrl` 直接表示关节力矩（N·m）；XML 不再叠加位置伺服，PD 由 `mujoco_backend.py` 计算。
12 个腿部电机的 `ctrlrange` 和 `forcerange` 均为 `-23.7 23.7`，4 个轮电机均为 `-5 5`，沿用原模型的力矩上限。
旧位置执行器的 `kp="50"` 已移除，原角度 `ctrlrange` 已替换为上述力矩范围；关节角度限制仍由各 `<joint range="...">` 定义。

ROS 趴下初始姿态、Stand 目标和被动连杆初始位置已按下节处理。Go2W 策略的关节零位、方向和观测动作映射仍未适配，不能据此认为 HUSTDOG 已能用该策略行走。

## ROS 初始姿态与连杆闭合

`launch/hustdog_sim.launch.py` 使用 `config/hustdog.yaml`，仿真先以类似 Go2W 的折腿趴下姿态启动。
该入口默认 `autostart:=false`，启动后等待模式指令，不会自动站立或切入 RL；策略节点可以加载但不会接管。
初始左腿的 `hip/thigh/calf` 为 `[0, -0.4, 1.5]`，右腿为 `[0, 0.4, -1.5]`，轮关节为零。
基座初始高度为 `0.10 m`，机身底面离地约 `7.25 mm`；主动和被动关节初始角度均在限位内。

控制器 Stand 目标单独保留：左腿 `[0.663225, -1.55, 1.2]`，右腿符号相反，轮关节为零。
请求 Stand 时，控制器从收到反馈的趴下角度出发，在 `1.5 s` 内插值到站立目标。
目前这条简单关节插值轨迹在测试中触发了 60 度倾斜保护，尚不能作为稳定起身动作。
站立目标中的髋关节 38 度补偿抵消 CAD 装配倾角。

`mujoco_backend.py` 在写入主动关节姿态后，按启用的 joint equality 及各关节参考角初始化被动关节：
小腿与曲柄同角度，连杆与曲柄反角度。初始化不依赖物理步推进，避免等待控制器时连杆仍停在零位。
运行中由 MuJoCo equality 求解约束；两个 HUSTDOG XML 使用 `implicitfast`，联动约束设置为
`solref="0.005 1"`、`solimp="0.999 0.999 0.001"`。
HUSTDOG 的 5 ms 控制周期包含 5 个 1 ms 物理子步；Go2W 默认仍为每周期一个物理步。

启动并观察趴姿：

```bash
source install/setup.bash
ros2 launch launch/hustdog_sim.launch.py
```

窗口初始显示趴下姿态，等待 Stand 指令后才开始推进物理仿真。另一个已加载工作空间环境的终端执行：

```bash
ros2 topic pub --once /robot/mode std_msgs/msg/String "{data: stand}"
```

运行 `.venv/bin/python tests/test_mujoco_initialization.py` 检查平地和完整地图的启动角度、连杆闭合、
5 秒被动趴姿的高度与倾角、关节运动时的连杆闭合及 Go2W 默认行为。
连杆运动检查只验证闭合与有限数值，不表示站立轨迹或旧策略已适配。

