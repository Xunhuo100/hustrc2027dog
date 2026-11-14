# Parkour 部署检查（2026-11-12）

## 已定位问题

原默认 `policy.pt` 是 flat 权重，SHA256 前 16 位为 `c2dd4935389fafaa`。
最新版 parkour 导出权重的 SHA256 前 16 位为 `925945b770662439`，actor 和 estimator 全部参数
均与 `_PaperWorks/HIMLoco/legged_gym/logs/him_go2w_parkour/Oct24_15-45-13_/model_13500.pt` 一致。
当前已单独保存为 `parkour.pt`，启动文件默认选择 `policy_profile:=parkour`。
原 flat 权重保留在 `flat.pt` 和兼容旧入口的 `policy.pt`。

明确的仿真错误位于 IMU 读取：`mj_objectVelocity(..., mjOBJ_BODY, ..., local=1)` 使用惯性主轴，
而姿态使用 `data.xquat[base]` 的机体坐标系。Go2W 的 base 含有非对角惯量，主轴与 base 轴不重合。
例如设置机体角速度 `[0.7, -0.4, 0.2]` rad/s 和四元数 `[0.5, 0.5, 0.5, 0.5]`，
旧代码输出约 `[0.21385, 0.39815, 0.69695]`，新代码用 `mjOBJ_XBODY` 后输出与设置值一致。
加速度读取也同步改为 XBODY，保持坐标一致；当前策略不读取加速度，直接影响推理的是陀螺仪轴错误。

## 与训练端对照

直接用 Isaac Gym 加载训练 URDF，并核对了实际 DOF 顺序和有效参数。
关节顺序仍为 FL、FR、RL、RR，每条腿 hip、thigh、calf、foot；没有发生关节错位。
训练与当前部署使用以下 parkour 配置：

| 项目 | Parkour | 原 Flat |
| --- | --- | --- |
| 默认 hip | 0 | 左 +0.1、右 -0.1 |
| 默认 thigh | 四腿 0.8 | 前腿 0.8、后腿 1.0 |
| 默认 calf | -1.5 | -1.5 |
| 腿 kp / kd | 40 / 1.0 | 20 / 0.5 |
| 轮 kp / kd | 0 / 0.5 | 0 / 0.3 |
| 腿 action 缩放 | 0.25 | 0.25 |
| 轮 action 缩放 | 10 rad/s | 10 rad/s |
| 角速度观测缩放 | 0.25 | 0.25 |
| 物理步长 / 策略周期 | 0.005 / 0.02 s | 0.005 / 0.02 s |

本次将权重、Stand、策略参数和仿真初始站姿放在同一个 profile 下选择。
没有通过降低 IMU 缩放或任意降低 PD 来掩盖问题。
MuJoCo XML 与 Isaac Gym 在关节被动阻力、接触模型上仍有差别；本次保留了原 XML 动力学，
通过坐标修正和配置配套已消除复现中的明显失稳。

## 量化复现

固定步长、单线程 CPU 推理，先保持默认站姿 1.5 s，再执行各方向 12 s。
RMS 和峰值统计后 10 s，速度为机体坐标系平均速度。

| 配置 | 指令 | 结果 |
| --- | --- | --- |
| 旧 IMU + flat 原配置 | vy=+0.3 | 约 3.94 s 横滚超过 60° |
| 旧 IMU + parkour 匹配配置 | vy=+0.3 | 约 2.66 s 倾角超过 60° |
| 修正 IMU + parkour 匹配配置 | vx=+0.3 | 12 s 完成，横滚/俯仰 RMS 2.01°/0.34° |
| 修正 IMU + parkour 匹配配置 | vy=+0.3 | 12 s 完成，横滚/俯仰 RMS 0.80°/2.93°，峰值 2.14°/4.19° |
| 修正 IMU + parkour 匹配配置 | vy=-0.3 | 12 s 完成，横滚/俯仰 RMS 3.48°/0.99°，峰值 5.54°/1.57° |

左右侧移的平均机体速度分别约为 `[0.029, 0.309, -0.002]`、`[0.081, -0.281, -0.015]` m/s，
仍有速度偏差和前向漂移。前进指令 vx=0.3 时平均约 0.456 m/s；在单环境 Isaac Gym 对照中也观察到
前进速度高于目标，不能承诺两套物理引擎的轨迹完全一致。

原 `scene_fields.xml` 首个高墙厚 0.08 m、宽 0.8 m、高 0.3 m，中心 X=3 m。
修正后的 parkour 在 vx=1.0 下有真实高墙碰撞，并从中央通过：
机体经过 X=3 时约为 `[3.003, -0.066, 0.443]`，4.6 s 到达 `[4.007, -0.024, 0.391]`。
检查包含 y 方向约束，避免把绕墙误判为越墙。
同场景 vx=0.3 下运行 30 s 仍停在 X≈2.63 m，因此越障表现依赖速度和障碍条件。
这不是整条场地通关验证，也不包含深度感知或规划。

复现：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
source .venv/bin/activate
python -m pytest -q tests/test_policy_core.py
python tests/check_parkour.py --fields
python tests/check_ros_topics.py
```

最后一项独立验证真实 ROS2 launch、键盘期望速度和电机闭环，包括左右平移姿态检查。
固定步长的高墙测试用于验证策略与物理接触；尚未验证 ROS2 调度下整条障碍场地通关。
