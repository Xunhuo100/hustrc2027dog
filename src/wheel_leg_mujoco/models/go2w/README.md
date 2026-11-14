# Go2W 仿真资源

本目录包含当前 ROS2 MuJoCo 仿真运行所需的机器人、场景及网格，随 `wheel_leg_mujoco` 包安装。
运行不需要另外克隆或启动外部仿真工程。

## 文件

- `urdf/go2w.xml`：16 电机轮腿机器人 MJCF，包括惯量、关节、执行器等参数。
- `urdf/scene_flat.xml`：平地；仿真节点和 launch 的默认场景。
- `urdf/scene_fields.xml`：障碍场地；根目录 `start_sim.sh` 的默认场景。
- `urdf/scene.xml`：原有基础场景。
- `dae/`：MJCF 实际引用的机器人及障碍网格。目录名保留原路径，内容为 OBJ/STL。
- `LICENSE`：原工程 BSD 3-Clause 许可。

MJCF 的 include 和 meshdir 使用相对路径，移动整个 `go2w` 目录仍可加载。
模型文件及网格保持导入时的原始内容；电机 PD、力矩限制、惯量和碰撞参数没有因迁移改变。

## 使用和修改

根目录执行 `./start_sim.sh` 即可启动障碍场景。
设置 `MODEL_XML=/absolute/path/scene.xml ./start_sim.sh` 可选择自己的场景。
直接通过 ROS2 启动时也可设置 `model_xml:=/absolute/path/scene.xml`。

`colcon build --packages-up-to wheel_leg_mujoco` 会把资源安装到
`share/wheel_leg_mujoco/models/go2w`。仿真 launch 和单独运行的 `go2w_mujoco` 都通过
ROS2 包索引查找安装资源，不依赖当前工作目录。
普通构建会复制资源，因此修改模型后应重新构建，以更新安装副本。

障碍位置在 `urdf/scene_fields.xml` 中修改；机器人参数在 `urdf/go2w.xml` 中修改。
深度发布默认关闭；当前场景只包含场地观察相机，没有名为 `front_depth` 的机载相机。
接入视觉策略前需要按训练配置补充机载相机。

## 来源

2026-11-14 从本机 `dogMujoco2027/unitree_robots/go2_w` 导入。
只收录 4 个 MJCF 和它们引用的 305 个网格，不包含 SDK 桥接、仿真可执行程序、CAD 转换脚本或预览图。
原工程根目录 `LICENSE` 原样保存在本目录；来源名称仅用于记录，不是运行路径。
