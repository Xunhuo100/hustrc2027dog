# HUSTRC_2027_DOG
2027年狗子的主控代码

## 结构
修改自unitree_guide的FSM结构，将其底层机制从ROS1+GAZEBO修改为ROS2，与控制逻辑和仿真解耦

底层控制在'src/robot_ctrl/src/interface/RobotIO.cpp',向'motor_command_topic'发布消息获取指令，向'motor_state_topic'读取电机反馈数据和IMU数据等，深度相机暂时没加

主要的代码流程是：main启动，进入passive状态，

## 当前内容


## 使用
MUJOCO通讯是来自宇树的接口的，需要有另外的mujoco工作空间（如果可以简化最好）

目前是用键盘控制，用键盘输出期望的速度，狗子跟踪之



## 后续
把自动决策、雷达定位、视觉等工作一点一点移植进来，不急但尽可能快

## 仿真相机跟随

`start_sim.sh` 默认开启观察视角跟随，注视点跟随机器人机身，初始观察距离为 3 m；可用鼠标旋转、缩放，视角不随机身翻滚。该功能只控制 MuJoCo 观察窗口。

```bash
./start_sim.sh                               # 默认跟随狗子
FOLLOW_CAMERA=false ./start_sim.sh           # 关闭跟随，自由观察场地
CAMERA_DISTANCE=4.0 ./start_sim.sh            # 初始观察距离改为 4 m
```

修改源码后可执行 `BUILD=true ./start_sim.sh` 重新编译并启动。直接使用 ROS2 launch 时，对应参数为 `viewer:=true follow_camera:=true camera_distance:=3.0`，`dog_mujoco.launch.py` 和 `sim2sim.launch.py` 均支持。
