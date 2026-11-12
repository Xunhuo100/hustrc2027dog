# stamp 使用与控制器一致的 ROS 时钟；超时或会话不符会退出 RL。
std_msgs/Header header
# 从最近 RobotState 原样复制，不能自行递增；每次切换控制状态后需要更新。
uint32 control_session
# 同 RobotState 顺序；腿 q/kp/kd，轮 dq/kd 且 kp=0。
# 全部 16 路必须 mode=1；轮还需 q=0。q: rad，dq: rad/s，tau: Nm。
# PD 语义: tau_total = kp*(q_des-q) + kd*(dq_des-dq) + tau。
# RL 状态直接使用这里的 kp/kd，控制器的站立 PD 参数不会再次覆盖它们。
unitree_go/MotorCmd[16] motor_cmd
