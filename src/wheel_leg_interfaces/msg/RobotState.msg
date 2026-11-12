# 顺序 FL, FR, RL, RR，每条腿 hip, thigh, calf, wheel。
# stamp 是底层状态接收时间，转发时不刷新；不是驱动采样时间。
std_msgs/Header header
uint8 PASSIVE=0
uint8 STAND=1
uint8 RL=2
uint8 control_mode
# 每次切换状态递增；指令必须携带当前会话号。
uint32 control_session
unitree_go/MotorState[16] motor_state
# motor_state: q 为 rad，dq 为 rad/s，tau_est 为 Nm；轮累计角不直接作为 HIM 观测。
# ROS 四元数字段 x,y,z,w；角速度 rad/s、加速度 m/s²，均位于机体坐标系。
# 驱动需提前补偿 IMU 安装方向；当前转换层不处理安装角和偏置。
sensor_msgs/Imu imu
