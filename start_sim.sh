#!/usr/bin/env bash
# 从任意目录运行；修改下面的默认值，或用环境变量临时覆盖。
# 例：ENABLE_KEYBOARD=true SPEED=0.6 ./start_sim.sh
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ---------- 常用配置 ----------
ENABLE_KEYBOARD="${ENABLE_KEYBOARD:-true}"  # true：键盘控制；false：外部或固定速度
SPEED="${SPEED:-1.0}"                      # 方向键平移速度，m/s，须大于 0
VX="${VX:-0.0}"                           # 关闭键盘时的固定前进速度，m/s
VY="${VY:-0.0}"                           # 关闭键盘时的固定左移速度，m/s
WZ="${WZ:-0.0}"                           # 关闭键盘时的固定左转速度，rad/s
VIEWER="${VIEWER:-true}"                   # 是否打开 MuJoCo 窗口
FOLLOW_CAMERA="${FOLLOW_CAMERA:-true}"     # 观察视角是否跟随狗子
CAMERA_DISTANCE="${CAMERA_DISTANCE:-3.0}"  # 初始观察距离，m，可用鼠标继续缩放
POLICY_PROFILE="${POLICY_PROFILE:-parkour}"
MODEL_XML="${MODEL_XML:-$ROOT/src/wheel_leg_mujoco/models/go2w/urdf/scene_fields.xml}"
BUILD="${BUILD:-false}"                    # 修改源码后设为 true，启动前重新编译
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
VENV_DIR="${VENV_DIR:-$ROOT/.venv}"
# -----------------------------

cd "$ROOT"
for required_file in "$ROS_SETUP" "$VENV_DIR/bin/activate" "$MODEL_XML"; do
    if [ ! -f "$required_file" ]; then
        echo "缺少文件：$required_file。请检查配置；环境安装见 README_BY_GPT.md。" >&2
        exit 1
    fi
done
if [ "$ENABLE_KEYBOARD" != true ] && [ "$ENABLE_KEYBOARD" != false ]; then
    echo "ENABLE_KEYBOARD 请填写 true 或 false。" >&2
    exit 1
fi

source "$ROS_SETUP"
# 首次运行自动编译；已有构建时默认跳过。
if [ "$BUILD" = true ] || [ ! -f install/wheel_leg_mujoco/share/wheel_leg_mujoco/package.bash ] ||
   [ ! -f install/himloco_policy/share/himloco_policy/package.bash ] ||
   [ ! -f install/robot_ctrl/share/robot_ctrl/package.bash ]; then
    colcon build --packages-up-to robot_ctrl wheel_leg_mujoco himloco_policy --cmake-args -DBUILD_TESTING=OFF
fi
source install/setup.bash
source "$VENV_DIR/bin/activate"
# 提前验证 Python 环境和速度，避免启动后才发现错误。
python3 - "$SPEED" "$VX" "$VY" "$WZ" <<'PY_CHECK'
import math
import sys
import rclpy
import torch
import mujoco

values = [float(value) for value in sys.argv[1:]]
if not all(math.isfinite(value) for value in values) or values[0] <= 0.0:
    sys.exit("SPEED 必须大于 0，且 SPEED/VX/VY/WZ 都必须是有限数值。")
PY_CHECK

launch_command=(ros2 launch "$ROOT/launch/dog_mujoco.launch.py"
    "policy_profile:=$POLICY_PROFILE" "model_xml:=$MODEL_XML"
    "viewer:=$VIEWER" "follow_camera:=$FOLLOW_CAMERA" "camera_distance:=$CAMERA_DISTANCE"
    "enable_keyboard:=$ENABLE_KEYBOARD"
    "keyboard_linear_speed:=$SPEED")
echo "配置：策略=$POLICY_PROFILE，键盘=$ENABLE_KEYBOARD，按键速度=$SPEED m/s，相机跟随=$FOLLOW_CAMERA"

# 键盘开启时，只让键盘发布速度；三个固定速度均为零时，等待外部 /cmd_vel。
if [ "$ENABLE_KEYBOARD" = true ] || python3 - "$VX" "$VY" "$WZ" <<'PY_CHECK'
import sys
sys.exit(0 if all(float(value) == 0.0 for value in sys.argv[1:]) else 1)
PY_CHECK
then
    exec "${launch_command[@]}"
fi

# 固定速度必须持续发布，单次消息会被策略节点的速度超时机制清零。
# 两个进程中任意一个退出，就结束另一个；Ctrl+C 也会清理。
launch_pid=""
velocity_pid=""
cleanup() {
    trap - EXIT INT TERM
    if [ -n "$velocity_pid" ]; then kill -TERM "$velocity_pid" 2>/dev/null || true; fi
    if [ -n "$launch_pid" ]; then kill -TERM "$launch_pid" 2>/dev/null || true; fi
    wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
"${launch_command[@]}" &
launch_pid=$!
echo "持续发布 /cmd_vel：vx=$VX m/s，vy=$VY m/s，wz=$WZ rad/s"
ros2 topic pub -r 20 --print 0 /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: $VX, y: $VY}, angular: {z: $WZ}}" &
velocity_pid=$!
wait -n "$launch_pid" "$velocity_pid"
