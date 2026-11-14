"""Go2W 电机顺序、仿真默认姿态与输入检查，不依赖策略或 Torch。"""
import numpy as np

JOINT_NAMES = [leg + "_" + joint + "_joint"
               for leg in ["FL", "FR", "RL", "RR"]
               for joint in ["hip", "thigh", "calf", "foot"]]
DEFAULT_POSITION = [0.1, 0.8, -1.5, 0.0, -0.1, 0.8, -1.5, 0.0,
                    0.1, 1.0, -1.5, 0.0, -0.1, 1.0, -1.5, 0.0]

def finite_vector(value, size):
    value = np.asarray(value, dtype=np.float32)
    if value.shape != (size,) or not np.isfinite(value).all():
        raise ValueError("Expected finite vector of size " + str(size))
    return value

