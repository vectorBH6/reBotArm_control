#!/usr/bin/env python3
"""
arm_state_monitor.py — 实机关节状态实时监视（Python 版）

50 Hz 轮询电机状态，只读，不使能运动；Ctrl+C 退出。
使用: python3 arm_state_monitor.py [串口] [YAML] [URDF]

等价 C++ 版: example/real/arm_state_monitor.cpp
"""

import sys
import os
import time
import math
import numpy as np

sys.path.insert(0, "../build")
import rebot_py

# 自动计算项目根目录
_script_dir = os.path.dirname(os.path.abspath(__file__))
_project_root = os.path.dirname(_script_dir)


def se3_to_pos_rpy(T: np.ndarray):
    pos = T[:3, 3]
    R = T[:3, :3]
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-6:
        roll  = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw   = math.atan2(R[1, 0], R[0, 0])
    else:
        roll  = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw   = 0.0
    return pos, np.array([roll, pitch, yaw])


def main():
    dev  = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    yaml = sys.argv[2] if len(sys.argv) > 2 else os.path.join(_project_root, "config/arm.yaml")
    urdf = sys.argv[3] if len(sys.argv) > 3 else os.path.join(
        _project_root, "urdf/reBot-DevArm_fixend_description/urdf/reBot-DevArm_fixend.urdf")

    arm = rebot_py.ArmController()
    if not arm.init_monitor(dev, yaml, urdf):
        sys.exit(1)

    nq = arm.robot.nq()
    names = arm.joint_names()

    print("[arm] 状态监视启动，Ctrl+C 退出\n")

    try:
        while True:
            arm.refresh_hw()

            q = arm.q
            T = arm.fk()
            pos, rpy = se3_to_pos_rpy(T)

            joints_str = "  ".join(f"{names[i]}={q[i]:.4f}" for i in range(nq))
            print(f"关节 [rad]: {joints_str}")
            print(f"末端  pos[{pos[0]:.4f} {pos[1]:.4f} {pos[2]:.4f}]"
                  f"  rpy[{rpy[0]:.4f} {rpy[1]:.4f} {rpy[2]:.4f}]\n")

            time.sleep(0.02)  # 50 Hz

    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
