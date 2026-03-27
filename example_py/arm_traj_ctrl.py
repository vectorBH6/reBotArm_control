#!/usr/bin/env python3
"""
arm_traj_ctrl.py — 实机测地线轨迹交互控制（Python 版）

终端输入末端目标 "x y z [roll pitch yaw(rad)]"，确认后执行；Ctrl+C 回零退出。
使用: sudo python3 arm_traj_ctrl.py [串口] [YAML] [URDF]

等价 C++ 版: example/real/arm_traj_ctrl.cpp
"""

import sys
import os
import math
import numpy as np

sys.path.insert(0, "../build")
import rebot_py

# 自动计算项目根目录（基于脚本自身位置）
_script_dir = os.path.dirname(os.path.abspath(__file__))
_project_root = os.path.dirname(_script_dir)  # example_py 的上一级


def se3_to_pos_rpy(T: np.ndarray):
    """从 4×4 位姿矩阵提取 (pos, rpy)"""
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


def print_pose(prefix: str, T: np.ndarray):
    pos, rpy = se3_to_pos_rpy(T)
    print(f"{prefix}  pos[{pos[0]:.3f} {pos[1]:.3f} {pos[2]:.3f}]"
          f"  rpy[{rpy[0]:.3f} {rpy[1]:.3f} {rpy[2]:.3f}]")


def main():
    # 如果用户没有传入参数，使用自动计算的路径
    dev  = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    yaml = sys.argv[2] if len(sys.argv) > 2 else os.path.join(_project_root, "config/arm.yaml")
    urdf = sys.argv[3] if len(sys.argv) > 3 else os.path.join(
        _project_root, "urdf/reBot-DevArm_fixend_description/urdf/reBot-DevArm_fixend.urdf")

    arm = rebot_py.ArmController()
    if not arm.init(dev, yaml, urdf):
        sys.exit(1)

    print("输入: x y z [roll pitch yaw(rad)]  Ctrl+C 退出\n")

    try:
        while arm.running():
            print_pose("当前", arm.fk())

            line = input("> ").strip()
            if not line:
                continue

            parts = line.split()
            if len(parts) < 3:
                print("格式错误: x y z [roll pitch yaw]")
                continue

            x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
            ro = float(parts[3]) if len(parts) > 3 else 0.0
            pi = float(parts[4]) if len(parts) > 4 else 0.0
            ya = float(parts[5]) if len(parts) > 5 else 0.0

            print(f"目标  pos[{x:.3f} {y:.3f} {z:.3f}]  rpy[{ro:.3f} {pi:.3f} {ya:.3f}]")
            yn = input("是否执行? [y/N]: ").strip()
            if not yn or yn[0].lower() != "y":
                print("已取消\n")
                continue

            target = rebot_py.ArmController.pose(x, y, z, ro, pi, ya)
            if not arm.move_to_geodesic(target):
                print("IK 或规划失败\n")
                continue

            arm.refresh_hw()
            print_pose("到达", arm.fk())
            print()

    except (KeyboardInterrupt, EOFError):
        pass

    arm.shutdown()


if __name__ == "__main__":
    main()
