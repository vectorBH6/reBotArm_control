#!/usr/bin/env python3
"""
ik_fk_demo.py — 纯运动学演示（无需硬件）

演示 RobotModel 加载、FK、IK、轨迹规划等纯算法接口。
使用: python3 ik_fk_demo.py [URDF路径]

此示例不需要连接实机，适合离线开发与调试。
"""

import sys
import math
import numpy as np

sys.path.insert(0, "../build")
import rebot_py

URDF_DEFAULT = "../urdf/reBot-DevArm_fixend_description/urdf/reBot-DevArm_fixend.urdf"


def main():
    urdf = sys.argv[1] if len(sys.argv) > 1 else URDF_DEFAULT

    # ── 加载模型 ──────────────────────────────────────────────────────────────
    print(f"[model] 加载 {urdf}")
    robot = rebot_py.RobotModel(urdf)
    print(f"  nq={robot.nq()}, nv={robot.nv()}")

    q0 = robot.neutral_config()
    print(f"  零位构型: {q0}")

    # ── 正运动学 ──────────────────────────────────────────────────────────────
    T0 = rebot_py.compute_fk(robot, q0)
    print(f"\n[FK] 零位末端位姿:\n{T0}")
    print(f"  位置: {T0[:3, 3]}")

    # ── 雅可比 ────────────────────────────────────────────────────────────────
    J = rebot_py.compute_end_jacobian(robot, q0)
    print(f"\n[Jacobian] 末端雅可比 shape={J.shape}")

    # ── 逆运动学 ──────────────────────────────────────────────────────────────
    target = rebot_py.make_pose(0.35, 0.0, 0.25, math.pi, 0, 0)
    print(f"\n[IK] 目标位姿:\n{target}")

    ik_params = rebot_py.IKParams()
    ik_params.max_iter = 500
    ik_params.tolerance = 1e-4

    result = rebot_py.solve_ik(robot, target, q0, ik_params)
    print(f"  {result}")
    if result.success:
        print(f"  解: {result.q}")
        T_solved = rebot_py.compute_fk(robot, result.q)
        print(f"  验证位置: {T_solved[:3, 3]}")
        pos_err = np.linalg.norm(T_solved[:3, 3] - target[:3, 3])
        print(f"  位置误差: {pos_err:.6f} m")

    # ── 轨迹规划 ──────────────────────────────────────────────────────────────
    if result.success:
        print("\n[Traj] 关节空间轨迹规划 (零位 → IK 解, 2s)")
        params = rebot_py.TrajPlanParams()
        params.profile = rebot_py.TrajProfile.MIN_JERK

        traj = rebot_py.plan_joint_space_trajectory(
            robot, q0, result.q, duration=2.0, params=params)

        print(f"  轨迹点数: {len(traj)}")
        if traj:
            print(f"  起始 t={traj[0].time:.3f}s  q={traj[0].q}")
            print(f"  终止 t={traj[-1].time:.3f}s  q={traj[-1].q}")

            ik_ok = sum(1 for pt in traj if pt.ik_success)
            print(f"  IK 成功率: {ik_ok}/{len(traj)} = {ik_ok/len(traj)*100:.1f}%")

    # ── 重力补偿 ──────────────────────────────────────────────────────────────
    print("\n[Gravity] 重力补偿力矩")
    comp = rebot_py.GravityCompensator(robot)
    tau_g = comp.compute(q0)
    print(f"  零位力矩: {tau_g}")

    if result.success:
        tau_g2 = comp.compute(result.q)
        print(f"  IK 解处力矩: {tau_g2}")

    print("\n完成!")


if __name__ == "__main__":
    main()
