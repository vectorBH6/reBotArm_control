#pragma once

#include "kinematics/robot_model.h"
#include <Eigen/Core>

namespace rebot {

/**
 * 重力补偿力矩计算器（基于 pinocchio RNEA）。
 *
 * 动力学方程：M(q)·a + C(q,v)·v + g(q) = τ
 * 令 v=0、a=0 → τ_g = g(q)，即抵消重力所需的关节力矩。
 *
 * 典型用法（零力拖拽模式）：
 *   GravityCompensator comp(robot_model);
 *   // 控制循环内：
 *   auto tau_g = comp.compute(q_current);
 *   arm[i].set_mit(0.f, kd_damp, q_current[i], 0.f, (float)tau_g[i]);
 *
 * 扩展用法（PD + 重力补偿）：
 *   arm[i].set_mit(kp, kd, q_desired[i], dq_desired[i], (float)tau_g[i]);
 */
class GravityCompensator {
public:
    /**
     * @param model  已加载 URDF 的机器人模型（持有引用，须保证生命周期）
     */
    explicit GravityCompensator(RobotModel& model);

    /**
     * 计算关节构型 q 对应的重力补偿力矩向量。
     *
     * @param q   关节角度 [rad]，长度须等于 model.nq()
     * @return    各关节重力补偿力矩 [N·m]，长度等于 model.nv()
     */
    Eigen::VectorXd compute(const Eigen::VectorXd& q);

    int nq() const { return model_.nq(); }
    int nv() const { return model_.nv(); }

private:
    RobotModel& model_;
};

}  // namespace rebot
