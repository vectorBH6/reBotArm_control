#pragma once

#include "robot_model.h"
#include <pinocchio/spatial/se3.hpp>
#include <Eigen/Core>

namespace rebot {

struct IKParams {
    int    max_iter  = 1000;
    double tolerance = 1e-4;   // 收敛阈值 ||err||
    double step_size = 0.5;    // 每步更新的缩放系数
    double damping   = 1e-6;   // Tikhonov 正则化系数 λ
};

struct IKResult {
    Eigen::VectorXd q;
    bool   success;
    double error;      // 最终 ||err||
    int    iterations;
};

// 阻尼最小二乘雅可比逆运动学（CLIK）
// q_init：初始关节构型，维度须满足 nq
IKResult solveIK(RobotModel& robot,
                 const pinocchio::SE3& target,
                 const Eigen::VectorXd& q_init,
                 const IKParams& params = IKParams{});

// 带随机重试的 IK：先用 q_seed 求解，若失败则在关节限位内随机采样最多 max_retries 次
// 返回误差最小的结果；q_seed 会被更新为本次最优解
IKResult solveIKWithRetry(RobotModel& robot,
                          const pinocchio::SE3& target,
                          Eigen::VectorXd& q_seed,
                          const IKParams& params = IKParams{},
                          int max_retries = 8);

} // namespace rebot
