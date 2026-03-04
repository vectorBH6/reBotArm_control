#include "kinematics/inverse_kinematics.h"

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/spatial/explog.hpp>

namespace rebot {

// 计算当前误差范数（内部辅助）
static double computeError(RobotModel& robot, const Eigen::VectorXd& q,
                            const pinocchio::SE3& target,
                            Eigen::Matrix<double, 6, 1>& err) {
    pinocchio::forwardKinematics(robot.model(), robot.data(), q);
    pinocchio::updateFramePlacements(robot.model(), robot.data());
    const pinocchio::SE3& T_cur = robot.data().oMf[robot.endFrameId()];
    err = pinocchio::log6(T_cur.inverse() * target).toVector();
    return err.norm();
}

IKResult solveIK(RobotModel& robot, const pinocchio::SE3& target,
                 const Eigen::VectorXd& q_init, const IKParams& params) {
    const int nv = robot.nv();
    Eigen::VectorXd q = q_init;
    Eigen::MatrixXd J(6, nv);
    Eigen::Matrix<double, 6, 1> err;

    IKResult result;
    result.success    = false;
    result.iterations = 0;

    // 自适应阻尼初值：随误差大小动态缩放，远离奇异时阻尼小收敛快，
    // 靠近奇异时阻尼自动增大防震荡（Levenberg-Marquardt 风格）
    double lambda = params.damping;
    double prev_err = computeError(robot, q, target, err);

    for (int iter = 0; iter < params.max_iter; ++iter) {
        result.iterations = iter + 1;

        if (prev_err < params.tolerance) {
            result.success = true;
            break;
        }

        // LOCAL 系体雅可比
        pinocchio::computeJointJacobians(robot.model(), robot.data(), q);
        J.setZero();
        pinocchio::getFrameJacobian(robot.model(), robot.data(),
                                    robot.endFrameId(), pinocchio::LOCAL, J);

        // 自适应阻尼：用误差幅度决定 lambda，误差越大阻尼越小
        lambda = params.damping * std::max(1.0, prev_err * 10.0);

        // 阻尼最小二乘 dq
        Eigen::MatrixXd JJT = J * J.transpose();
        JJT.diagonal().array() += lambda;
        Eigen::VectorXd dq = params.step_size * J.transpose() * JJT.ldlt().solve(err);

        // 回退线搜索：若新误差未减小则缩步，最多折半 4 次
        double alpha = 1.0;
        Eigen::VectorXd q_new;
        double new_err;
        Eigen::Matrix<double, 6, 1> err_new;
        for (int ls = 0; ls < 4; ++ls) {
            q_new   = robot.clampConfig(pinocchio::integrate(robot.model(), q, alpha * dq));
            new_err = computeError(robot, q_new, target, err_new);
            if (new_err < prev_err) break;
            alpha *= 0.5;
        }

        q        = q_new;
        err      = err_new;
        prev_err = new_err;
    }

    result.q     = q;
    result.error = prev_err;
    return result;
}

} // namespace rebot
