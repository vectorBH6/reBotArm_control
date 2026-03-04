#include "kinematics/end_motion_stepper.h"

#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include <cmath>

namespace rebot {

static Eigen::VectorXd jointLimitGrad(const RobotModel& robot, const Eigen::VectorXd& q) {
    const auto& lo = robot.model().lowerPositionLimit;
    const auto& hi = robot.model().upperPositionLimit;
    Eigen::VectorXd g = Eigen::VectorXd::Zero(robot.nv());
    for (int i = 0; i < robot.nv(); ++i) {
        if (!std::isfinite(lo[i]) || !std::isfinite(hi[i])) continue;
        double dl = q[i] - lo[i], dh = hi[i] - q[i];
        if (dl < 1e-6 || dh < 1e-6) continue;
        g[i] = (dh - dl) / (dl * dh);
    }
    return g;
}

static Eigen::Matrix<double, 6, 1> se3ErrorLWA(const pinocchio::SE3& T_a, const pinocchio::SE3& T_b) {
    Eigen::Matrix<double, 6, 1> err;
    err.head<3>() = T_b.translation() - T_a.translation();
    Eigen::Quaterniond qa(T_a.rotation()), qb(T_b.rotation());
    if (qa.dot(qb) < 0.0) qb.coeffs() = -qb.coeffs();
    err.tail<3>() = 2.0 * (qb * qa.inverse()).vec();
    return err;
}

EndMotionStepper::EndMotionStepper(RobotModel& robot, const StepperConfig& cfg)
    : robot_(robot), cfg_(cfg), target_(pinocchio::SE3::Identity()) {}

void EndMotionStepper::setTarget(const pinocchio::SE3& target) {
    target_      = target;
    done_        = false;
    stall_count_ = 0;
    prev_error_  = 1e9;
}

Eigen::VectorXd EndMotionStepper::step(const Eigen::VectorXd& q_cur) {
    if (done_) return q_cur;

    pinocchio::forwardKinematics(robot_.model(), robot_.data(), q_cur);
    pinocchio::updateFramePlacements(robot_.model(), robot_.data());
    const pinocchio::SE3 T_cur = robot_.data().oMf[robot_.endFrameId()];

    const Eigen::Matrix<double, 6, 1> e_full = se3ErrorLWA(T_cur, target_);
    error_ = e_full.norm();
    if (error_ < cfg_.tolerance) { done_ = true; return q_cur; }

    if (error_ > prev_error_ * 0.999) {
        if (++stall_count_ >= 30) {
            stall_count_ = 0;
            return robot_.clampConfig(
                pinocchio::integrate(robot_.model(), q_cur,
                    Eigen::VectorXd::Random(robot_.nv()) * (cfg_.tolerance * 2.0)));
        }
    } else {
        stall_count_ = 0;
    }
    prev_error_ = error_;

    Eigen::Matrix<double, 6, 1> e_step = e_full;
    {
        const double max_lin = cfg_.cart_speed * cfg_.dt;
        const double max_ang = cfg_.rot_speed * cfg_.dt;
        const double lin = e_step.head<3>().norm(), ang = e_step.tail<3>().norm();
        if (lin > max_lin) e_step.head<3>() *= max_lin / lin;
        if (ang > max_ang) e_step.tail<3>() *= max_ang / ang;
    }

    pinocchio::SE3 T_sub;
    T_sub.translation() = T_cur.translation() + e_step.head<3>();
    T_sub.rotation() = T_cur.rotation() *
        pinocchio::exp3(Eigen::Vector3d(T_cur.rotation().transpose() * e_step.tail<3>()));

    const int nv = robot_.nv();
    Eigen::MatrixXd J(6, nv);
    Eigen::MatrixXd JJT(6, 6);
    Eigen::Matrix<double, 6, 1> e_sub;
    Eigen::VectorXd q = q_cur;

    for (int i = 0; i < cfg_.ik_iter; ++i) {
        J.setZero();
        pinocchio::computeFrameJacobian(robot_.model(), robot_.data(), q,
            robot_.endFrameId(), pinocchio::LOCAL_WORLD_ALIGNED, J);
        const pinocchio::SE3& T = robot_.data().oMf[robot_.endFrameId()];

        e_sub = se3ErrorLWA(T, T_sub);
        if (e_sub.norm() < cfg_.tolerance * 0.1) break;

        const double err_ratio = e_sub.norm() / cfg_.tolerance;
        const double lambda = std::max(1e-6, cfg_.damping * std::min(1.0, err_ratio));

        JJT.noalias() = J * J.transpose();
        JJT.diagonal().array() += lambda;
        const auto ldlt = JJT.ldlt();

        Eigen::VectorXd dq = J.transpose() * ldlt.solve(e_sub);

        if (cfg_.null_gain > 0.0) {
            const double null_scale = std::min(1.0, err_ratio / 2.0);
            if (null_scale > 1e-3) {
                const Eigen::VectorXd g = jointLimitGrad(robot_, q);
                dq += null_scale * cfg_.null_gain * (g - J.transpose() * ldlt.solve(J * g));
            }
        }
        q = robot_.clampConfig(pinocchio::integrate(robot_.model(), q, dq));
    }
    return q;
}

} // namespace rebot
