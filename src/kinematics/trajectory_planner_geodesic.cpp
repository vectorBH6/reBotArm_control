#include "kinematics/trajectory_planner_geodesic.h"
#include "kinematics/forward_kinematics.h"

#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include <stdexcept>
#include <cmath>

namespace rebot {

// ─── 内部辅助 ──────────────────────────────────────────────────────────────────

static pinocchio::SE3 se3Geodesic(const pinocchio::SE3& a, const pinocchio::SE3& b, double s) {
    auto twist = pinocchio::log6(a.inverse() * b);
    return a * pinocchio::exp6(twist * s);
}

static pinocchio::SE3 se3Slerp(const pinocchio::SE3& a, const pinocchio::SE3& b, double s) {
    Eigen::Vector3d t = (1.0 - s) * a.translation() + s * b.translation();
    Eigen::Quaterniond qa(a.rotation()), qb(b.rotation());
    if (qa.dot(qb) < 0) qb.coeffs() = -qb.coeffs();
    return pinocchio::SE3(qa.slerp(s, qb).normalized().toRotationMatrix(), t);
}

static double applyProfile(double tau, TrajProfile profile, double accel_ratio) {
    tau = std::max(0.0, std::min(1.0, tau));
    switch (profile) {
        case TrajProfile::LINEAR: return tau;
        case TrajProfile::MIN_JERK: {
            double t2 = tau*tau, t3 = t2*tau, t4 = t3*tau, t5 = t4*tau;
            return 10.0*t3 - 15.0*t4 + 6.0*t5;
        }
        case TrajProfile::TRAPEZOID: {
            double ta = std::max(0.01, std::min(0.49, accel_ratio));
            double td = 1.0 - ta, vm = 1.0 / (1.0 - ta);
            if (tau <= ta)      return 0.5 * vm / ta * tau * tau;
            if (tau <= td) return 0.5 * vm * ta + vm * (tau - ta);
            double dt = 1.0 - tau;
            return 1.0 - 0.5 * vm / ta * dt * dt;
        }
    }
    return tau;
}

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

// ─── CartesianTrajectory ───────────────────────────────────────────────────────

void CartesianTrajectory::addPoint(double t, const pinocchio::SE3& pose) {
    points_.push_back({t, pose});
}

double CartesianTrajectory::duration() const {
    return points_.empty() ? 0.0 : points_.back().time;
}

pinocchio::SE3 CartesianTrajectory::sample(double t) const {
    if (points_.empty()) throw std::runtime_error("CartesianTrajectory::sample: 空轨迹");
    if (t <= points_.front().time) return points_.front().pose;
    if (t >= points_.back().time)  return points_.back().pose;
    int lo = 0, hi = static_cast<int>(points_.size()) - 1;
    while (hi - lo > 1) {
        int mid = (lo + hi) / 2;
        (t < points_[mid].time ? hi : lo) = mid;
    }
    double dt = points_[hi].time - points_[lo].time;
    double s  = (dt < 1e-12) ? 0.0 : (t - points_[lo].time) / dt;
    return se3Slerp(points_[lo].pose, points_[hi].pose, s);
}

// ─── 测地线轨迹 ─────────────────────────────────────────────────────────────────

CartesianTrajectory planCartesianGeodesicTrajectory(const pinocchio::SE3&  start_pose,
                                                    const pinocchio::SE3&  end_pose,
                                                    double                 duration,
                                                    const TrajPlanParams&  params) {
    if (duration <= 0.0)
        throw std::invalid_argument("planCartesianGeodesicTrajectory: duration 必须 > 0");
    CartesianTrajectory traj;
    int n = std::max(2, static_cast<int>(std::ceil(duration / params.dt)) + 1);
    for (int i = 0; i < n; ++i) {
        double t = (i == n - 1) ? duration : i * params.dt;
        double s = applyProfile(t / duration, params.profile, params.accel_ratio);
        traj.addPoint(t, se3Geodesic(start_pose, end_pose, s));
    }
    return traj;
}

// ─── trackTrajectory（CLIK + 零空间限位规避）────────────────────────────────────

std::vector<JointTrajectoryPoint> trackTrajectory(RobotModel&                robot,
                                                  const CartesianTrajectory& traj,
                                                  const Eigen::VectorXd&     q_init,
                                                  const IKParams&            ik_params,
                                                  double                     null_gain) {
    Eigen::VectorXd q = q_init;
    Eigen::MatrixXd J(6, robot.nv());
    Eigen::Matrix<double, 6, 1> err;
    std::vector<JointTrajectoryPoint> result;
    result.reserve(traj.points().size());

    for (const auto& pt : traj.points()) {
        bool converged = false;
        for (int iter = 0; iter < ik_params.max_iter; ++iter) {
            pinocchio::forwardKinematics(robot.model(), robot.data(), q);
            pinocchio::updateFramePlacements(robot.model(), robot.data());
            err = pinocchio::log6(
                robot.data().oMf[robot.endFrameId()].inverse() * pt.pose).toVector();

            if (err.norm() < ik_params.tolerance) { converged = true; break; }

            pinocchio::computeJointJacobians(robot.model(), robot.data(), q);
            J.setZero();
            pinocchio::getFrameJacobian(robot.model(), robot.data(),
                robot.endFrameId(), pinocchio::LOCAL, J);

            double lam = ik_params.damping / std::max(1.0, err.norm() * 10.0);
            Eigen::MatrixXd JJT = J * J.transpose();
            JJT.diagonal().array() += lam;
            auto ldlt = JJT.ldlt();
            Eigen::VectorXd dq = ik_params.step_size * J.transpose() * ldlt.solve(err);

            if (null_gain > 0.0) {
                Eigen::VectorXd g = jointLimitGrad(robot, q);
                dq += null_gain * (g - J.transpose() * ldlt.solve(J * g));
            }
            q = robot.clampConfig(pinocchio::integrate(robot.model(), q, dq));
        }
        result.push_back({pt.time, q, converged});
    }
    return result;
}

// ─── planJointSpaceTrajectory ───────────────────────────────────────────────────

std::vector<JointTrajectoryPoint> planJointSpaceTrajectory(RobotModel&            robot,
                                                          const Eigen::VectorXd& q_start,
                                                          const Eigen::VectorXd& q_end,
                                                          double                 duration,
                                                          const TrajPlanParams&  params,
                                                          const IKParams&        ik_params,
                                                          double                 null_gain) {
    if (duration <= 0.0)
        throw std::invalid_argument("planJointSpaceTrajectory: duration 必须 > 0");
    CartesianTrajectory cart = planCartesianGeodesicTrajectory(
        computeFK(robot, q_start), computeFK(robot, q_end), duration, params);
    return trackTrajectory(robot, cart, q_start, ik_params, null_gain);
}

// ─── jointTrajToCartesian ───────────────────────────────────────────────────────

CartesianTrajectory jointTrajToCartesian(RobotModel&                              robot,
                                         const std::vector<JointTrajectoryPoint>& jt) {
    CartesianTrajectory ct;
    for (const auto& pt : jt) ct.addPoint(pt.time, computeFK(robot, pt.q));
    return ct;
}

// ─── computeTrajStats ────────────────────────────────────────────────────────────

TrajStats computeTrajStats(RobotModel&                              robot,
                           const std::vector<JointTrajectoryPoint>& jt,
                           const pinocchio::SE3&                    T_start,
                           const pinocchio::SE3&                    T_end,
                           double                                   duration,
                           const TrajPlanParams&                    params) {
    TrajStats stats{};
    stats.total_points = static_cast<int>(jt.size());
    CartesianTrajectory ref = planCartesianGeodesicTrajectory(T_start, T_end, duration, params);

    double sum_err = 0.0;
    for (int i = 0; i < stats.total_points && i < static_cast<int>(ref.points().size()); ++i) {
        if (jt[i].ik_success) ++stats.success_count;
        double err = pinocchio::log6(
            computeFK(robot, jt[i].q).inverse() * ref.points()[i].pose).toVector().norm();
        stats.max_ik_error = std::max(stats.max_ik_error, err);
        sum_err += err;
    }
    stats.success_rate = stats.total_points > 0
        ? static_cast<double>(stats.success_count) / stats.total_points : 0.0;
    stats.avg_ik_error = stats.total_points > 0 ? sum_err / stats.total_points : 0.0;
    return stats;
}

} // namespace rebot
