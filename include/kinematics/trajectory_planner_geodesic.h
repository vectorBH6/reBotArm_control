#pragma once

#include "robot_model.h"
#include "inverse_kinematics.h"

#include <pinocchio/spatial/se3.hpp>
#include <Eigen/Core>
#include <vector>

namespace rebot {

// ─── 数据结构 ─────────────────────────────────────────────────────────────────

struct TrajectoryPoint {
    double         time;
    pinocchio::SE3 pose;
};

class CartesianTrajectory {
public:
    CartesianTrajectory() = default;
    void addPoint(double t, const pinocchio::SE3& pose);
    pinocchio::SE3 sample(double t) const;  // 相邻点间 LERP+SLERP 插值
    double duration() const;
    const std::vector<TrajectoryPoint>& points() const { return points_; }
    bool empty() const { return points_.empty(); }
private:
    std::vector<TrajectoryPoint> points_;
};

enum class TrajProfile {
    LINEAR,
    MIN_JERK,   // 五阶多项式，加减速最平滑
    TRAPEZOID,  // 梯形速度
};

struct TrajPlanParams {
    double      dt          = 0.02;
    TrajProfile profile     = TrajProfile::MIN_JERK;
    double      accel_ratio = 0.25;  // 梯形：加减速段占比
};

struct JointTrajectoryPoint {
    double          time;
    Eigen::VectorXd q;
    bool            ik_success;
};

struct TrajStats {
    int    total_points;
    int    success_count;
    double success_rate;
    double max_ik_error;
    double avg_ik_error;
};

// ─── 接口 ─────────────────────────────────────────────────────────────────────

/** 测地线轨迹：T(s)=T_start*exp6(log6(T_start⁻¹*T_end)*s)，仅改姿态时定点旋转 */
CartesianTrajectory planCartesianGeodesicTrajectory(
    const pinocchio::SE3&  start_pose,
    const pinocchio::SE3&  end_pose,
    double                 duration,
    const TrajPlanParams&  params = TrajPlanParams{});

/** 关节轨迹：测地线 + CLIK 跟踪，需 ik_params、null_gain */
std::vector<JointTrajectoryPoint> planJointSpaceTrajectory(
    RobotModel&            robot,
    const Eigen::VectorXd& q_start,
    const Eigen::VectorXd& q_end,
    double                 duration,
    const TrajPlanParams&  params = TrajPlanParams{},
    const IKParams&        ik_params = IKParams{},
    double                 null_gain = 0.1);

/** CLIK 跟踪笛卡尔轨迹，零空间规避关节限位 */
std::vector<JointTrajectoryPoint> trackTrajectory(
    RobotModel&                robot,
    const CartesianTrajectory&  traj,
    const Eigen::VectorXd&      q_init,
    const IKParams&             ik_params = IKParams{},
    double                      null_gain = 0.1);

CartesianTrajectory jointTrajToCartesian(
    RobotModel&                              robot,
    const std::vector<JointTrajectoryPoint>& joint_traj);

/** 统计 joint_traj 相对测地线 T_start→T_end 的误差 */
TrajStats computeTrajStats(
    RobotModel&                              robot,
    const std::vector<JointTrajectoryPoint>& joint_traj,
    const pinocchio::SE3&                    T_start,
    const pinocchio::SE3&                    T_end,
    double                                   duration,
    const TrajPlanParams&                    params = TrajPlanParams{});

} // namespace rebot
