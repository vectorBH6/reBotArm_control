#pragma once

#include "robot_model.h"
#include <pinocchio/spatial/se3.hpp>
#include <Eigen/Core>

namespace rebot {

/** 实时末端步进控制参数 */
struct StepperConfig {
    double dt         = 0.01;   // 控制周期 (s)
    double cart_speed = 0.20;   // 末端线速度上限 (m/s)
    double rot_speed  = 1.50;   // 末端角速度上限 (rad/s)
    double tolerance  = 1e-3;   // 到达阈值
    int    max_steps  = 1000;   // 最大步数
    int    ik_iter    = 15;     // 每步 IK 迭代次数
    double damping    = 1e-3;   // DLS 阻尼
    double null_gain  = 0.10;   // 零空间限位规避增益
};

/** 实时末端步进控制器：每步速度限幅 + DLS-IK，无需预规划路径 */
class EndMotionStepper {
public:
    explicit EndMotionStepper(RobotModel& robot, const StepperConfig& cfg = {});
    void setTarget(const pinocchio::SE3& target);
    Eigen::VectorXd step(const Eigen::VectorXd& q_cur);
    bool   done()  const { return done_;  }
    double error() const { return error_; }
private:
    RobotModel&    robot_;
    StepperConfig  cfg_;
    pinocchio::SE3 target_;
    double         error_       = 0.0;
    bool           done_        = true;
    double         prev_error_  = 1e9;
    int            stall_count_ = 0;
};

} // namespace rebot
