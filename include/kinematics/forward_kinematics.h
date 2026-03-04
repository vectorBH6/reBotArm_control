#pragma once

#include "robot_model.h"
#include <pinocchio/spatial/se3.hpp>
#include <Eigen/Core>
#include <vector>

namespace rebot {

// 正运动学：返回末端执行器在世界系下的位姿
pinocchio::SE3 computeFK(RobotModel& robot, const Eigen::VectorXd& q);

// 返回模型中所有坐标系的位姿
std::vector<pinocchio::SE3> computeAllFramePoses(RobotModel& robot,
                                                  const Eigen::VectorXd& q);

// 计算末端执行器处的 6×nv 体雅可比（LOCAL 坐标系）
Eigen::MatrixXd computeEndJacobian(RobotModel& robot, const Eigen::VectorXd& q);

} // namespace rebot
