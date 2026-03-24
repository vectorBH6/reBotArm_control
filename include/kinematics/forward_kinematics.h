#pragma once

#include "robot_model.h"
#include <pinocchio/spatial/se3.hpp>
#include <Eigen/Core>
#include <vector>

namespace rebot {

pinocchio::SE3 computeFK(RobotModel& robot, const Eigen::VectorXd& q);

std::vector<pinocchio::SE3> computeAllFramePoses(RobotModel& robot,
                                                  const Eigen::VectorXd& q);

Eigen::MatrixXd computeEndJacobian(RobotModel& robot, const Eigen::VectorXd& q);

/** 由 x,y,z,roll,pitch,yaw(rad) 构造 SE3 位姿（ZYX 欧拉角） */
pinocchio::SE3 make_pose(double x, double y, double z,
                          double roll = 0, double pitch = 0, double yaw = 0);

} // namespace rebot
