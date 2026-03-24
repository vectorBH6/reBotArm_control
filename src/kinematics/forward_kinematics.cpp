#include "kinematics/forward_kinematics.h"

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <Eigen/Geometry>

namespace rebot {

pinocchio::SE3 computeFK(RobotModel& robot, const Eigen::VectorXd& q) {
    pinocchio::forwardKinematics(robot.model, robot.data, q);
    pinocchio::updateFramePlacements(robot.model, robot.data);
    return robot.data.oMf[robot.end_frame_id];
}

std::vector<pinocchio::SE3> computeAllFramePoses(RobotModel& robot,
                                                  const Eigen::VectorXd& q) {
    pinocchio::forwardKinematics(robot.model, robot.data, q);
    pinocchio::updateFramePlacements(robot.model, robot.data);
    return {robot.data.oMf.begin(), robot.data.oMf.end()};
}

Eigen::MatrixXd computeEndJacobian(RobotModel& robot, const Eigen::VectorXd& q) {
    pinocchio::computeJointJacobians(robot.model, robot.data, q);
    pinocchio::updateFramePlacements(robot.model, robot.data);

    Eigen::MatrixXd J(6, robot.nv());
    J.setZero();
    pinocchio::getFrameJacobian(robot.model, robot.data,
                                robot.end_frame_id, pinocchio::LOCAL, J);
    return J;
}

pinocchio::SE3 make_pose(double x, double y, double z,
                          double roll, double pitch, double yaw)
{
    auto R = (Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ()) *
              Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
              Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX())).toRotationMatrix();
    return pinocchio::SE3(R, Eigen::Vector3d(x, y, z));
}

} // namespace rebot
