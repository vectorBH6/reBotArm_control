#include "kinematics/forward_kinematics.h"

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

namespace rebot {

pinocchio::SE3 computeFK(RobotModel& robot, const Eigen::VectorXd& q) {
    pinocchio::forwardKinematics(robot.model(), robot.data(), q);
    pinocchio::updateFramePlacements(robot.model(), robot.data());
    return robot.data().oMf[robot.endFrameId()];
}

std::vector<pinocchio::SE3> computeAllFramePoses(RobotModel& robot,
                                                  const Eigen::VectorXd& q) {
    pinocchio::forwardKinematics(robot.model(), robot.data(), q);
    pinocchio::updateFramePlacements(robot.model(), robot.data());
    return {robot.data().oMf.begin(), robot.data().oMf.end()};
}

Eigen::MatrixXd computeEndJacobian(RobotModel& robot, const Eigen::VectorXd& q) {
    pinocchio::computeJointJacobians(robot.model(), robot.data(), q);
    pinocchio::updateFramePlacements(robot.model(), robot.data());

    Eigen::MatrixXd J(6, robot.nv());
    J.setZero();
    pinocchio::getFrameJacobian(robot.model(), robot.data(),
                                robot.endFrameId(), pinocchio::LOCAL, J);
    return J;
}

} // namespace rebot
