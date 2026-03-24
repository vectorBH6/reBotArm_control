#include "kinematics/robot_model.h"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <stdexcept>

namespace rebot {

RobotModel::RobotModel(const std::string& urdf_path, const std::string& end_frame) {
    pinocchio::urdf::buildModel(urdf_path, model);
    data = pinocchio::Data(model);

    if (!model.existFrame(end_frame))
        throw std::runtime_error("URDF 中未找到坐标系：" + end_frame);

    end_frame_id = model.getFrameId(end_frame);
}

Eigen::VectorXd RobotModel::neutralConfig() const {
    return pinocchio::neutral(model);
}

bool RobotModel::isConfigValid(const Eigen::VectorXd& q) const {
    if (q.size() != model.nq) return false;
    return (q.array() >= model.lowerPositionLimit.array()).all() &&
           (q.array() <= model.upperPositionLimit.array()).all();
}

Eigen::VectorXd RobotModel::clampConfig(const Eigen::VectorXd& q) const {
    return q.cwiseMax(model.lowerPositionLimit).cwiseMin(model.upperPositionLimit);
}

} // namespace rebot
