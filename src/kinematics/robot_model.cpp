#include "kinematics/robot_model.h"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <stdexcept>

namespace rebot {

RobotModel::RobotModel(const std::string& urdf_path, const std::string& end_frame) {
    pinocchio::urdf::buildModel(urdf_path, model_);
    data_ = pinocchio::Data(model_);

    if (!model_.existFrame(end_frame))
        throw std::runtime_error("URDF 中未找到坐标系：" + end_frame);

    end_frame_id_ = model_.getFrameId(end_frame);
}

Eigen::VectorXd RobotModel::neutralConfig() const {
    return pinocchio::neutral(model_);
}

bool RobotModel::isConfigValid(const Eigen::VectorXd& q) const {
    if (q.size() != model_.nq) return false;
    return (q.array() >= model_.lowerPositionLimit.array()).all() &&
           (q.array() <= model_.upperPositionLimit.array()).all();
}

Eigen::VectorXd RobotModel::clampConfig(const Eigen::VectorXd& q) const {
    return q.cwiseMax(model_.lowerPositionLimit).cwiseMin(model_.upperPositionLimit);
}

} // namespace rebot
