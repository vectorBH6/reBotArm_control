#pragma once

#include <string>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <Eigen/Core>

namespace rebot {

struct RobotModel {
    pinocchio::Model      model;
    pinocchio::Data       data;
    pinocchio::FrameIndex end_frame_id;

    explicit RobotModel() = default;
    explicit RobotModel(const std::string& urdf_path,
                        const std::string& end_frame = "end_link");

    int nq() const { return model.nq; }
    int nv() const { return model.nv; }

    Eigen::VectorXd neutralConfig() const;
    bool isConfigValid(const Eigen::VectorXd& q) const;
    Eigen::VectorXd clampConfig(const Eigen::VectorXd& q) const;
};

} // namespace rebot
