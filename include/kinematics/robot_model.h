#pragma once

#include <string>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <Eigen/Core>

namespace rebot {

// 封装 pinocchio Model+Data，提供机器人模型相关功能
class RobotModel {
public:
    explicit RobotModel(const std::string& urdf_path,
                        const std::string& end_frame = "end_link");

    const pinocchio::Model& model() const { return model_; }
    pinocchio::Data&        data()        { return data_;  }
    const pinocchio::Data&  data()  const { return data_;  }

    int nq() const { return model_.nq; }
    int nv() const { return model_.nv; }

    pinocchio::FrameIndex endFrameId() const { return end_frame_id_; }

    // 各关节零位构型
    Eigen::VectorXd neutralConfig() const;

    // 检查维度与关节限位
    bool isConfigValid(const Eigen::VectorXd& q) const;

    // 将 q 裁剪到关节限位范围内
    Eigen::VectorXd clampConfig(const Eigen::VectorXd& q) const;

private:
    pinocchio::Model      model_;
    pinocchio::Data       data_;
    pinocchio::FrameIndex end_frame_id_;
};

} // namespace rebot
