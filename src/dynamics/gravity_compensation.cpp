#include "dynamics/gravity_compensation.h"

#include <pinocchio/algorithm/rnea.hpp>

namespace rebot {

GravityCompensator::GravityCompensator(RobotModel& model)
    : model_(model) {}

Eigen::VectorXd GravityCompensator::compute(const Eigen::VectorXd& q) {
    // RNEA(q, v=0, a=0) → M·0 + C·0 + g(q) = g(q)
    const Eigen::VectorXd zero = Eigen::VectorXd::Zero(model_.nv());
    return pinocchio::rnea(model_.model, model_.data, q, zero, zero);
}

}  // namespace rebot
