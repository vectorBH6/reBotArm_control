#include <iostream>
#include <iomanip>
#include <cmath>
#include "kinematics/robot_model.h"
#include "kinematics/forward_kinematics.h"
#include "kinematics/inverse_kinematics.h"
#include <pinocchio/spatial/explog.hpp>

// ─── 轻量测试框架 ──────────────────────────────────────────────────────────────
static int s_pass = 0, s_fail = 0;

#define CHECK(cond, msg)                                                   \
    do {                                                                   \
        if (cond) { std::cout << "  [通过] " << msg << "\n"; ++s_pass; }  \
        else      { std::cout << "  [失败] " << msg << "\n"; ++s_fail; }  \
    } while(0)

// ─── 辅助函数 ─────────────────────────────────────────────────────────────────

// SE3 距离：与 IK 求解器使用相同的 log6 度量
static double se3Dist(const pinocchio::SE3& a, const pinocchio::SE3& b) {
    return pinocchio::log6(a.inverse() * b).toVector().norm();
}

static bool se3Near(const pinocchio::SE3& a, const pinocchio::SE3& b, double tol = 1e-3) {
    return se3Dist(a, b) < tol;
}

// ─── 测试用例 ─────────────────────────────────────────────────────────────────
void testModelLoad(const std::string& urdf) {
    std::cout << "\n=== testModelLoad ===\n";
    rebot::RobotModel robot(urdf);
    CHECK(robot.nq() == 6,  "nq == 6");
    CHECK(robot.nv() == 6,  "nv == 6");
    CHECK(robot.endFrameId() > 0, "end_link 坐标系存在");

    Eigen::VectorXd q0 = robot.neutralConfig();
    CHECK(q0.size() == 6,               "零位构型维度正确");
    CHECK(robot.isConfigValid(q0),      "零位构型有效");
    CHECK(!robot.isConfigValid(q0 * 0 + Eigen::VectorXd::Constant(6, 100.0)),
          "超范围构型被拒绝");
}

void testFKIdentity(const std::string& urdf) {
    std::cout << "\n=== testFKIdentity ===\n";
    rebot::RobotModel robot(urdf);
    Eigen::VectorXd q0 = robot.neutralConfig();

    // FK 不应崩溃，且旋转矩阵行列式应为 1
    pinocchio::SE3 T = rebot::computeFK(robot, q0);
    double det = T.rotation().determinant();
    CHECK(std::abs(det - 1.0) < 1e-9, "旋转矩阵行列式 == 1");
    CHECK(T.translation().allFinite(), "平移向量为有限值");
}

void testFKJacobian(const std::string& urdf) {
    std::cout << "\n=== testFKJacobian ===\n";
    rebot::RobotModel robot(urdf);
    Eigen::VectorXd q = robot.neutralConfig();
    q[1] = 0.5;  q[2] = 0.8;

    Eigen::MatrixXd J = rebot::computeEndJacobian(robot, q);
    CHECK(J.rows() == 6 && J.cols() == 6, "雅可比形状 6x6");
    CHECK(J.allFinite(), "雅可比为有限值");

    // 该构型下雅可比应满秩（非奇异）
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J);
    double smallest_sv = svd.singularValues().minCoeff();
    CHECK(smallest_sv > 1e-3, "雅可比满秩");
}

void testIKRoundTrip(const std::string& urdf) {
    std::cout << "\n=== testIKRoundTrip ===\n";

    // 每行为 [joint1..joint6]，均在 URDF 限位范围内
    const double cases[][6] = {
        { 0.0,  0.5,  0.8,  0.0,  0.0,  0.0},
        { 0.5,  1.0,  0.5, -0.2,  0.3,  0.5},
        {-0.5,  0.6,  1.0,  0.2, -0.4, -0.3},
    };

    rebot::IKParams params;
    params.tolerance = 1e-4;
    params.max_iter  = 3000;
    params.step_size = 0.4;

    for (int i = 0; i < 3; ++i) {
        rebot::RobotModel robot(urdf);

        // 构建目标构型并裁剪到限位
        Eigen::VectorXd q_target(6);
        for (int j = 0; j < 6; ++j) q_target[j] = cases[i][j];
        q_target = robot.clampConfig(q_target);

        pinocchio::SE3 target = rebot::computeFK(robot, q_target);

        // 从目标构型附近出发做逆运动学
        Eigen::VectorXd q_init = robot.clampConfig(q_target + Eigen::VectorXd::Constant(6, 0.05));
        rebot::IKResult res    = rebot::solveIK(robot, target, q_init, params);

        pinocchio::SE3 T_check = rebot::computeFK(robot, res.q);

        std::string tag = "case" + std::to_string(i);
        CHECK(res.success,              tag + " IK 收敛");
        CHECK(se3Near(T_check, target), tag + " FK(IK(T))≈T  err=" + std::to_string(se3Dist(T_check, target)));
    }

    // 全局 IK：从零位出发，测试大范围收敛能力
    std::cout << "  [从零位出发的全局 IK]\n";
    rebot::RobotModel robot(urdf);
    Eigen::VectorXd q_t(6);
    q_t << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    q_t = robot.clampConfig(q_t);
    pinocchio::SE3 tgt = rebot::computeFK(robot, q_t);
    rebot::IKResult global = rebot::solveIK(robot, tgt, robot.neutralConfig(), params);
    pinocchio::SE3 T_chk   = rebot::computeFK(robot, global.q);
    CHECK(global.success,          "全局 IK 收敛");
    CHECK(se3Near(T_chk, tgt),     "全局 FK(IK(T))≈T");
    std::cout << "    iters=" << global.iterations << "  err=" << global.error << "\n";
}

void testAllFramePoses(const std::string& urdf) {
    std::cout << "\n=== testAllFramePoses ===\n";
    rebot::RobotModel robot(urdf);
    auto poses = rebot::computeAllFramePoses(robot, robot.neutralConfig());
    CHECK(!poses.empty(), "坐标系位姿列表非空");
    for (const auto& p : poses)
        CHECK(p.rotation().determinant() > 0.5, "各坐标系旋转矩阵有效");
    std::cout << "  nframes=" << poses.size()
              << "  end_frame_id=" << robot.endFrameId() << "\n";
}

// ─── 入口 ─────────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    const std::string urdf = (argc > 1) ? argv[1] : URDF_PATH;

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "URDF: " << urdf << "\n";

    testModelLoad(urdf);
    testFKIdentity(urdf);
    testFKJacobian(urdf);
    testAllFramePoses(urdf);
    testIKRoundTrip(urdf);

    std::cout << "\n=== 结果：" << s_pass << " 通过，"
              << s_fail << " 失败 ===\n";
    return s_fail ? 1 : 0;
}
