// =============================================================================
// 交互式：当前 q -> plan 测地线轨迹 -> 统计 + MeshCat 回放
// =============================================================================
// 直接使用底层 RobotModel + planJointSpaceTrajectory，不经 ArmController。

#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include "viz_bridge.h"

using rebot::RobotModel;
using rebot::IKParams;
using rebot::TrajPlanParams;
using rebot::sim::Pipe;
using Clock = std::chrono::steady_clock;
using Ms    = std::chrono::duration<double, std::milli>;

static RobotModel      robot;
static Pipe             viz_pipe;
static Eigen::VectorXd  q;
static IKParams         ik_params;
static TrajPlanParams   plan_params;

static constexpr double LINEAR_SPEED = 0.2;

static bool sim_spawn(const char* urdf) { return rebot::sim::spawn(&viz_pipe, urdf); }
static void sim_send(const std::string& s) { rebot::sim::send_line(&viz_pipe, s); }
static void sim_stop() { rebot::sim::stop(&viz_pipe); }

static void run_interactive()
{
    std::cout << "dt=" << plan_params.dt << "s  IK tol=" << ik_params.tolerance
              << "\n输入: x y z [roll pitch yaw] (米 / 弧度)，q 退出\n\n";

    std::string line;
    while (true) {
        const auto      T0  = rebot::computeFK(robot, q);
        const auto&     p   = T0.translation();
        Eigen::Vector3d rpy = T0.rotation().eulerAngles(2, 1, 0).reverse();
        std::cout << std::fixed << std::setprecision(3)
                  << "pos[" << p[0] << ' ' << p[1] << ' ' << p[2]
                  << "] rpy[" << rpy[0] << ' ' << rpy[1] << ' ' << rpy[2] << "]> ";
        std::cout.flush();

        if (!std::getline(std::cin, line)) break;
        if (line.find_first_not_of(" \t") == std::string::npos) continue;
        if (line == "q" || line == "quit" || line == "exit") break;

        double x = 0, y = 0, z = 0, roll = 0, pitch = 0, yaw = 0;
        std::istringstream iss(line);
        if (!(iss >> x >> y >> z)) {
            std::cerr << "格式: x y z [roll pitch yaw]\n";
            continue;
        }
        iss >> roll >> pitch >> yaw;

        const pinocchio::SE3 target = rebot::make_pose(x, y, z, roll, pitch, yaw);

        auto ik = rebot::solveIK(robot, target, q, ik_params);
        if (!ik.success) {
            std::cout << "  IK 无解\n\n";
            continue;
        }

        const pinocchio::SE3 T_end = rebot::computeFK(robot, ik.q);
        double duration = std::max(1.0,
            (target.translation() - T0.translation()).norm() / LINEAR_SPEED);

        auto t0   = Clock::now();
        auto traj = rebot::planJointSpaceTrajectory(
            robot, q, ik.q, duration, plan_params, ik_params, 0.1, &T0, &T_end);
        double ms = Ms(Clock::now() - t0).count();

        rebot::TrajStats st = rebot::computeTrajStats(
            robot, traj, T0, target, traj.back().time, plan_params);
        std::cout << "  " << ms << " ms  " << traj.size() << " 点  成功率="
                  << std::fixed << std::setprecision(1) << st.success_rate * 100
                  << "%  最大误差=" << std::scientific << st.max_ik_error << "\n\n";
        q = traj.back().q;

        if (viz_pipe.out) {
            std::string lab = "(" + std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z) + ")";
            sim_send(rebot::sim::pack_traj_json(robot, lab, plan_params.dt, traj));
        }
    }
}

int main(int argc, char** argv)
{
    std::string urdf_path = (argc > 1 && argv[1][0]) ? argv[1] : URDF_PATH;

    robot = RobotModel(urdf_path);
    q = robot.neutralConfig();
    plan_params.dt      = 1.0 / 50.0;
    plan_params.profile = rebot::TrajProfile::MIN_JERK;
    ik_params.max_iter  = 200;
    ik_params.tolerance = 1e-4;
    ik_params.step_size = 0.8;

    if (!sim_spawn(urdf_path.c_str())) {
        std::cerr << "MeshCat 启动失败\n";
        return 1;
    }
    run_interactive();
    sim_stop();
    return 0;
}
