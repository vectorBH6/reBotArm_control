// =============================================================================
// 末端沿空间圆轨迹：离散位姿 + 逐点 IK -> 关节轨迹 -> MeshCat
// =============================================================================
// 直接使用底层 RobotModel + solveIK，不经 ArmController。

#include <cmath>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <vector>

#include "viz_bridge.h"

using rebot::RobotModel;
using rebot::IKParams;
using rebot::TrajPlanParams;
using rebot::sim::Pipe;

static constexpr double kCenterX    = 0.45;
static constexpr double kCenterY    = 0.0;
static constexpr double kCenterZ    = 0.35;
static constexpr double kRadius     = 0.1;
static constexpr int    kNumSamples = 128;
static constexpr double kSampleDt   = 10.0 / kNumSamples;

static RobotModel                    robot;
static Pipe                          viz_pipe;
static Eigen::VectorXd               q;
static IKParams                      ik_params;
static TrajPlanParams                plan_params;
static std::vector<pinocchio::SE3>   ring;

static void sim_send(const std::string& s) { rebot::sim::send_line(&viz_pipe, s); }
static void sim_stop() { rebot::sim::stop(&viz_pipe); }

static void on_sigint(int) { sim_stop(); std::_Exit(0); }

static pinocchio::SE3 sample_circle_pose(int i, int n)
{
    const double th = M_PI / 2 + 2.0 * M_PI * i / n;
    return rebot::make_pose(
        kCenterX,
        kCenterY + kRadius * std::cos(th),
        kCenterZ + kRadius * std::sin(th),
        0, 0, 0);
}

static void build_ring()
{
    ring.clear();
    for (int i = 0; i <= kNumSamples; ++i)
        ring.push_back(sample_circle_pose(i, kNumSamples));
    for (int i = kNumSamples - 1; i >= 0; --i)
        ring.push_back(sample_circle_pose(i, kNumSamples));
}

static void run_trajectory_loop()
{
    int cycle = 0;
    while (true) {
        std::vector<rebot::JointTrajectoryPoint> traj;
        for (const auto& pose : ring) {
            auto ik = rebot::solveIK(robot, pose, q, ik_params);
            q = ik.q;
            traj.push_back({0.0, q, ik.success});
        }
        std::cout << "周期 " << ++cycle << " 点数 " << traj.size() << "\n";
        if (viz_pipe.out)
            sim_send(rebot::sim::pack_traj_json(robot, "circle", kSampleDt, traj));
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

    signal(SIGINT, on_sigint);
    rebot::sim::spawn(&viz_pipe, urdf_path.c_str());

    auto ik0 = rebot::solveIK(robot, sample_circle_pose(0, kNumSamples), q, ik_params);
    if (!ik0.success) {
        std::cerr << "起点 IK 失败\n";
        sim_stop();
        return 1;
    }

    auto go = rebot::planJointSpaceTrajectory(
        robot, q, ik0.q, 2.0, plan_params, ik_params);
    if (viz_pipe.out)
        sim_send(rebot::sim::pack_traj_json(robot, "go_start", plan_params.dt, go));

    build_ring();
    std::cout << "圆心 (" << kCenterX << "," << kCenterY << "," << kCenterZ
              << ") 半径 " << kRadius << " m\n";
    q = go.back().q;
    run_trajectory_loop();
    return 0;
}
