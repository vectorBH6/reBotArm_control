// =============================================================================
// 定点 + 姿态「圆锥」式关键帧：笛卡尔测地线 -> 每点 IK -> MeshCat
// =============================================================================
// 直接使用底层 RobotModel + planCartesianGeodesicTrajectory，不经 ArmController。

#include <cmath>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <vector>

#include "viz_bridge.h"

using rebot::RobotModel;
using rebot::IKParams;
using rebot::TrajPlanParams;
using rebot::CartesianTrajectory;
using rebot::sim::Pipe;

static constexpr double kRefX          = 0.3;
static constexpr double kRefY          = 0.0;
static constexpr double kRefZ          = 0.0;
static constexpr int    kSegPtsPerEdge = 64;
static constexpr double kLoopPeriodSec = 5.0;

static RobotModel          robot;
static Pipe                viz_pipe;
static Eigen::VectorXd     q;
static IKParams            ik_params;
static TrajPlanParams      plan_params;
static CartesianTrajectory cart;
static double              traj_dt = 0.0;

static void sim_send(const std::string& s) { rebot::sim::send_line(&viz_pipe, s); }
static void sim_stop() { rebot::sim::stop(&viz_pipe); }

static void on_sigint(int) { sim_stop(); std::_Exit(0); }

static void build_cartesian_loop(const std::vector<pinocchio::SE3>& keyframes,
                                 TrajPlanParams plan)
{
    const int    n     = static_cast<int>(keyframes.size()) - 1;
    const double seg_t = kLoopPeriodSec / n;
    cart = CartesianTrajectory{};
    double t0 = 0.0;
    for (int s = 0; s < n; ++s) {
        auto seg = rebot::planCartesianGeodesicTrajectory(
            keyframes[s], keyframes[s + 1], seg_t, plan);
        for (size_t i = (cart.empty() ? 0 : 1); i < seg.points().size(); ++i)
            cart.addPoint(t0 + seg.points()[i].time, seg.points()[i].pose);
        t0 += seg_t;
    }
}

static void run_trajectory_loop()
{
    int cycle = 0;
    while (true) {
        std::vector<rebot::JointTrajectoryPoint> traj;
        for (const auto& pt : cart.points()) {
            auto ik = rebot::solveIK(robot, pt.pose, q, ik_params);
            q = ik.q;
            traj.push_back({pt.time, q, ik.success});
        }
        std::cout << "周期 " << ++cycle << " 点数 " << traj.size() << "\n";
        if (viz_pipe.out)
            sim_send(rebot::sim::pack_traj_json(robot, "cone", traj_dt, traj));
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

    std::vector<pinocchio::SE3> keyframes = {
        rebot::make_pose(kRefX, kRefY, kRefZ, 0.00, 1.14, 0.00),
        rebot::make_pose(kRefX, kRefY, kRefZ, -1.57, 1.14, -1.57),
        rebot::make_pose(kRefX, kRefY, kRefZ, 0.00, 2.00, 0.00),
        rebot::make_pose(kRefX, kRefY, kRefZ, 1.57, 1.14, 1.57),
        rebot::make_pose(kRefX, kRefY, kRefZ, 0.00, 1.14, 0.00),
    };

    auto ik0 = rebot::solveIK(robot, keyframes[0], q, ik_params);
    if (!ik0.success) {
        std::cerr << "起点 IK 失败\n";
        sim_stop();
        return 1;
    }

    auto go = rebot::planJointSpaceTrajectory(
        robot, q, ik0.q, 2.0, plan_params, ik_params);
    if (viz_pipe.out)
        sim_send(rebot::sim::pack_traj_json(robot, "go_start", plan_params.dt, go));

    TrajPlanParams seg_plan = plan_params;
    seg_plan.dt = (kLoopPeriodSec / (static_cast<int>(keyframes.size()) - 1)) / kSegPtsPerEdge;
    traj_dt = seg_plan.dt;
    build_cartesian_loop(keyframes, seg_plan);

    std::cout << "定点 (" << kRefX << "," << kRefY << "," << kRefZ << ") 圆锥姿态循环\n";
    q = go.back().q;
    run_trajectory_loop();
    return 0;
}
