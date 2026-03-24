// =============================================================================
// 交互式逆运动学（IK）+ MeshCat 可视化
// =============================================================================
// 直接使用底层 RobotModel + solveIK，不经 ArmController。

#include <iostream>
#include <sstream>
#include <string>

#include "viz_bridge.h"

using rebot::RobotModel;
using rebot::IKParams;
using rebot::IKResult;
using rebot::sim::Pipe;

static RobotModel      robot;
static Pipe             viz_pipe;
static Eigen::VectorXd  q;
static IKParams         ik_params;

static bool sim_spawn(const char* urdf) { return rebot::sim::spawn(&viz_pipe, urdf); }
static void sim_send(const std::string& s) { rebot::sim::send_line(&viz_pipe, s); }
static void sim_stop() { rebot::sim::stop(&viz_pipe); }

static void run_interactive()
{
    std::cout << "\n输入: x y z [roll pitch yaw] (米 / 弧度)，q 退出\n\n";
    std::string line;
    while (std::getline(std::cin, line)) {
        if (line.find_first_not_of(" \t") == std::string::npos) continue;
        if (line == "q" || line == "quit" || line == "exit") break;

        double x = 0, y = 0, z = 0, roll = 0, pitch = 0, yaw = 0;
        std::istringstream iss(line);
        if (!(iss >> x >> y >> z)) {
            std::cerr << "  格式: x y z [roll pitch yaw]\n";
            continue;
        }
        iss >> roll >> pitch >> yaw;

        pinocchio::SE3 target = rebot::make_pose(x, y, z, roll, pitch, yaw);
        IKResult res = rebot::solveIKWithRetry(robot, target, q, ik_params);

        std::cout << "  IK success=" << res.success << " iters=" << res.iterations
                  << " err=" << res.error << "\n  q=" << res.q.transpose() << "\n";
        if (!res.success)
            std::cerr << "  警告: 未完全收敛，显示最佳近似\n";

        sim_send(rebot::sim::pack_ik_json(target, res.q));
    }
}

int main(int argc, char** argv)
{
    std::string urdf_path = (argc > 1 && argv[1][0]) ? argv[1] : URDF_PATH;

    robot = RobotModel(urdf_path);
    q = robot.neutralConfig();
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
