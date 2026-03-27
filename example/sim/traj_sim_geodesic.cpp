// =============================================================================
// 交互式测地线轨迹规划 + 统计分析 + MeshCat 回放
// =============================================================================
//
// 功能：
//   - 从当前关节角出发，输入目标位姿
//   - 使用测地线轨迹规划（SE3 空间最短路径 + 关节空间平滑插值）
//   - 计算并显示轨迹统计信息（耗时、点数、IK 成功率、最大误差）
//   - 在 MeshCat 中回放完整轨迹
//
// 使用方法：
//   ./traj_sim_geodesic [urdf_path]
//   输入格式：x y z [roll pitch yaw]  (米 / 弧度)
//   输入 'q' 退出
//
// 算法说明：
//   - 测地线插值：在 SE3 流形上进行位姿插值（旋转 + 平移同步平滑）
//   - 轨迹优化：结合最小加加速度曲线，保证加速度连续
//   - IK 跟踪：沿笛卡尔轨迹逐点求解关节角
//
// 适用场景：
//   - 测试轨迹规划算法性能
//   - 对比不同参数配置（dt、速度曲线、IK 容差）
//   - 验证复杂路径的可行性
//
// =============================================================================

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

// ── 全局状态 ──
static RobotModel      robot;
static Pipe            viz_pipe;
static Eigen::VectorXd q;
static IKParams        ik_params;
static TrajPlanParams  plan_params;

// 运动速度参数（用于自动估算轨迹时长）
static constexpr double LINEAR_SPEED = 0.2;  // 末端线速度 [m/s]

static bool sim_spawn(const char* urdf) { return rebot::sim::spawn(&viz_pipe, urdf); }
static void sim_send(const std::string& s) { rebot::sim::send_line(&viz_pipe, s); }
static void sim_stop() { rebot::sim::stop(&viz_pipe); }

/**
 * @brief 交互式轨迹规划主循环
 * 
 * 工作流程：
 *   1. 显示当前末端位姿（位置 + RPY 欧拉角）
 *   2. 读取目标位姿输入
 *   3. 求解目标点的 IK
 *   4. 规划测地线轨迹（从当前 q 到目标 q）
 *   5. 计算轨迹统计信息（规划耗时、点数、成功率、最大误差）
 *   6. 发送到 MeshCat 播放
 *   7. 更新当前状态，进入下一轮
 */
static void run_interactive()
{
    std::cout << "dt=" << plan_params.dt << "s  IK tol=" << ik_params.tolerance
              << "\n输入: x y z [roll pitch yaw] (米 / 弧度)，q 退出\n\n";

    std::string line;
    while (true) {
        // 显示当前末端位姿
        const auto      T0  = rebot::computeFK(robot, q);
        const auto&     p   = T0.translation();
        Eigen::Vector3d rpy = T0.rotation().eulerAngles(2, 1, 0).reverse();
        std::cout << std::fixed << std::setprecision(3)
                  << "pos[" << p[0] << ' ' << p[1] << ' ' << p[2]
                  << "] rpy[" << rpy[0] << ' ' << rpy[1] << ' ' << rpy[2] << "]> ";
        std::cout.flush();

        // 读取输入
        if (!std::getline(std::cin, line)) break;
        if (line.find_first_not_of(" \t") == std::string::npos) continue;
        if (line == "q" || line == "quit" || line == "exit") break;

        // 解析目标位姿
        double x = 0, y = 0, z = 0, roll = 0, pitch = 0, yaw = 0;
        std::istringstream iss(line);
        if (!(iss >> x >> y >> z)) {
            std::cerr << "格式: x y z [roll pitch yaw]\n";
            continue;
        }
        iss >> roll >> pitch >> yaw;

        const pinocchio::SE3 target = rebot::make_pose(x, y, z, roll, pitch, yaw);

        // 求解目标点 IK
        auto ik = rebot::solveIK(robot, target, q, ik_params);
        if (!ik.success) {
            std::cout << "  IK 无解\n\n";
            continue;
        }

        const pinocchio::SE3 T_end = rebot::computeFK(robot, ik.q);
        
        // 根据距离自动估算运动时长
        double duration = std::max(1.0,
            (target.translation() - T0.translation()).norm() / LINEAR_SPEED);

        // 规划测地线轨迹（计时）
        auto t0   = Clock::now();
        auto traj = rebot::planJointSpaceTrajectory(
            robot, q, ik.q, duration, plan_params, ik_params, 0.1, &T0, &T_end);
        double ms = Ms(Clock::now() - t0).count();

        // 计算统计信息
        rebot::TrajStats st = rebot::computeTrajStats(
            robot, traj, T0, target, traj.back().time, plan_params);
        
        // 输出统计结果
        std::cout << "  " << ms << " ms  " << traj.size() << " 点  成功率="
                  << std::fixed << std::setprecision(1) << st.success_rate * 100
                  << "%  最大误差=" << std::scientific << st.max_ik_error << "\n\n";
        
        // 更新当前状态
        q = traj.back().q;

        // 发送到 MeshCat 播放
        if (viz_pipe.out) {
            std::string lab = "(" + std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z) + ")";
            sim_send(rebot::sim::pack_traj_json(robot, lab, plan_params.dt, traj));
        }
    }
}

/**
 * @brief 主函数：初始化环境并启动交互循环
 * 
 * 命令行参数：
 *   argv[1] - URDF 文件路径（可选，默认使用 URDF_PATH 宏）
 */
int main(int argc, char** argv)
{
    // 解析命令行参数
    std::string urdf_path = (argc > 1 && argv[1][0]) ? argv[1] : URDF_PATH;

    // 初始化机器人模型和参数（使用推荐默认值）
    rebot::sim::init_sim_env(urdf_path, robot, q, ik_params, plan_params);

    // 启动 MeshCat 可视化
    if (!sim_spawn(urdf_path.c_str())) {
        std::cerr << "MeshCat 启动失败\n";
        return 1;
    }
    
    // 进入交互式规划循环
    run_interactive();
    
    // 清理：停止子进程
    sim_stop();
    return 0;
}
