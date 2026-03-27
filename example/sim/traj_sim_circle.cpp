// =============================================================================
// 末端空间圆形轨迹仿真演示
// =============================================================================
//
// 功能：
//   - 在空间中定义一个圆形路径（固定圆心、半径、采样数）
//   - 离散采样圆上的位姿点
//   - 逐点求解 IK 得到关节轨迹
//   - 在 MeshCat 中循环播放（正反双向闭合环）
//
// 使用方法：
//   ./traj_sim_circle [urdf_path]
//   程序自动运行，按 Ctrl+C 退出
//
// 算法说明：
//   - 圆形采样：在 YZ 平面上均匀采样圆周点
//   - 构建闭环：正向 + 反向采样，形成平滑闭合轨迹
//   - IK 序列求解：从中位配置出发，逐点求解（利用前一点作为初值）
//
// 适用场景：
//   - 测试工作空间内的连续运动能力
//   - 验证 IK 求解器的稳定性
//   - 演示末端笛卡尔空间轨迹跟踪
//
// 参数说明：
//   - kCenterX/Y/Z：圆心坐标 [米]
//   - kRadius：圆半径 [米]
//   - kNumSamples：圆周采样点数（越大越平滑）
//   - kSampleDt：播放时的帧间隔 [秒]
//
// =============================================================================

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

// ── 圆形轨迹参数 ──
static constexpr double kCenterX    = 0.45;     // 圆心 X 坐标 [m]
static constexpr double kCenterY    = 0.0;      // 圆心 Y 坐标 [m]
static constexpr double kCenterZ    = 0.35;     // 圆心 Z 坐标 [m]
static constexpr double kRadius     = 0.1;      // 圆半径 [m]
static constexpr int    kNumSamples = 128;      // 圆周采样点数
static constexpr double kSampleDt   = 10.0 / kNumSamples;  // 播放帧间隔 [s]

// ── 全局状态 ──
static RobotModel                    robot;
static Pipe                          viz_pipe;
static Eigen::VectorXd               q;
static IKParams                      ik_params;
static TrajPlanParams                plan_params;
static std::vector<pinocchio::SE3>   ring;      // 闭合圆形轨迹位姿序列

static void sim_send(const std::string& s) { rebot::sim::send_line(&viz_pipe, s); }
static void sim_stop() { rebot::sim::stop(&viz_pipe); }

// 信号处理：Ctrl+C 时清理并退出
static void on_sigint(int) { sim_stop(); std::_Exit(0); }

/**
 * @brief 在圆上采样第 i 个位姿点
 * 
 * 圆在 YZ 平面，从 π/2 开始逆时针采样
 * 
 * @param i 采样索引 [0, n-1]
 * @param n 总采样数
 * @return SE3 位姿（位置 + 恒定姿态）
 */
static pinocchio::SE3 sample_circle_pose(int i, int n)
{
    // 参数方程：从 π/2 开始，逆时针旋转
    const double th = M_PI / 2 + 2.0 * M_PI * i / n;
    return rebot::make_pose(
        kCenterX,
        kCenterY + kRadius * std::cos(th),
        kCenterZ + kRadius * std::sin(th),
        0, 0, 0);  // 姿态保持恒定
}

/**
 * @brief 构建闭合圆形轨迹
 * 
 * 策略：
 *   - 正向采样：[0, kNumSamples]（包含起点和终点，确保闭合）
 *   - 反向采样：[kNumSamples-1, 0]（形成返回路径）
 *   - 最终得到双倍点数的平滑闭环
 */
static void build_ring()
{
    ring.clear();
    // 正向：0 → kNumSamples（含端点）
    for (int i = 0; i <= kNumSamples; ++i)
        ring.push_back(sample_circle_pose(i, kNumSamples));
    // 反向：kNumSamples-1 → 0（形成返程）
    for (int i = kNumSamples - 1; i >= 0; --i)
        ring.push_back(sample_circle_pose(i, kNumSamples));
}

/**
 * @brief 无限循环播放圆形轨迹
 * 
 * 流程：
 *   1. 遍历圆上所有位姿点
 *   2. 逐点求解 IK（使用前一点的解作为初值，加速收敛）
 *   3. 打包为关节轨迹并发送到 MeshCat
 *   4. 重复播放
 */
static void run_trajectory_loop()
{
    int cycle = 0;
    while (true) {
        std::vector<rebot::JointTrajectoryPoint> traj;
        
        // 遍历圆形轨迹上的所有位姿点，逐点求解 IK
        for (const auto& pose : ring) {
            auto ik = rebot::solveIK(robot, pose, q, ik_params);
            q = ik.q;  // 更新状态（作为下一次 IK 的初值）
            traj.push_back({0.0, q, ik.success});
        }
        
        // 打印周期信息
        std::cout << "周期 " << ++cycle << " 点数 " << traj.size() << "\n";
        
        // 发送到 MeshCat 播放
        if (viz_pipe.out)
            sim_send(rebot::sim::pack_traj_json(robot, "circle", kSampleDt, traj));
    }
}

/**
 * @brief 主函数：初始化并启动圆形轨迹演示
 * 
 * 执行流程：
 *   1. 加载机器人模型和参数
 *   2. 启动 MeshCat 可视化
 *   3. 规划从中位配置到圆起点的过渡轨迹
 *   4. 构建闭合圆形轨迹
 *   5. 进入无限循环播放
 * 
 * 命令行参数：
 *   argv[1] - URDF 文件路径（可选）
 */
int main(int argc, char** argv)
{
    // 解析命令行参数
    std::string urdf_path = (argc > 1 && argv[1][0]) ? argv[1] : URDF_PATH;

    // 初始化机器人模型和参数（使用推荐默认值）
    rebot::sim::init_sim_env(urdf_path, robot, q, ik_params, plan_params);

    // 注册信号处理（Ctrl+C 清理退出）
    signal(SIGINT, on_sigint);
    rebot::sim::spawn(&viz_pipe, urdf_path.c_str());

    // 求解圆起点的 IK
    auto ik0 = rebot::solveIK(robot, sample_circle_pose(0, kNumSamples), q, ik_params);
    if (!ik0.success) {
        std::cerr << "起点 IK 失败\n";
        sim_stop();
        return 1;
    }

    // 规划从中位配置到圆起点的过渡轨迹
    auto go = rebot::planJointSpaceTrajectory(
        robot, q, ik0.q, 2.0, plan_params, ik_params);
    if (viz_pipe.out)
        sim_send(rebot::sim::pack_traj_json(robot, "go_start", plan_params.dt, go));

    // 构建闭合圆形轨迹
    build_ring();
    std::cout << "圆心 (" << kCenterX << "," << kCenterY << "," << kCenterZ
              << ") 半径 " << kRadius << " m\n";
    
    // 更新状态并进入循环播放
    q = go.back().q;
    run_trajectory_loop();
    return 0;
}
