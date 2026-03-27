// =============================================================================
// 圆锥姿态轨迹仿真演示
// =============================================================================
//
// 功能：
//   - 固定末端位置，改变末端姿态形成圆锥式扫描
//   - 使用笛卡尔测地线规划连接关键姿态帧
//   - 逐点 IK 求解后在 MeshCat 中循环播放
//
// 使用方法：
//   ./traj_sim_cone [urdf_path]
//   程序自动运行，按 Ctrl+C 退出
//
// 算法说明：
//   - 关键帧定义：4个不同姿态的关键位姿 + 回到起点（闭环）
//   - 笛卡尔插值：使用 planCartesianGeodesicTrajectory 在 SE3 流形上平滑插值
//   - IK 跟踪：沿插值轨迹逐点求解关节角
//
// 适用场景：
//   - 测试姿态空间的灵活性
//   - 验证测地线插值算法（旋转插值质量）
//   - 演示固定位置的姿态变换能力
//
// 参数说明：
//   - kRefX/Y/Z：固定的末端位置 [米]
//   - kSegPtsPerEdge：每段关键帧之间的插值点数
//   - kLoopPeriodSec：完整一圈的时间 [秒]
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
using rebot::CartesianTrajectory;
using rebot::sim::Pipe;

// ── 圆锥轨迹参数 ──
static constexpr double kRefX          = 0.3;      // 固定位置 X [m]
static constexpr double kRefY          = 0.0;      // 固定位置 Y [m]
static constexpr double kRefZ          = 0.0;      // 固定位置 Z [m]
static constexpr int    kSegPtsPerEdge = 64;       // 每段插值点数
static constexpr double kLoopPeriodSec = 5.0;      // 完整循环时长 [s]

// ── 全局状态 ──
static RobotModel          robot;
static Pipe                viz_pipe;
static Eigen::VectorXd     q;
static IKParams            ik_params;
static TrajPlanParams      plan_params;
static CartesianTrajectory cart;        // 笛卡尔空间轨迹
static double              traj_dt = 0.0;

static void sim_send(const std::string& s) { rebot::sim::send_line(&viz_pipe, s); }
static void sim_stop() { rebot::sim::stop(&viz_pipe); }

// 信号处理：Ctrl+C 时清理并退出
static void on_sigint(int) { sim_stop(); std::_Exit(0); }

/**
 * @brief 构建笛卡尔空间闭合轨迹
 * 
 * 工作流程：
 *   - 将关键帧序列分段，每两个关键帧之间用测地线插值
 *   - 拼接所有段形成完整的笛卡尔轨迹
 *   - 时间均匀分配（总时长 / 段数）
 * 
 * @param keyframes 关键位姿序列（首尾相同可形成闭环）
 * @param plan 轨迹规划参数（dt、速度曲线等）
 */
static void build_cartesian_loop(const std::vector<pinocchio::SE3>& keyframes,
                                 TrajPlanParams plan)
{
    const int    n     = static_cast<int>(keyframes.size()) - 1;  // 段数
    const double seg_t = kLoopPeriodSec / n;  // 每段时长
    cart = CartesianTrajectory{};
    double t0 = 0.0;
    
    // 逐段规划并拼接
    for (int s = 0; s < n; ++s) {
        // 在 SE3 流形上进行测地线插值
        auto seg = rebot::planCartesianGeodesicTrajectory(
            keyframes[s], keyframes[s + 1], seg_t, plan);
        
        // 拼接到总轨迹（跳过重复的首点，除了第一段）
        for (size_t i = (cart.empty() ? 0 : 1); i < seg.points().size(); ++i)
            cart.addPoint(t0 + seg.points()[i].time, seg.points()[i].pose);
        t0 += seg_t;
    }
}

/**
 * @brief 无限循环播放圆锥轨迹
 * 
 * 流程：
 *   1. 遍历笛卡尔轨迹上的所有位姿点
 *   2. 逐点求解 IK（使用前一点的解作为初值）
 *   3. 打包为关节轨迹并发送到 MeshCat
 *   4. 重复播放
 */
static void run_trajectory_loop()
{
    int cycle = 0;
    while (true) {
        std::vector<rebot::JointTrajectoryPoint> traj;
        
        // 沿笛卡尔轨迹逐点求解 IK
        for (const auto& pt : cart.points()) {
            auto ik = rebot::solveIK(robot, pt.pose, q, ik_params);
            q = ik.q;  // 更新状态（作为下一次初值）
            traj.push_back({pt.time, q, ik.success});
        }
        
        // 打印周期信息
        std::cout << "周期 " << ++cycle << " 点数 " << traj.size() << "\n";
        
        // 发送到 MeshCat 播放
        if (viz_pipe.out)
            sim_send(rebot::sim::pack_traj_json(robot, "cone", traj_dt, traj));
    }
}

/**
 * @brief 主函数：初始化并启动圆锥轨迹演示
 * 
 * 执行流程：
 *   1. 加载机器人模型和参数
 *   2. 定义圆锥关键姿态帧（4个方向 + 回到起点）
 *   3. 启动 MeshCat 可视化
 *   4. 规划从中位配置到起点的过渡轨迹
 *   5. 构建笛卡尔闭合轨迹
 *   6. 进入无限循环播放
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

    // 定义圆锥关键姿态帧（固定位置，姿态变化）
    std::vector<pinocchio::SE3> keyframes = {
        rebot::make_pose(kRefX, kRefY, kRefZ, 0.00, 1.14, 0.00),      // 姿态1
        rebot::make_pose(kRefX, kRefY, kRefZ, -1.57, 1.14, -1.57),    // 姿态2（-90°）
        rebot::make_pose(kRefX, kRefY, kRefZ, 0.00, 2.00, 0.00),      // 姿态3（更大俯仰）
        rebot::make_pose(kRefX, kRefY, kRefZ, 1.57, 1.14, 1.57),      // 姿态4（+90°）
        rebot::make_pose(kRefX, kRefY, kRefZ, 0.00, 1.14, 0.00),      // 回到起点（闭环）
    };

    // 求解起点 IK
    auto ik0 = rebot::solveIK(robot, keyframes[0], q, ik_params);
    if (!ik0.success) {
        std::cerr << "起点 IK 失败\n";
        sim_stop();
        return 1;
    }

    // 规划从中位配置到起点的过渡轨迹
    auto go = rebot::planJointSpaceTrajectory(
        robot, q, ik0.q, 2.0, plan_params, ik_params);
    if (viz_pipe.out)
        sim_send(rebot::sim::pack_traj_json(robot, "go_start", plan_params.dt, go));

    // 调整 dt 以适配总周期时长
    TrajPlanParams seg_plan = plan_params;
    seg_plan.dt = (kLoopPeriodSec / (static_cast<int>(keyframes.size()) - 1)) / kSegPtsPerEdge;
    traj_dt = seg_plan.dt;
    
    // 构建笛卡尔闭合轨迹
    build_cartesian_loop(keyframes, seg_plan);

    std::cout << "定点 (" << kRefX << "," << kRefY << "," << kRefZ << ") 圆锥姿态循环\n";
    
    // 更新状态并进入循环播放
    q = go.back().q;
    run_trajectory_loop();
    return 0;
}
