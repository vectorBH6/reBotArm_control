// =============================================================================
// 交互式逆运动学（IK）可视化示例
// =============================================================================
//
// 功能：
//   - 终端输入目标位姿（xyz + 可选的 roll/pitch/yaw）
//   - 使用阻尼最小二乘法求解 IK
//   - 在 MeshCat 中实时显示目标坐标系和机械臂姿态
//
// 使用方法：
//   ./ik_viz [urdf_path]
//   输入格式：x y z [roll pitch yaw]  (米 / 弧度)
//   输入 'q' 退出
//
// 算法说明：
//   - 采用 CLIK（闭环逆运动学）算法，基于雅可比伪逆
//   - 支持自动重试（不同初值）以提高求解成功率
//   - 无轨迹插值，直接跳转到目标关节角
//
// 适用场景：
//   - 测试工作空间可达性
//   - 调试 IK 参数（迭代次数、收敛阈值、步长）
//   - 教学演示：理解正运动学与逆运动学的关系
//
// =============================================================================

#include <iostream>
#include <sstream>
#include <string>

#include "viz_bridge.h"

using rebot::RobotModel;
using rebot::IKParams;
using rebot::IKResult;
using rebot::TrajPlanParams;
using rebot::sim::Pipe;

// ── 全局状态 ──
static RobotModel      robot;       // 机器人运动学模型
static Pipe            viz_pipe;    // 与 viewer.py 的通信管道
static Eigen::VectorXd q;           // 当前关节角配置
static IKParams        ik_params;   // IK 求解器参数

// ── 管道操作简化封装 ──
static bool sim_spawn(const char* urdf) { return rebot::sim::spawn(&viz_pipe, urdf); }
static void sim_send(const std::string& s) { rebot::sim::send_line(&viz_pipe, s); }
static void sim_stop() { rebot::sim::stop(&viz_pipe); }

/**
 * @brief 交互式 IK 求解主循环
 * 
 * 流程：
 *   1. 从终端读取目标位姿（x y z 必选，roll pitch yaw 可选）
 *   2. 调用 solveIKWithRetry 求解（自动尝试多个初值）
 *   3. 打印求解结果（成功标志、迭代次数、误差、关节角）
 *   4. 发送 JSON 到 viewer.py 更新可视化
 */
static void run_interactive()
{
    std::cout << "\n输入: x y z [roll pitch yaw] (米 / 弧度)，q 退出\n\n";
    std::string line;
    
    while (std::getline(std::cin, line)) {
        // 跳过空行
        if (line.find_first_not_of(" \t") == std::string::npos) continue;
        
        // 退出指令
        if (line == "q" || line == "quit" || line == "exit") break;

        // 解析输入（xyz 必选，rpy 可选）
        double x = 0, y = 0, z = 0, roll = 0, pitch = 0, yaw = 0;
        std::istringstream iss(line);
        if (!(iss >> x >> y >> z)) {
            std::cerr << "  格式: x y z [roll pitch yaw]\n";
            continue;
        }
        iss >> roll >> pitch >> yaw;

        // 构建目标位姿（ZYX 欧拉角）
        pinocchio::SE3 target = rebot::make_pose(x, y, z, roll, pitch, yaw);
        
        // 求解 IK（带自动重试）
        IKResult res = rebot::solveIKWithRetry(robot, target, q, ik_params);

        // 打印求解结果
        std::cout << "  IK success=" << res.success << " iters=" << res.iterations
                  << " err=" << res.error << "\n  q=" << res.q.transpose() << "\n";
        if (!res.success)
            std::cerr << "  警告: 未完全收敛，显示最佳近似解\n";

        // 发送到可视化
        sim_send(rebot::sim::pack_ik_json(target, res.q));
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
    TrajPlanParams unused_plan_params;  // init_sim_env 需要此参数
    rebot::sim::init_sim_env(urdf_path, robot, q, ik_params, unused_plan_params);

    // 启动 MeshCat 可视化
    if (!sim_spawn(urdf_path.c_str())) {
        std::cerr << "MeshCat 启动失败\n";
        return 1;
    }
    
    // 进入交互式求解循环
    run_interactive();
    
    // 清理：停止子进程
    sim_stop();
    return 0;
}
