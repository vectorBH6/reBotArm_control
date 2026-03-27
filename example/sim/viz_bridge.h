#pragma once

/**
 * ============================================================================
 * viz_bridge.h - 仿真可视化桥接工具
 * ============================================================================
 * 
 * 功能：example/sim 与 viewer.py（MeshCat渲染进程）之间的通信层
 * 
 * 核心机制：
 *   - 使用 pipe + fork 创建 Python 子进程
 *   - 父进程（C++）通过管道写入 JSON 指令（一行一条）
 *   - 子进程（viewer.py）解析 JSON 并更新 MeshCat 场景
 * 
 * 依赖说明：
 *   - 仅依赖底层 kinematics 模块（RobotModel、FK、IK、轨迹规划）
 *   - 不依赖 ArmController，可独立用于算法测试
 * 
 * 使用场景：
 *   - 算法开发：在仿真环境中测试 IK、轨迹规划等算法
 *   - 参数调优：可视化不同参数下的轨迹效果
 *   - 教学演示：直观展示机械臂运动学原理
 * ============================================================================
 */

#include "kinematics/robot_model.h"
#include "kinematics/forward_kinematics.h"
#include "kinematics/inverse_kinematics.h"
#include "kinematics/trajectory_planner_geodesic.h"

#include <chrono>
#include <cstdio>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <sys/wait.h>
#include <unistd.h>

namespace rebot {
namespace sim {

// =============================================================================
// 通用仿真参数（推荐默认值）
// =============================================================================

/** IK 求解器默认参数 */
struct DefaultIKParams {
    static constexpr int    MAX_ITER  = 200;    // 最大迭代次数
    static constexpr double TOLERANCE = 1e-4;   // 收敛误差阈值 [m/rad]
    static constexpr double STEP_SIZE = 0.8;    // 梯度下降步长因子
};

/** 轨迹规划器默认参数 */
struct DefaultTrajParams {
    static constexpr double DT = 1.0 / 50.0;  // 采样周期 [s]，对应 50Hz
    // 速度曲线默认为 MIN_JERK（最小加加速度），在 init_default_traj_params 中设置
};

// =============================================================================
// 管道通信结构
// =============================================================================

/**
 * @brief C++ 与 Python 渲染进程的管道封装
 * 
 * 成员：
 *   - out: 输出文件流（C++ 写入 → Python 读取）
 *   - pid: Python 子进程 PID
 */
struct Pipe {
    FILE*   out = nullptr;
    pid_t   pid = -1;
};

// =============================================================================
// 通信管理函数
// =============================================================================

/**
 * @brief 启动 MeshCat 可视化子进程
 * 
 * 工作流程：
 *   1. 创建管道（pipe）
 *   2. fork 创建子进程
 *   3. 子进程执行 viewer.py（通过 execl）
 *   4. 父进程将 URDF 路径作为首行发送给子进程
 *   5. 等待 3 秒让 MeshCat 初始化完成
 * 
 * @param p 管道结构体指针，用于后续通信
 * @param urdf_path URDF 模型文件的绝对路径
 * @return 成功返回 true，失败返回 false
 */
inline bool spawn(Pipe* p, const char* urdf_path)
{
    int fd[2];
    if (::pipe(fd) != 0) return false;
    pid_t child = fork();
    if (child < 0) return false;
    if (child == 0) {
        close(fd[1]);
        dup2(fd[0], STDIN_FILENO);
        close(fd[0]);
        std::string py = std::string(PROJECT_SOURCE_DIR) + "/example/sim/viewer.py";
        execl(PYTHON_BIN, PYTHON_BIN, py.c_str(), nullptr);
        _exit(127);
    }
    close(fd[0]);
    p->out = fdopen(fd[1], "w");
    p->pid = child;
    if (!p->out) return false;
    
    // 发送 URDF 路径作为首行（viewer.py 协议要求）
    fprintf(p->out, "%s\n", urdf_path);
    fflush(p->out);
    
    // 等待 MeshCat 启动和浏览器打开
    std::this_thread::sleep_for(std::chrono::seconds(3));
    return true;
}

/**
 * @brief 停止可视化子进程并清理资源
 * 
 * 工作流程：
 *   1. 发送 {"cmd":"exit"} 指令通知子进程退出
 *   2. 关闭管道文件流
 *   3. 等待子进程结束（waitpid）
 * 
 * @param p 管道结构体指针
 */
inline void stop(Pipe* p)
{
    if (!p || !p->out) return;
    fputs("{\"cmd\":\"exit\"}\n", p->out);
    fflush(p->out);
    fclose(p->out);
    p->out = nullptr;
    if (p->pid > 0) {
        waitpid(p->pid, nullptr, 0);
        p->pid = -1;
    }
}

/**
 * @brief 向子进程发送一行 JSON 指令
 * 
 * @param p 管道结构体指针
 * @param line JSON 字符串（自动添加换行符）
 */
inline void send_line(Pipe* p, const std::string& line)
{
    if (!p || !p->out) return;
    fputs(line.c_str(), p->out);
    if (line.empty() || line.back() != '\n') fputc('\n', p->out);
    fflush(p->out);
}

// =============================================================================
// JSON 数据打包函数
// =============================================================================

/**
 * @brief 打包 IK 结果为 JSON（用于单帧位姿可视化）
 * 
 * JSON 格式：
 *   {
 *     "target": {"xyz": [x,y,z], "R": [[r11,r12,r13],[r21,r22,r23],[r31,r32,r33]]},
 *     "q": [q1, q2, ..., qn]
 *   }
 * 
 * viewer.py 行为：
 *   - 在场景中显示目标坐标系（三色轴 + 球体标记）
 *   - 更新机械臂关节角到 q
 * 
 * @param T 目标位姿（SE3）
 * @param q 对应的关节角解
 * @return JSON 字符串（含尾部换行符）
 */
inline std::string pack_ik_json(const pinocchio::SE3& T, const Eigen::VectorXd& q)
{
    std::ostringstream ss;
    ss << std::setprecision(9);
    
    // 提取位置和旋转矩阵
    const auto& p = T.translation();
    const Eigen::Matrix3d& R = T.rotation();
    
    // 构建 JSON：target 对象（xyz + 3x3旋转矩阵）
    ss << "{\"target\":{\"xyz\":[" << p[0] << "," << p[1] << "," << p[2] << "],\"R\":[";
    for (int i = 0; i < 3; ++i) {
        if (i) ss << ',';
        ss << "[" << R(i, 0) << "," << R(i, 1) << "," << R(i, 2) << "]";
    }
    
    // 构建 JSON：q 数组（关节角）
    ss << "]},\"q\":[";
    for (int i = 0; i < q.size(); ++i) {
        if (i) ss << ',';
        ss << q[i];
    }
    ss << "]}\n";
    return ss.str();
}

/**
 * @brief 打包关节轨迹为 JSON（用于轨迹动画播放）
 * 
 * JSON 格式：
 *   {
 *     "name": "轨迹名称",
 *     "dt": 时间步长,
 *     "q_list": [[q1_t0, q2_t0, ...], [q1_t1, q2_t1, ...], ...],
 *     "path": [[x0,y0,z0], [x1,y1,z1], ...]
 *   }
 * 
 * viewer.py 行为：
 *   - 按 dt 时间步长逐帧播放关节角
 *   - 绘制末端参考路径（灰色线）和已走路径（绿色线）
 * 
 * @param robot 机器人模型（用于计算末端轨迹）
 * @param name 轨迹名称（便于调试输出）
 * @param dt 采样时间步长 [秒]
 * @param traj 关节轨迹点序列
 * @return JSON 字符串（含尾部换行符）
 */
inline std::string pack_traj_json(RobotModel&                             robot,
                                  const std::string&                      name,
                                  double                                  dt,
                                  const std::vector<JointTrajectoryPoint>& traj)
{
    std::ostringstream ss;
    ss << std::setprecision(6) << std::fixed;
    
    // 构建 JSON 头部（name 和 dt）
    ss << "{\"name\":\"" << name << "\",\"dt\":" << dt << ",\"q_list\":[";
    
    // 填充关节角序列
    for (size_t i = 0; i < traj.size(); ++i) {
        if (i) ss << ',';
        ss << '[';
        for (int j = 0; j < traj[i].q.size(); ++j) {
            if (j) ss << ',';
            ss << traj[i].q[j];
        }
        ss << ']';
    }
    
    // 计算并填充末端位置路径（通过正运动学）
    ss << "],\"path\":[";
    for (size_t i = 0; i < traj.size(); ++i) {
        if (i) ss << ',';
        const auto& p = computeFK(robot, traj[i].q).translation();
        ss << '[' << p[0] << ',' << p[1] << ',' << p[2] << ']';
    }
    ss << "]}\n";
    return ss.str();
}

// =============================================================================
// 通用初始化工具（减少各示例的重复代码）
// =============================================================================

/**
 * @brief 初始化 IK 参数为推荐默认值
 * 
 * @param params IK 参数结构体引用
 */
inline void init_default_ik_params(IKParams& params) {
    params.max_iter  = DefaultIKParams::MAX_ITER;
    params.tolerance = DefaultIKParams::TOLERANCE;
    params.step_size = DefaultIKParams::STEP_SIZE;
}

/**
 * @brief 初始化轨迹规划参数为推荐默认值
 * 
 * @param params 轨迹规划参数结构体引用
 */
inline void init_default_traj_params(TrajPlanParams& params) {
    params.dt      = DefaultTrajParams::DT;
    params.profile = TrajProfile::MIN_JERK;
}

/**
 * @brief 一键初始化仿真环境（模型 + 参数）
 * 
 * 适用场景：快速搭建仿真测试环境，减少样板代码
 * 
 * @param urdf_path URDF 文件路径
 * @param robot 输出：机器人模型
 * @param q 输出：初始关节角（设为中位配置）
 * @param ik_params 输出：IK 参数（设为默认值）
 * @param plan_params 输出：轨迹规划参数（设为默认值）
 */
inline void init_sim_env(const std::string& urdf_path,
                         RobotModel& robot,
                         Eigen::VectorXd& q,
                         IKParams& ik_params,
                         TrajPlanParams& plan_params)
{
    robot = RobotModel(urdf_path);
    q = robot.neutralConfig();
    init_default_ik_params(ik_params);
    init_default_traj_params(plan_params);
}

}  // namespace sim
}  // namespace rebot
