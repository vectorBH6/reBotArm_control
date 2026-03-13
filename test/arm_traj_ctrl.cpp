/**
 * arm_traj_ctrl — 实机测地线轨迹控制
 *
 * 终端输入末端期望 "x y z [roll pitch yaw (rad)]"，机械臂执行测地线轨迹，
 * 到达后打印末端位姿和关节角度；Ctrl+C 缓慢回零点后失能退出。
 *
 * 使用: sudo ./arm_traj_ctrl [串口] [YAML] [URDF]
 *       默认: /dev/ttyACM0  <exe_dir>/../config/arm.yaml  <编译时 URDF_PATH>
 *
 * 控制频率设计：
 *   CTRL_HZ  — 控制指令发送频率目标（实际受串口 USB-CAN 吞吐限制）
 *   PLAN_HZ  — 轨迹规划点播放频率（决定轨迹 dt）
 *   两者解耦：规划线程按 PLAN_HZ 更新目标角度，控制线程以最大吞吐持续发送指令。
 */

#include "kinematics/robot_model.h"
#include "kinematics/forward_kinematics.h"
#include "kinematics/inverse_kinematics.h"
#include "kinematics/trajectory_planner_geodesic.h"
#include "actuator/arm_actuator_group.h"
#include "utils/control_loop.h"

#include <Eigen/Core>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <limits.h>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace rebot;
using namespace actuator;

// ─── 频率常量（修改此处即可调整控制/规划频率）───────────────────────────────
// 控制指令发送频率目标 [Hz]。实际频率受 USB-CAN 串行往返延迟限制
//（7 关节约 150-200 Hz），设为 1000 使控制线程以最大吞吐运行。
static constexpr double CTRL_HZ = 1000.0;
// 轨迹规划点播放频率 [Hz]，决定轨迹规划的 dt = 1/PLAN_HZ。
static constexpr double PLAN_HZ = 50.0;

// ─── 全局运行标志 ──────────────────────────────────────────────────────────────
static volatile bool g_running = true;
static void on_signal(int) { g_running = false; }

// ─── 工具函数 ─────────────────────────────────────────────────────────────────

static std::string exe_parent_dir()
{
    char buf[PATH_MAX]{};
    ssize_t n = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (n <= 0) return ".";
    std::string p(buf, n);
    p = p.substr(0, p.rfind('/'));
    return p.substr(0, p.rfind('/'));
}

static Eigen::VectorXd toEigen(const std::vector<float>& v)
{
    Eigen::VectorXd q(v.size());
    for (size_t i = 0; i < v.size(); ++i) q[i] = v[i];
    return q;
}

static void printState(RobotModel& robot, const Eigen::VectorXd& q)
{
    const auto T = computeFK(robot, q);
    const auto& p = T.translation();
    const Eigen::Vector3d rpy = T.rotation().eulerAngles(2, 1, 0).reverse();
    printf("  末端: pos[%.4f  %.4f  %.4f]  rpy[%.4f  %.4f  %.4f]\n",
           p[0], p[1], p[2], rpy[0], rpy[1], rpy[2]);
    printf("  关节 [rad]:");
    for (int i = 0; i < q.size(); ++i) printf("  %7.4f", q[i]);
    printf("\n");
}

// ─── 主程序 ───────────────────────────────────────────────────────────────────
int main(int argc, char* argv[])
{
    const std::string root = exe_parent_dir();
    std::string dev       = "/dev/ttyACM0";
    std::string yaml_path = root + "/config/arm.yaml";
    std::string urdf_path = URDF_PATH;
    if (argc > 1) dev       = argv[1];
    if (argc > 2) yaml_path = argv[2];
    if (argc > 3) urdf_path = argv[3];

    // 使用 sigaction 禁用 SA_RESTART，让 Ctrl+C 打断阻塞的 getline
    struct sigaction sa{};
    sa.sa_handler = on_signal;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT,  &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);

    // ── 加载运动学模型 ────────────────────────────────────────────────────────
    printf("[model] %s\n", urdf_path.c_str());
    RobotModel robot(urdf_path);
    const int nq = robot.nq();

    TrajPlanParams params;
    params.dt      = 1.0 / PLAN_HZ;  // 轨迹点间隔与规划播放频率一致
    params.profile = TrajProfile::MIN_JERK;

    IKParams ik_params;
    ik_params.max_iter  = 200;
    ik_params.tolerance = 1e-4;
    ik_params.step_size = 0.8;

    // ── 连接硬件 ──────────────────────────────────────────────────────────────
    printf("[arm] 连接 %s ...\n", dev.c_str());
    ArmActuatorGroup arm = [&]() -> ArmActuatorGroup {
        try { return ArmActuatorGroup::from_yaml(dev, yaml_path); }
        catch (const std::exception& e) {
            fprintf(stderr, "[错误] 初始化失败: %s\n", e.what());
            std::exit(1);
        }
    }();

    const int N = static_cast<int>(arm.size());
    if (N < nq) {
        fprintf(stderr, "[错误] 执行器数(%d) 少于 URDF 关节数(%d)\n", N, nq);
        return 1;
    } else if (N > nq) {
        fprintf(stderr, "[警告] 执行器数(%d) 多于 URDF 关节数(%d)，仅前 %d 个关节参与运动学和轨迹控制，其余关节将保持当前位置。\n",
                N, nq, nq);
    }

    auto connected = arm.scan_connectivity();
    auto names     = arm.joint_names();
    for (int i = 0; i < N; ++i)
        printf("  [%d] %s: %s\n", i, names[i].c_str(), connected[i] ? "已连接" : "无响应");

    // ── 使能（先读当前位置，使能后立即锁住原位，防止突跳）────────────────────
    Eigen::VectorXd q = toEigen(arm.get_all_positions());
    if (q.size() > nq) q.conservativeResize(nq);
    printf("[arm] 使能...\n");
    arm.enable_all();
    sleep(1);

    // ── 初始化共享目标（控制线程和规划线程均通过此访问）──────────────────────
    // g_targets: 控制线程持续发送此目标；runTraj 按 PLAN_HZ 更新此值
    // g_ctrl_speed_limit: < 0 表示使用各关节配置的控制模式；> 0 强制速度上限
    std::vector<float>    g_targets(N);
    std::mutex            g_targets_mtx;
    std::atomic<float>    g_ctrl_speed_limit{-1.f};

    {
        std::lock_guard<std::mutex> lock(g_targets_mtx);
        for (int i = 0; i < N; ++i) g_targets[i] = static_cast<float>(q[i]);
    }

    // ── 启动控制线程（持续以 CTRL_HZ 目标频率发送指令）──────────────────────
    ControlLoop ctrl_loop(CTRL_HZ, [&]() -> bool {
        std::vector<float> targets;
        {
            std::lock_guard<std::mutex> lock(g_targets_mtx);
            targets = g_targets;
        }
        float sl = g_ctrl_speed_limit.load(std::memory_order_relaxed);
        if (sl > 0.f)
            arm.set_all_positions_with_speed_limit(targets, sl);
        else
            arm.set_all_positions(targets);
        return true;  // 始终运行，由外部调用 stop() 终止
    });

    printf("[arm] 就绪  输入: x y z [roll pitch yaw(rad)]  Ctrl+C 退出\n\n");

    // ── runTraj: 按 PLAN_HZ 更新目标角度（控制线程负责实际发送）────────────
    // vel_limit > 0 时强制所有关节使用 POS_VEL 模式并限制速度（用于回零）。
    auto runTraj = [&](const std::vector<JointTrajectoryPoint>& traj,
                       bool interruptible, float vel_limit = -1.f)
    {
        if (traj.empty()) return;
        g_ctrl_speed_limit.store(vel_limit, std::memory_order_relaxed);
        const int N_ctrl = static_cast<int>(traj.front().q.size());
        const int n = std::min(N_ctrl, N);
        for (const auto& pt : traj) {
            if (interruptible && !g_running) break;
            {
                std::lock_guard<std::mutex> lock(g_targets_mtx);
                for (int i = 0; i < n; ++i)
                    g_targets[i] = static_cast<float>(pt.q[i]);
            }
            std::this_thread::sleep_for(
                std::chrono::duration<double>(1.0 / PLAN_HZ));
        }
        g_ctrl_speed_limit.store(-1.f, std::memory_order_relaxed);
    };

    // ── 主控制循环 ────────────────────────────────────────────────────────────
    std::string line;
    while (g_running) {
        const auto T_cur = computeFK(robot, q);
        const auto& p    = T_cur.translation();
        const Eigen::Vector3d rpy = T_cur.rotation().eulerAngles(2, 1, 0).reverse();
        printf("当前 pos[%.3f %.3f %.3f] rpy[%.3f %.3f %.3f]> ",
               p[0], p[1], p[2], rpy[0], rpy[1], rpy[2]);
        fflush(stdout);

        if (!std::getline(std::cin, line) || !g_running) break;
        { size_t s = line.find_first_not_of(" \t"); if (s != std::string::npos) line = line.substr(s); }
        if (line.empty()) continue;

        std::istringstream iss(line);
        double x, y, z, ro = 0, pi_v = 0, ya = 0;
        if (!(iss >> x >> y >> z)) { fprintf(stderr, "格式: x y z [roll pitch yaw]\n"); continue; }
        iss >> ro >> pi_v >> ya;

        const Eigen::Matrix3d R =
            (Eigen::AngleAxisd(ya,   Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(pi_v, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(ro,   Eigen::Vector3d::UnitX())).toRotationMatrix();
        const pinocchio::SE3 T_tgt(R, Eigen::Vector3d(x, y, z));

        // IK
        IKResult ik = solveIK(robot, T_tgt, q, ik_params);
        if (!ik.success) { fprintf(stderr, "IK 失败 err=%.2e\n", ik.error); continue; }

        printf("目标关节 [rad]:");
        for (int i = 0; i < ik.q.size(); ++i)
            printf("  %7.4f", ik.q[i]);
        printf("\n是否执行该运动? [y/N]: ");
        fflush(stdout);
        std::string yn;
        if (!std::getline(std::cin, yn)) break;
        { size_t s = yn.find_first_not_of(" \t"); if (s != std::string::npos) yn = yn.substr(s); }
        if (yn.empty() || (yn[0] != 'y' && yn[0] != 'Y')) {
            printf("已取消本次运动。\n\n");
            continue;
        }

        // 运动时长：按位移/旋转距离自适应
        Eigen::Quaterniond qa(T_cur.rotation()), qb(R);
        if (qa.dot(qb) < 0) qb.coeffs() = -qb.coeffs();
        const double ang_dist = 2.0 * std::acos(std::min(1.0, std::abs((qb * qa.inverse()).w())));
        const double duration = std::max(0.5,
            std::max((T_tgt.translation() - p).norm() / 0.2, ang_dist / 1.0));

        // 规划并执行（runTraj 仅更新目标，控制线程发送指令）
        auto traj = planJointSpaceTrajectory(robot, q, ik.q, duration, params, ik_params, 0.1);
        if (traj.empty()) { fprintf(stderr, "轨迹规划失败\n"); continue; }
        printf("规划完成 %zu 点 时长=%.2fs  执行中...\n", traj.size(), duration);

        runTraj(traj, /*interruptible=*/true);

        // 等待电机收敛后读取反馈位置
        sleep(5);
        q = toEigen(arm.get_all_positions());
        if (q.size() > nq) q.conservativeResize(nq);
        if (!g_running) break;

        printf("到达:\n");
        printState(robot, q);
        printf("\n");
    }

    // ── 缓慢回零点 ────────────────────────────────────────────────────────────
    printf("\n[arm] 缓慢回到零点...\n");
    q = toEigen(arm.get_all_positions());
    if (q.size() > nq) q.conservativeResize(nq);
    const Eigen::VectorXd q_home = Eigen::VectorXd::Zero(nq);
    double max_disp = 0.0;
    for (int i = 0; i < nq; ++i) max_disp = std::max(max_disp, std::abs(q[i]));
    const double home_dur = std::max(3.0, max_disp / 0.3);

    auto home_traj = planJointSpaceTrajectory(robot, q, q_home, home_dur, params, ik_params, 0.1);
    runTraj(home_traj, /*interruptible=*/false, /*vel_limit=*/0.6f);

    // ── 停止控制线程后失能 ────────────────────────────────────────────────────
    ctrl_loop.stop();
    printf("[arm] 失能\n");
    arm.disable_all();
    return 0;
}
