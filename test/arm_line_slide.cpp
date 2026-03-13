/**
 * arm_line_slide — 末端直线来回滑动 demo（轨迹同步 + 速度前馈）
 *
 * 流程：
 *   1. 测地线轨迹运动到起始点 (0.36, 0, 0.26, roll=π, pitch=0, yaw=0)
 *   2. 按 SLIDE_VEL 控制末端线速度，在 y=+SLIDE_Y_POS ↔ y=SLIDE_Y_NEG 间逐帧 IK 求解；
 *      每帧计算所需关节角速度 dq，若任意关节超出速度限制则等比缩小步长（轨迹同步），
 *      并通过 set_all_positions_velocities 发送速度前馈以改善跟踪精度。
 *   3. Ctrl+C：停止滑动，缓慢回零点后失能退出
 *
 * 速度控制：修改 SLIDE_VEL（m/s）即可调整末端线速度。
 *
 * 使用: sudo ./arm_line_slide [串口] [YAML] [URDF]
 *       默认: /dev/ttyACM0  <exe_dir>/../config/arm.yaml  <编译时 URDF_PATH>
 */

#include "kinematics/robot_model.h"
#include "kinematics/forward_kinematics.h"
#include "kinematics/inverse_kinematics.h"
#include "kinematics/trajectory_planner_geodesic.h"
#include "actuator/arm_actuator_group.h"
#include "utils/control_loop.h"

#include <Eigen/Core>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <thread>
#include <vector>
#include <limits.h>
#include <unistd.h>

using namespace rebot;
using namespace actuator;

// ─── 频率 ─────────────────────────────────────────────────────────────────────
static constexpr double CTRL_HZ  = 1000.0;
static constexpr double PLAN_HZ  = 100.0;    // IK 求解 + 指令更新频率

// ─── 滑动参数 ─────────────────────────────────────────────────────────────────
static constexpr double SLIDE_X     =  0.36;
static constexpr double SLIDE_Z     =  0.3;
static constexpr double SLIDE_Y_POS =  0.1;
static constexpr double SLIDE_Y_NEG = -0.1;
static constexpr double SLIDE_ROLL  =  M_PI;
static constexpr double SLIDE_VEL   =  0.09;  // m/s，末端线速度；越大越快

// ─── 位置步长限制（轨迹同步核心参数）──────────────────────────────────────────
// 每帧每个关节允许的最大角度增量 [rad]。
// 若某关节增量超出，则所有关节等比缩小步长，保证同步到达。
static constexpr double MAX_DQ_PER_FRAME = 0.06;  // rad/帧，可调

// ─── 信号 ─────────────────────────────────────────────────────────────────────
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

static pinocchio::SE3 makePose(double x, double y, double z,
                               double roll, double pitch, double yaw)
{
    const Eigen::Matrix3d R =
        (Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX())).toRotationMatrix();
    return pinocchio::SE3(R, Eigen::Vector3d(x, y, z));
}

// 位置增量缩放因子：若任一关节 |delta_q[i]| > MAX_DQ_PER_FRAME，
// 返回 < 1 的缩放比，使所有关节等比缩小步长（轨迹同步）。
static double posScale(const Eigen::VectorXd& delta_q)
{
    double scale = 1.0;
    for (int i = 0; i < delta_q.size(); ++i) {
        double absdq = std::abs(delta_q[i]);
        if (absdq > MAX_DQ_PER_FRAME)
            scale = std::min(scale, MAX_DQ_PER_FRAME / absdq);
    }
    return scale;
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

    IKParams ik_params;
    ik_params.max_iter  = 500;
    ik_params.tolerance = 1e-6;
    ik_params.step_size = 0.8;

    TrajPlanParams plan_params;
    plan_params.dt      = 1.0 / PLAN_HZ;
    plan_params.profile = TrajProfile::MIN_JERK;

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
    }

    auto connected = arm.scan_connectivity();
    auto names     = arm.joint_names();
    for (int i = 0; i < N; ++i)
        printf("  [%d] %s: %s\n", i, names[i].c_str(), connected[i] ? "已连接" : "无响应");

    // ── 使能 ──────────────────────────────────────────────────────────────────
    Eigen::VectorXd q = toEigen(arm.get_all_positions());
    if (q.size() > nq) q.conservativeResize(nq);
    printf("[arm] 使能...\n");
    arm.enable_all();
    sleep(1);

    // ── 共享状态：位置目标 + 速度前馈 ────────────────────────────────────────
    std::vector<float> g_targets(N, 0.f);
    std::mutex         g_state_mtx;
    std::atomic<float> g_ctrl_speed_limit{-1.f};

    {
        std::lock_guard<std::mutex> lk(g_state_mtx);
        for (int i = 0; i < N; ++i) g_targets[i] = static_cast<float>(q[i]);
    }

    ControlLoop ctrl_loop(CTRL_HZ, [&]() -> bool {
        std::vector<float> tgt;
        {
            std::lock_guard<std::mutex> lk(g_state_mtx);
            tgt = g_targets;
        }
        float sl = g_ctrl_speed_limit.load(std::memory_order_relaxed);
        if (sl > 0.f)
            arm.set_all_positions_with_speed_limit(tgt, sl);
        else
            arm.set_all_positions(tgt);
        return true;
    });

    // runTraj：按 PLAN_HZ 播放预规划轨迹（仅阶段1和回零，不启用速度前馈）
    auto runTraj = [&](const std::vector<JointTrajectoryPoint>& traj,
                       bool interruptible, float vel_limit = -1.f)
    {
        if (traj.empty()) return;
        g_ctrl_speed_limit.store(vel_limit, std::memory_order_relaxed);
        const int n = std::min(static_cast<int>(traj.front().q.size()), N);
        for (const auto& pt : traj) {
            if (interruptible && !g_running) break;
            {
                std::lock_guard<std::mutex> lk(g_state_mtx);
                for (int i = 0; i < n; ++i)
                    g_targets[i] = static_cast<float>(pt.q[i]);
            }
            std::this_thread::sleep_for(std::chrono::duration<double>(1.0 / PLAN_HZ));
        }
        g_ctrl_speed_limit.store(-1.f, std::memory_order_relaxed);
    };

    // ── 阶段1：测地线到起始点 ─────────────────────────────────────────────────
    printf("[slide] 前往起始点 (%.2f, 0, %.2f)...\n", SLIDE_X, SLIDE_Z);
    const pinocchio::SE3 T_start = makePose(SLIDE_X, 0.0, SLIDE_Z, SLIDE_ROLL, 0, 0);
    IKResult ik_start = solveIK(robot, T_start, q, ik_params);
    if (!ik_start.success) {
        fprintf(stderr, "[错误] 起始点 IK 失败 err=%.2e\n", ik_start.error);
        ctrl_loop.stop(); arm.disable_all(); return 1;
    }
    const double dist0 = (T_start.translation() - computeFK(robot, q).translation()).norm();
    runTraj(planJointSpaceTrajectory(robot, q, ik_start.q,
                std::max(2.0, dist0 / 0.15), plan_params, ik_params, 0.1),
            /*interruptible=*/true);
    if (!g_running) goto cleanup;

    sleep(1);
    q = toEigen(arm.get_all_positions());
    if (q.size() > nq) q.conservativeResize(nq);

    // ── 阶段2：逐帧 IK + 轨迹同步 + 速度前馈 ────────────────────────────────
    //
    // 每帧 y 步长 = SLIDE_VEL * dt（由末端线速度和规划频率决定）
    //
    // 速度同步：dq = (q_next - q_prev) / dt
    //   若某关节超速，等比缩小 y 步长（末端实际速度自动降低），保证所有关节不超限。
    //
    // 速度前馈：dq 通过 set_all_positions_velocities 发送，
    //   POS_VEL 关节直接获得速度指令，消除摩擦引起的跟踪滞后。
    {
        const double dt       = 1.0 / PLAN_HZ;
        const double dy_nom   = SLIDE_VEL * dt;   // 标称 y 步长 [m/帧]

        printf("[slide] 开始来回滑动  y∈[%.3f, %.3f] m  末端速度%.3f m/s"
               "  标称步长%.5f m/帧  （Ctrl+C 停止）\n\n",
               SLIDE_Y_NEG, SLIDE_Y_POS, SLIDE_VEL, dy_nom);

        double y    =  0.0;
        double dir  = -1.0;   // 先向 y_neg
        int log_cnt = 0;

        while (g_running) {
            // 1. 读取真实电机位置作为 IK 初值
            Eigen::VectorXd q_actual = toEigen(arm.get_all_positions());
            if (q_actual.size() > nq) q_actual.conservativeResize(nq);

            // 2. 按标称步长推进 y，到达端点反向
            double step    = dir * dy_nom;
            double y_next  = y + step;
            if (y_next <= SLIDE_Y_NEG) { y_next = SLIDE_Y_NEG; dir =  1.0; }
            if (y_next >= SLIDE_Y_POS) { y_next = SLIDE_Y_POS; dir = -1.0; }

            // 3. IK 求解（以真实电机位置为初值）
            IKResult ik = solveIK(robot,
                makePose(SLIDE_X, y_next, SLIDE_Z, SLIDE_ROLL, 0, 0),
                q_actual, ik_params);
            if (!ik.success) {
                fprintf(stderr, "[警告] IK 失败 y=%.5f err=%.2e，跳过本帧\n",
                        y_next, ik.error);
                std::this_thread::sleep_for(std::chrono::duration<double>(dt));
                continue;
            }

            // 4. 位置增量检查 + 轨迹同步（基于真实位置的增量）
            Eigen::VectorXd delta_q = ik.q - q_actual;
            double scale = posScale(delta_q);
            if (scale < 1.0) {
                y_next = y + step * scale;
                IKResult ik2 = solveIK(robot,
                    makePose(SLIDE_X, y_next, SLIDE_Z, SLIDE_ROLL, 0, 0),
                    q_actual, ik_params);
                if (ik2.success) {
                    ik      = ik2;
                    delta_q = ik.q - q_actual;
                }
            }

            // 5. 更新共享目标位置
            {
                std::lock_guard<std::mutex> lk(g_state_mtx);
                for (int i = 0; i < nq && i < N; ++i)
                    g_targets[i] = static_cast<float>(ik.q[i]);
            }

            y = y_next;

            // 每 50 帧（~1 s）打印精度信息
            if (++log_cnt % 50 == 0) {
                const auto T_fb = computeFK(robot, q_actual);
                const double xz_err = std::hypot(
                    T_fb.translation()[0] - SLIDE_X,
                    T_fb.translation()[2] - SLIDE_Z);
                double max_dq = delta_q.cwiseAbs().maxCoeff();
                printf("  [精度] xz偏差=%.4f m  y指令=%.4f  反馈=%.4f m"
                       "  max|dq|=%.4f rad/帧  scale=%.3f\n",
                       xz_err, y, T_fb.translation()[1], max_dq, scale);
                int n_print = std::min(nq, 6);
                printf("    期望:");
                for (int i = 0; i < n_print; ++i) printf(" %7.4f", ik.q[i]);
                printf("\n    实际:");
                for (int i = 0; i < n_print; ++i) printf(" %7.4f", q_actual[i]);
                printf("\n    差值:");
                for (int i = 0; i < n_print; ++i) printf(" %7.4f", ik.q[i] - q_actual[i]);
                printf("\n");
            }

            std::this_thread::sleep_for(std::chrono::duration<double>(dt));
        }

    }

cleanup:
    // ── 缓慢回零点 ────────────────────────────────────────────────────────────
    printf("\n[arm] 缓慢回到零点...\n");
    q = toEigen(arm.get_all_positions());
    if (q.size() > nq) q.conservativeResize(nq);
    const Eigen::VectorXd q_home = Eigen::VectorXd::Zero(nq);
    double max_disp = 0.0;
    for (int i = 0; i < nq; ++i) max_disp = std::max(max_disp, std::abs(q[i]));
    runTraj(planJointSpaceTrajectory(robot, q, q_home,
                std::max(3.0, max_disp / 0.3), plan_params, ik_params, 0.1),
            /*interruptible=*/false, /*vel_limit=*/0.6f);

    ctrl_loop.stop();
    printf("[arm] 失能\n");
    arm.disable_all();
    return 0;
}
