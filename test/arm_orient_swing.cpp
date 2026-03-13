/**
 * arm_orient_swing — 末端定点姿态摆动 demo（闭环等待推进）
 *
 * 流程：
 *   1. 测地线轨迹运动到起始姿态 (0.4, 0, 0.4, roll=π, pitch=0, yaw=0)
 *   2. 在 pitch=-0.3 ↔ +0.3 间按固定步长推进；
 *      每步发出 IK 指令后等待电机追上（关节误差 < SETTLE_THRESH），
 *      再推进下一步。超过 STEP_TIMEOUT_MS 未到位则强制推进。
 *   3. Ctrl+C：缓慢回零点后失能退出
 *
 * 使用: sudo ./arm_orient_swing [串口] [YAML] [URDF]
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
static constexpr double CTRL_HZ = 1000.0;
static constexpr double PLAN_HZ = 50.0;

// ─── 摆动参数 ─────────────────────────────────────────────────────────────────
static constexpr double SWING_X      =  0.4;
static constexpr double SWING_Y      =  0.0;
static constexpr double SWING_Z      =  0.4;
static constexpr double SWING_ROLL   =  M_PI;
static constexpr double PITCH_A      = -0.3;   // rad，一端
static constexpr double PITCH_B      =  0.3;   // rad，另一端

// ─── 闭环等待参数 ─────────────────────────────────────────────────────────────
// 每步 pitch 增量 [rad]
static constexpr double PITCH_STEP   = 0.006;
// 关节到位阈值：所有关节误差的最大绝对值 < 此值视为到位 [rad]
static constexpr double SETTLE_THRESH = 0.02;
// 单步等待上限 [ms]：超时后强制推进下一步
static constexpr int    STEP_TIMEOUT_MS = 100;
// 等待轮询间隔 [ms]
static constexpr int    POLL_MS = 2;

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
                               double roll, double pitch, double yaw = 0.0)
{
    const Eigen::Matrix3d R =
        (Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX())).toRotationMatrix();
    return pinocchio::SE3(R, Eigen::Vector3d(x, y, z));
}

static double maxAbsError(const Eigen::VectorXd& a, const Eigen::VectorXd& b)
{
    return (a - b).cwiseAbs().maxCoeff();
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

    // runTraj：按 PLAN_HZ 播放预规划轨迹（仅用于阶段1和回零，不启用速度前馈）
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

    // ── 阶段1：测地线到起始姿态 ───────────────────────────────────────────────
    printf("[swing] 前往起始点 (%.2f, %.2f, %.2f, pitch=0)...\n",
           SWING_X, SWING_Y, SWING_Z);
    IKResult ik_start = solveIK(robot,
        makePose(SWING_X, SWING_Y, SWING_Z, SWING_ROLL, 0.0), q, ik_params);
    if (!ik_start.success) {
        fprintf(stderr, "[错误] 起始点 IK 失败 err=%.2e\n", ik_start.error);
        ctrl_loop.stop(); arm.disable_all(); return 1;
    }
    const double dist0 = (Eigen::Vector3d(SWING_X, SWING_Y, SWING_Z)
                          - computeFK(robot, q).translation()).norm();
    runTraj(planJointSpaceTrajectory(robot, q, ik_start.q,
                std::max(2.0, dist0 / 0.15), plan_params, ik_params, 0.1),
            /*interruptible=*/true);
    if (!g_running) goto cleanup;

    sleep(1);
    q = toEigen(arm.get_all_positions());
    if (q.size() > nq) q.conservativeResize(nq);

    // ── 阶段2：逐步 IK + 闭环等待 ───────────────────────────────────────────
    {
        const Eigen::Vector3d pos_ref(SWING_X, SWING_Y, SWING_Z);

        printf("[swing] 开始摆动  范围[%.2f, %.2f] rad  步长=%.4f rad"
               "  到位阈值=%.4f rad  超时=%d ms  （Ctrl+C 停止）\n\n",
               PITCH_A, PITCH_B, PITCH_STEP, SETTLE_THRESH, STEP_TIMEOUT_MS);

        double pitch = 0.0;
        double dir   = -1.0;
        int    step_cnt = 0;

        while (g_running) {
            // 1. 推进 pitch
            double pitch_next = pitch + dir * PITCH_STEP;
            if (pitch_next <= PITCH_A) { pitch_next = PITCH_A; dir =  1.0; }
            if (pitch_next >= PITCH_B) { pitch_next = PITCH_B; dir = -1.0; }

            // 2. 读取实际位置，求解 IK
            Eigen::VectorXd q_actual = toEigen(arm.get_all_positions());
            if (q_actual.size() > nq) q_actual.conservativeResize(nq);

            IKResult ik = solveIK(robot,
                makePose(SWING_X, SWING_Y, SWING_Z, SWING_ROLL, pitch_next),
                q_actual, ik_params);
            if (!ik.success) {
                fprintf(stderr, "[警告] IK 失败 pitch=%.4f err=%.2e，跳过\n",
                        pitch_next, ik.error);
                pitch = pitch_next;
                continue;
            }

            // 3. 发送 IK 目标
            {
                std::lock_guard<std::mutex> lk(g_state_mtx);
                for (int i = 0; i < nq && i < N; ++i)
                    g_targets[i] = static_cast<float>(ik.q[i]);
            }

            // 4. 闭环等待：轮询直到关节到位或超时
            auto t0 = std::chrono::steady_clock::now();
            double settle_err = 0.0;
            bool settled = false;
            while (g_running) {
                std::this_thread::sleep_for(std::chrono::milliseconds(POLL_MS));
                Eigen::VectorXd q_now = toEigen(arm.get_all_positions());
                if (q_now.size() > nq) q_now.conservativeResize(nq);
                settle_err = maxAbsError(ik.q, q_now);
                if (settle_err < SETTLE_THRESH) { settled = true; break; }
                auto elapsed = std::chrono::steady_clock::now() - t0;
                if (std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count()
                    >= STEP_TIMEOUT_MS) break;
            }

            pitch = pitch_next;

            // 5. 定期打印
            if (++step_cnt % 20 == 0) {
                Eigen::VectorXd q_fb = toEigen(arm.get_all_positions());
                if (q_fb.size() > nq) q_fb.conservativeResize(nq);
                const auto T_fb = computeFK(robot, q_fb);
                const double pos_err = (T_fb.translation() - pos_ref).norm();
                const Eigen::Vector3d rpy =
                    T_fb.rotation().eulerAngles(2, 1, 0).reverse();
                printf("  [精度] 位置偏差=%.4f m  pitch=%.4f  反馈pitch=%.4f"
                       "  关节误差=%.4f  %s\n",
                       pos_err, pitch, rpy[1], settle_err,
                       settled ? "到位" : "超时");
                int n_print = std::min(nq, 6);
                printf("    IK  :");
                for (int i = 0; i < n_print; ++i) printf(" %7.4f", ik.q[i]);
                printf("\n    实际:");
                for (int i = 0; i < n_print; ++i) printf(" %7.4f", q_fb[i]);
                printf("\n    差值:");
                for (int i = 0; i < n_print; ++i) printf(" %7.4f", ik.q[i] - q_fb[i]);
                printf("\n");
            }
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
