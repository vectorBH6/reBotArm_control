/**
 * @brief 重力补偿单关节安全测试
 *
 * 目标：
 * - 仅使能指定关节，其余关节保持失能（不发控制帧），降低实机测试风险
 * - 循环中计算并打印期望重力补偿力矩 τ_g
 * - Ctrl+C 触发“缓慢回零”：对已使能关节做插值位置回零（可选叠加重力补偿）
 *
 * 使用：
 *   sudo ./gravity_comp_single_joint_test [串口] [YAML] [URDF]
 *   可选参数：
 *     --enable  idx[,idx...]    使能关节索引列表（默认见代码 DEFAULT_ENABLE）
 *     --dry-run                不使能、不下发控制，只读状态并打印 τ_g
 *
 * 说明：
 * - “回到0点”在这里指关节角 q -> 0 rad（不是写入电机零点）
 * - 打印频率、回零时间、回零增益等都在下方常量里预留可改
 */

#include "dynamics/gravity_compensation.h"
#include "kinematics/robot_model.h"
#include "actuator/arm_actuator_group.h"

#include <Eigen/Core>

#include <algorithm>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits.h>
#include <string>
#include <unistd.h>
#include <vector>

using namespace rebot;
using namespace actuator;

// ─── 可调参数（按需修改）────────────────────────────────────────────────────────
static constexpr int   CTRL_HZ        = 500;  // 控制/采样频率 [Hz]
static constexpr int   PRINT_HZ       = 10;   // τ_g 打印频率 [Hz]
static constexpr int   STATUS_HZ      = 50;   // 反馈刷新频率 [Hz]（过高会受串口超时影响）
static constexpr float NORMAL_KP      = 2.0f; // 正常模式下的小 kp（仅对使能关节）[N·m/rad]
static constexpr float KD_DAMP        = 1.0f; // 单关节重力补偿时的阻尼 [N·m·s/rad]

static constexpr float RETURN_SEC     = 3.0f; // Ctrl+C 后回零时间 [s]
static constexpr float RETURN_KP      = 5.0f; // 回零位置增益（仅对使能关节）[N·m/rad]
static constexpr float RETURN_KD      = 1.5f; // 回零速度阻尼（仅对使能关节）[N·m·s/rad]
static constexpr bool  RETURN_USE_G   = false; // 回零时是否叠加重力补偿 tau_ff

// 默认使能关节索引（可通过 --enable 覆盖）
static const std::vector<int> DEFAULT_ENABLE = {1,2,3};

// ─── 信号处理 ─────────────────────────────────────────────────────────────────
static volatile sig_atomic_t g_stop_requested = 0;
static void on_signal(int) { g_stop_requested = 1; }

// ─── 获取项目根目录（用于默认 config 路径）──────────────────────────────────────
static std::string project_root()
{
    char buf[PATH_MAX]{};
    ssize_t n = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (n <= 0) return ".";
    std::string p(buf, n);
    auto s1 = p.rfind('/');
    p = (s1 == std::string::npos) ? "." : p.substr(0, s1);
    auto s2 = p.rfind('/');
    return (s2 == std::string::npos) ? "." : p.substr(0, s2);
}

static std::vector<int> parse_enable_list(const char* s)
{
    std::vector<int> out;
    if (!s || !*s) return out;

    const char* p = s;
    while (*p) {
        while (*p == ' ' || *p == '\t' || *p == ',') ++p;
        if (!*p) break;
        char* end = nullptr;
        long v = std::strtol(p, &end, 10);
        if (end == p) break;
        out.push_back(static_cast<int>(v));
        p = end;
        while (*p && *p != ',') ++p;
    }

    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

static bool contains(const std::vector<int>& v, int x)
{
    return std::find(v.begin(), v.end(), x) != v.end();
}

int main(int argc, char* argv[])
{
    const std::string root = project_root();

    std::string dev       = "/dev/ttyACM0";
    std::string yaml_path = root + "/config/arm.yaml";
    std::string urdf_path = URDF_PATH;
    bool dry_run = false;
    std::vector<int> enable = DEFAULT_ENABLE;

    // 参数：位置参数沿用 gravity_comp_test.cpp 的顺序；选项用 --xxx
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--dry-run") == 0) { dry_run = true; continue; }
        if (std::strcmp(argv[i], "--enable") == 0 && i + 1 < argc) { enable = parse_enable_list(argv[++i]); continue; }
        if (i == 1) dev       = argv[i];
        if (i == 2) yaml_path = argv[i];
        if (i == 3) urdf_path = argv[i];
    }
    if (enable.empty()) enable = DEFAULT_ENABLE;

    std::signal(SIGINT,  on_signal);
    std::signal(SIGTERM, on_signal);

    // ── 模型 ───────────────────────────────────────────────────────────────────
    std::printf("[model] 加载 URDF: %s\n", urdf_path.c_str());
    RobotModel robot(urdf_path);
    GravityCompensator comp(robot);
    const int nq = robot.nq();

    // ── 硬件 ───────────────────────────────────────────────────────────────────
    std::printf("[arm] 连接 %s ...\n", dev.c_str());
    ArmActuatorGroup arm = [&]() -> ArmActuatorGroup {
        try { return ArmActuatorGroup::from_yaml(dev, yaml_path); }
        catch (const std::exception& e) {
            std::fprintf(stderr, "[错误] 初始化失败: %s\n", e.what());
            std::exit(1);
        }
    }();

    const int N = static_cast<int>(arm.size());
    if (N < nq) {
        std::fprintf(stderr, "[错误] 执行器数量(%d)小于 URDF 关节数(%d)\n", N, nq);
        return 1;
    }
    const int use_n = std::min(nq, N);

    std::printf("[arm] 扫描关节...\n");
    auto connected = arm.scan_connectivity();
    auto names = arm.joint_names();
    for (int i = 0; i < N; ++i)
        std::printf("  [%d] %s: %s\n", i, names[i].c_str(), connected[i] ? "已连接" : "无响应");

    // ── 安全策略：先全部失能，再只使能指定关节 ────────────────────────────────
    std::printf("[arm] 安全：先全失能...\n");
    arm.disable_all();
    usleep(200 * 1000);

    std::printf("[arm] 使能关节索引:");
    for (int idx : enable) std::printf(" %d", idx);
    std::printf("\n");

    if (!dry_run) {
        for (int idx : enable) {
            if (idx < 0 || idx >= N) {
                std::fprintf(stderr, "[警告] enable idx=%d 越界，忽略\n", idx);
                continue;
            }
            if (!connected[idx]) {
                std::fprintf(stderr, "[警告] enable idx=%d (%s) 无响应，忽略\n", idx, names[idx].c_str());
                continue;
            }
            arm[static_cast<std::size_t>(idx)].enable();
        }
        sleep(1);
        std::printf("[arm] 单关节重力补偿测试启动 (%d Hz), 打印 %d Hz。Ctrl+C 触发缓慢回零\n\n",
                    CTRL_HZ, PRINT_HZ);
    } else {
        std::printf("[dry-run] 不使能不下发控制；实时打印 τ_g。Ctrl+C 退出\n\n");
    }

    Eigen::VectorXd q(nq);
    Eigen::VectorXd tau_g(nq);

    const int print_every = std::max(1, CTRL_HZ / std::max(1, PRINT_HZ));
    const int status_every = std::max(1, CTRL_HZ / std::max(1, STATUS_HZ));
    int loop_cnt = 0;

    // 回零状态
    bool returning = false;
    int  return_steps = std::max(1, static_cast<int>(RETURN_SEC * CTRL_HZ));
    int  return_k = 0;
    std::vector<float> q0_start(N, 0.f);

    while (true) {
        // 反馈刷新：
        // - dry-run / 回零阶段：每圈刷新，确保读数实时
        // - 正常阶段：按 STATUS_HZ 刷新（避免在高 CTRL_HZ 下被串口超时拖慢）
        const bool do_status = (dry_run || returning) || ((loop_cnt % status_every) == 0);
        if (do_status) {
            for (int i = 0; i < use_n; ++i)
                if (connected[i]) arm[static_cast<std::size_t>(i)].refresh_status();
        }

        auto pos = arm.get_all_positions();
        for (int i = 0; i < use_n; ++i) q[i] = pos[i];

        tau_g = comp.compute(q);

        if (!dry_run) {
            // Ctrl+C：进入回零阶段（只触发一次）
            if (g_stop_requested && !returning) {
                returning = true;
                return_k = 0;
                for (int i = 0; i < N; ++i) q0_start[i] = pos[i];
                std::printf("\n[arm] 收到退出信号，开始缓慢回零 (%.2fs)\n", RETURN_SEC);
            }

            if (!returning) {
                // 正常：仅对使能关节发送 MIT（零刚度+阻尼+重力补偿）
                for (int i = 0; i < use_n; ++i) {
                    if (!connected[i]) continue;
                    if (!contains(enable, i)) continue;
                    arm[static_cast<std::size_t>(i)].set_mit(
                        NORMAL_KP, KD_DAMP,
                        pos[i], 0.f,
                        static_cast<float>(tau_g[i]));
                }
            } else {
                // 回零：对使能关节做插值位置回零，可选叠加重力补偿
                const float alpha = std::min(1.0f, static_cast<float>(return_k) / static_cast<float>(return_steps));
                for (int i = 0; i < use_n; ++i) {
                    if (!connected[i]) continue;
                    if (!contains(enable, i)) continue;
                    const float q_des = (1.0f - alpha) * q0_start[i];  // -> 0
                    const float tau_ff = RETURN_USE_G ? static_cast<float>(tau_g[i]) : 0.f;
                    arm[static_cast<std::size_t>(i)].set_mit(
                        RETURN_KP, RETURN_KD,
                        q_des, 0.f,
                        tau_ff);
                }
                if (++return_k >= return_steps) break; // 回零完成后退出
            }
        } else {
            if (g_stop_requested) break;
        }

        if ((++loop_cnt % print_every) == 0) {
            std::printf("τ_g [N·m]:");
            for (int i = 0; i < nq; ++i) std::printf(" %6.3f", tau_g[i]);
            std::printf("\n");
            std::fflush(stdout);
        }

        usleep(1000000 / CTRL_HZ);
    }

    std::printf("\n[arm] 退出：失能已使能关节...\n");
    if (!dry_run) {
        for (int idx : enable) {
            if (idx < 0 || idx >= N) continue;
            if (!connected[idx]) continue;
            arm[static_cast<std::size_t>(idx)].disable();
        }
    }
    return 0;
}

