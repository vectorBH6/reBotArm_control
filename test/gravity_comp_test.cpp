/**
 * @brief 重力补偿实机测试
 *
 * 两种运行模式（通过命令行参数切换）：
 *
 *   正常模式（无额外参数）：
 *     使能电机 → 零力拖拽：kp=0, kd=KD_DAMP, tau_ff=g(q)
 *     可用手自由拖拽机械臂，体验近似无重力的柔顺效果。
 *
 *   --dry-run 模式：
 *     连接硬件但电机保持失能（无动力，可自由搬动）。
 *     持续读取关节位置 → 计算 g(q) → 打印，不下发任何控制指令。
 *     用于上线前验证力矩数值是否合理。
 *
 * 参数：
 *   KD_DAMP — 速度阻尼 [N·m·s/rad]，正常模式有效（建议 0.3~1.0）
 *   CTRL_HZ — 控制/采样频率 [Hz]
 *
 * 使用：
 *   sudo ./gravity_comp_test [串口] [YAML] [URDF] [--dry-run]
 *   默认: /dev/ttyACM0  config/arm.yaml  <编译时 URDF_PATH>
 */

#include "dynamics/gravity_compensation.h"
#include "kinematics/robot_model.h"
#include "actuator/arm_actuator_group.h"

#include <Eigen/Core>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <string>
#include <unistd.h>

using namespace rebot;
using namespace actuator;

// ─── 参数 ─────────────────────────────────────────────────────────────────────
static constexpr float KD_DAMP = 0.5f;   // 速度阻尼 [N·m·s/rad]
static constexpr int   CTRL_HZ = 200;    // 控制频率 [Hz]

// ─── 信号处理 ─────────────────────────────────────────────────────────────────
static volatile bool g_running = true;
static void on_signal(int) { g_running = false; }

// ─── 获取可执行文件父目录（项目根目录）────────────────────────────────────────
#include <limits.h>
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

// ─── 主程序 ───────────────────────────────────────────────────────────────────
int main(int argc, char* argv[])
{
    const std::string root = project_root();

    // 解析参数
    std::string dev       = "/dev/ttyACM0";
    std::string yaml_path = root + "/config/arm.yaml";
    std::string urdf_path = URDF_PATH;
    bool        dry_run   = false;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--dry-run") == 0) { dry_run = true; continue; }
        if (i == 1) dev       = argv[i];
        if (i == 2) yaml_path = argv[i];
        if (i == 3) urdf_path = argv[i];
    }

    std::signal(SIGINT,  on_signal);
    std::signal(SIGTERM, on_signal);

    // ── 加载机器人模型 ────────────────────────────────────────────────────────
    printf("[model] 加载 URDF: %s\n", urdf_path.c_str());
    RobotModel robot(urdf_path);
    GravityCompensator comp(robot);
    const int nq = robot.nq();
    printf("[model] nq=%d  nv=%d\n", nq, robot.nv());

    // ── 连接硬件 ──────────────────────────────────────────────────────────────
    printf("[arm] 连接 %s ...\n", dev.c_str());
    ArmActuatorGroup arm = [&]() -> ArmActuatorGroup {
        try { return ArmActuatorGroup::from_yaml(dev, yaml_path); }
        catch (const std::exception& e) {
            fprintf(stderr, "[错误] 初始化失败: %s\n", e.what());
            std::exit(1);
        }
    }();

    const size_t N = arm.size();
    if (static_cast<int>(N) != nq) {
        fprintf(stderr, "[错误] 执行器数量(%zu)与 URDF 关节数(%d)不一致\n", N, nq);
        return 1;
    }

    // ── 扫描连接 ─────────────────────────────────────────────────────────────
    printf("[arm] 扫描关节...\n");
    auto connected = arm.scan_connectivity();
    auto names     = arm.joint_names();
    for (size_t i = 0; i < N; ++i)
        printf("  [%zu] %s: %s\n", i, names[i].c_str(), connected[i] ? "已连接" : "无响应");

    // ── 根据模式决定是否使能 ─────────────────────────────────────────────────
    if (!dry_run) {
        printf("[arm] 使能...\n");
        for (size_t i = 0; i < N; ++i)
            if (connected[i]) arm[i].enable();
        sleep(1);
        printf("[arm] 进入重力补偿模式 (kd=%.2f, %d Hz)  Ctrl+C 退出\n\n", KD_DAMP, CTRL_HZ);
    } else {
        printf("[dry-run] 电机保持失能，可自由搬动机械臂，实时打印 g(q)  Ctrl+C 退出\n\n");
    }

    // ── 控制/采样循环 ─────────────────────────────────────────────────────────
    Eigen::VectorXd q(nq);
    int print_cnt = 0;

    while (g_running) {
        // dry-run：主动拉取反馈（不发控制帧）；正常模式：set_mit 内已隐含反馈刷新
        if (dry_run)
            for (size_t i = 0; i < N; ++i)
                if (connected[i]) arm[i].refresh_status();

        auto pos = arm.get_all_positions();
        for (int i = 0; i < nq; ++i) q[i] = pos[i];

        Eigen::VectorXd tau_g = comp.compute(q);

        if (!dry_run)
            for (size_t i = 0; i < N; ++i)
                if (connected[i])
                    arm[i].set_mit(0.f, KD_DAMP, pos[i], 0.f, static_cast<float>(tau_g[i]));

        if (++print_cnt >= CTRL_HZ) {
            print_cnt = 0;
            printf("  τ_g [N·m]:");
            for (int i = 0; i < nq; ++i) printf(" %6.3f", tau_g[i]);
            printf("\n");
        }

        usleep(1000000 / CTRL_HZ);
    }

    // ── 退出：若已使能则失能 ──────────────────────────────────────────────────
    printf("\n[arm] 退出...\n");
    if (!dry_run)
        for (size_t i = 0; i < N; ++i)
            if (connected[i]) arm[i].disable();

    return 0;
}
