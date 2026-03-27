/**
 * arm_traj_triangle — 末端三角形轨迹循环
 *
 * 交替执行两个三角形路径，每侧各 2 回合；Ctrl+C 回零退出。
 *
 * 使用: sudo ./arm_traj_triangle [串口] [YAML] [URDF]
 */

#include "application/arm_controller.h"

#include <cstdio>
#include <string>
#include <vector>
#include <chrono>

using rebot::ArmController;
using namespace std::chrono;

static ArmController arm_ctrl;

// 三角形顶点（末端位姿）
static const std::vector<pinocchio::SE3> tri1 = {
    ArmController::pose(0.4, -0.2, 0.4, 0.0, 0.0,  0.0),
    ArmController::pose(0.4,  0.2, 0.4, 0.0, 0.0,  0.0),
    ArmController::pose(0.4,  0.0, 0.2, 0.0, 0.0,  0.0),
};
static const std::vector<pinocchio::SE3> tri2 = {
    ArmController::pose(0.4,  0.0, 0.1, 0.0, 1.57, 0.0),
    ArmController::pose(0.2,  0.2, 0.1, 0.0, 1.57, 0.0),
    ArmController::pose(0.2, -0.2, 0.1, 0.0, 1.57, 0.0),
};

// 打印轨迹统计信息
static void print_traj_stats(const char* name, int rounds,
                             double total_time, bool success)
{
    printf("[轨迹统计] %s\n", name);
    printf("  圈数: %d\n", rounds);
    printf("  总耗时: %.2f 秒\n", total_time);
    printf("  状态: %s\n\n", success ? "完成" : "中断");
}

// 批量预规划模式：三角形首尾相连，执行 rounds 圈
static bool run_triangle_batch(const std::vector<pinocchio::SE3>& tri, int rounds)
{
    std::vector<pinocchio::SE3> path = tri;
    path.push_back(tri[0]);

    for (int r = 0; r < rounds && arm_ctrl.running(); ++r) {
        if (!arm_ctrl.move_through_geodesic(path)) {
            fprintf(stderr, "[错误] 批量轨迹执行失败于第 %d 圈\n", r + 1);
            return false;
        }
    }
    return true;
}

int main(int argc, char* argv[])
{
    const std::string dev  = (argc > 1) ? argv[1] : "/dev/ttyACM0";
    const std::string yaml = (argc > 2) ? argv[2] : "";
    const std::string urdf = (argc > 3) ? argv[3] : "";

    if (!arm_ctrl.init(dev, yaml, urdf)) return 1;

    printf("\n========= 三角形轨迹 =========\n");
    printf("Ctrl+C 退出\n\n");

    // 移动到起点
    auto start_time = high_resolution_clock::now();
    if (!arm_ctrl.move_to_geodesic(tri1[0])) {
        fprintf(stderr, "[错误] 移动到起点失败\n");
        arm_ctrl.shutdown();
        return 1;
    }

    constexpr int ROUNDS = 2;
    bool all_success = true;

    while (arm_ctrl.running()) {
        // 三角形 1
        printf("[tri] 三角形 1  x%d\n", ROUNDS);
        auto t1_start = high_resolution_clock::now();
        if (!run_triangle_batch(tri1, ROUNDS)) {
            all_success = false;
            break;
        }
        auto t1_end = high_resolution_clock::now();
        double t1_duration = duration_cast<milliseconds>(t1_end - t1_start).count() / 1000.0;
        print_traj_stats("三角形1", ROUNDS, t1_duration, true);

        if (!arm_ctrl.running()) break;

        // 移动到三角形2起点
        arm_ctrl.move_to_geodesic(tri2[0]);

        // 三角形 2
        printf("[tri] 三角形 2  x%d\n", ROUNDS);
        auto t2_start = high_resolution_clock::now();
        if (!run_triangle_batch(tri2, ROUNDS)) {
            all_success = false;
            break;
        }
        auto t2_end = high_resolution_clock::now();
        double t2_duration = duration_cast<milliseconds>(t2_end - t2_start).count() / 1000.0;
        print_traj_stats("三角形2", ROUNDS, t2_duration, true);

        if (!arm_ctrl.running()) break;

        // 返回起点
        arm_ctrl.move_to_geodesic(tri1[0]);
    }

    auto total_end = high_resolution_clock::now();
    double total_duration = duration_cast<milliseconds>(total_end - start_time).count() / 1000.0;
    print_traj_stats("总执行", 1, total_duration, all_success);

    arm_ctrl.shutdown();
    return 0;
}
