/**
 * arm_traj_triangle — 末端三角形轨迹循环
 *
 * 交替执行两个三角形路径，每侧各 2 回合；Ctrl+C 回零退出。
 * 使用: sudo ./arm_traj_triangle [串口] [YAML] [URDF]
 */

#include "application/arm_controller.h"

#include <cstdio>
#include <string>
#include <vector>

using rebot::ArmController;

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

// 将三角形首尾相连组成闭合路径，执行 rounds 圈
static void run_triangle(const std::vector<pinocchio::SE3>& tri, int rounds)
{
    std::vector<pinocchio::SE3> path = tri;
    path.push_back(tri[0]);
    for (int r = 0; r < rounds && arm_ctrl.running(); ++r)
        arm_ctrl.move_through_geodesic(path);
}

int main(int argc, char* argv[])
{
    const std::string dev  = (argc > 1) ? argv[1] : "/dev/ttyACM0";
    const std::string yaml = (argc > 2) ? argv[2] : "";
    const std::string urdf = (argc > 3) ? argv[3] : "";

    if (!arm_ctrl.init(dev, yaml, urdf)) return 1;

    printf("画三角形  Ctrl+C 退出\n\n");

    if (!arm_ctrl.move_to_geodesic(tri1[0])) {
        fprintf(stderr, "[错误] 移动到起点失败\n");
        arm_ctrl.shutdown();
        return 1;
    }

    constexpr int ROUNDS = 2;

    while (arm_ctrl.running()) {
        printf("[tri] 三角形 1  x%d\n", ROUNDS);
        run_triangle(tri1, ROUNDS);

        if (!arm_ctrl.running()) break;
        arm_ctrl.move_to_geodesic(tri2[0]);

        printf("[tri] 三角形 2  x%d\n", ROUNDS);
        run_triangle(tri2, ROUNDS);

        if (!arm_ctrl.running()) break;
        arm_ctrl.move_to_geodesic(tri1[0]);
    }

    arm_ctrl.shutdown();
    return 0;
}
