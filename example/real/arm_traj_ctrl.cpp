/**
 * arm_traj_ctrl — 实机测地线轨迹交互控制
 *
 * 终端输入末端目标 "x y z [roll pitch yaw(rad)]"，确认后执行；Ctrl+C 回零退出。
 *
 * 使用: sudo ./arm_traj_ctrl [串口] [YAML] [URDF]
 *
 * 轨迹规划算法（se3Geodesic 测地线插值 + CLIK 跟踪）：
 *   - 测地线：SE(3) 空间最短路径插值，姿态平滑
 *   - CLIK：阻尼最小二乘逆运动学，带零空间关节限位规避
 */

#include "application/arm_controller.h"

#include <iostream>
#include <sstream>
#include <string>
#include <chrono>

using rebot::ArmController;
using namespace std::chrono;

static ArmController arm_ctrl;

static void print_pose(const char* prefix, const pinocchio::SE3& T)
{
    const auto&     p   = T.translation();
    Eigen::Vector3d rpy = T.rotation().eulerAngles(2, 1, 0).reverse();
    printf("%s  pos[%.3f %.3f %.3f]  rpy[%.3f %.3f %.3f]\n",
           prefix, p[0], p[1], p[2], rpy[0], rpy[1], rpy[2]);
}

int main(int argc, char* argv[])
{
    const std::string dev  = (argc > 1) ? argv[1] : "/dev/ttyACM0";
    const std::string yaml = (argc > 2) ? argv[2] : "";
    const std::string urdf = (argc > 3) ? argv[3] : "";

    if (!arm_ctrl.init(dev, yaml, urdf)) return 1;

    printf("输入: x y z [roll pitch yaw(rad)]  Ctrl+C 退出\n\n");

    std::string line;
    while (arm_ctrl.running()) {
        print_pose("当前", arm_ctrl.fk());

        if (!std::getline(std::cin, line) || !arm_ctrl.running()) break;
        if (line.find_first_not_of(" \t") == std::string::npos) continue;

        std::istringstream iss(line);
        double x, y, z, ro = 0, pi = 0, ya = 0;
        if (!(iss >> x >> y >> z)) {
            fprintf(stderr, "格式错误: x y z [roll pitch yaw]\n");
            continue;
        }
        iss >> ro >> pi >> ya;

        printf("目标  pos[%.3f %.3f %.3f]  rpy[%.3f %.3f %.3f]\n是否执行? [y/N]: ",
               x, y, z, ro, pi, ya);
        fflush(stdout);
        std::string yn;
        if (!std::getline(std::cin, yn)) break;
        if (yn.empty() || (yn[0] != 'y' && yn[0] != 'Y')) { printf("已取消\n\n"); continue; }

        // 测地线轨迹规划 + CLIK 跟踪
        auto start_time = high_resolution_clock::now();
        if (!arm_ctrl.move_to_geodesic(ArmController::pose(x, y, z, ro, pi, ya))) {
            fprintf(stderr, "[错误] IK 或轨迹规划失败\n\n");
            continue;
        }
        auto end_time = high_resolution_clock::now();
        double duration = duration_cast<milliseconds>(end_time - start_time).count() / 1000.0;

        print_pose("到达", arm_ctrl.fk());
        printf("[轨迹] 耗时: %.2f 秒\n\n", duration);
    }

    arm_ctrl.shutdown();
    return 0;
}
