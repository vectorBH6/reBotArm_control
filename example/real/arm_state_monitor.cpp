/**
 * arm_state_monitor — 实机关节状态实时监视
 *
 * 以 50 Hz 刷新率轮询电机状态，只读，不使能运动；Ctrl+C 退出。
 * 使用: ./arm_state_monitor [串口] [YAML] [URDF]
 */

#include "application/arm_controller.h"

#include <csignal>
#include <cstdio>
#include <string>
#include <unistd.h>

using rebot::ArmController;

static ArmController arm_ctrl;
static volatile bool keep_running = true;

int main(int argc, char* argv[])
{
    const std::string dev  = (argc > 1) ? argv[1] : "/dev/ttyACM0";
    const std::string yaml = (argc > 2) ? argv[2] : "";
    const std::string urdf = (argc > 3) ? argv[3] : "";

    std::signal(SIGINT,  [](int) { keep_running = false; });
    std::signal(SIGTERM, [](int) { keep_running = false; });

    if (!arm_ctrl.init_monitor(dev, yaml, urdf)) return 1;

    const int nq = arm_ctrl.robot.nq();
    const auto names = arm_ctrl.joint_names();

    printf("[arm] 状态监视启动，Ctrl+C 退出\n\n");

    while (keep_running) {
        arm_ctrl.refresh_hw();

        const auto& q   = arm_ctrl.q;
        const auto  T   = arm_ctrl.fk();
        const auto& p   = T.translation();
        Eigen::Vector3d rpy = T.rotation().eulerAngles(2, 1, 0).reverse();

        printf("关节 [rad]:");
        for (int i = 0; i < nq; ++i)
            printf("  %s=%.4f", names[i].c_str(), q[i]);
        printf("\n末端  pos[%.4f %.4f %.4f]  rpy[%.4f %.4f %.4f]\n\n",
               p[0], p[1], p[2], rpy[0], rpy[1], rpy[2]);
        fflush(stdout);

        usleep(20'000); // 50 Hz
    }

    return 0;
}
