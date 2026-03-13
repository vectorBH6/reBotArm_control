#include "actuator/arm_actuator_group.h"
#include "kinematics/robot_model.h"
#include "kinematics/forward_kinematics.h"

#include <Eigen/Core>

#include <csignal>
#include <cstdio>
#include <string>
#include <unistd.h>
#include <limits.h>  // PATH_MAX

using namespace actuator;
using namespace rebot;

namespace {

static volatile bool g_running = true;
static void on_signal(int) { g_running = false; }

// 获取可执行文件所在目录的父目录（项目根），用于默认 config 路径
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
    for (std::size_t i = 0; i < v.size(); ++i) q[static_cast<int>(i)] = v[i];
    return q;
}

} // namespace

int main(int argc, char* argv[])
{
    const std::string root = exe_parent_dir();
    std::string dev       = "/dev/ttyACM0";
    std::string yaml_path = root + "/config/arm.yaml";
    std::string urdf_path = URDF_PATH;
    if (argc > 1) dev       = argv[1];
    if (argc > 2) yaml_path = argv[2];
    if (argc > 3) urdf_path = argv[3];

    std::signal(SIGINT,  on_signal);
    std::signal(SIGTERM, on_signal);

    // 加载模型
    std::printf("[model] %s\n", urdf_path.c_str());
    RobotModel robot(urdf_path);
    const int nq = robot.nq();

    // 连接执行器组
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
        std::fprintf(stderr, "[错误] 执行器数(%d) 少于 URDF 关节数(%d)\n", N, nq);
        return 1;
    }

    auto names = arm.joint_names();

    std::printf("[arm] 实时状态监视启动，Ctrl+C 退出。\n");

    const double dt = 0.02;  // 50 Hz
    while (g_running) {
        // 主动查询各关节状态，触发电机回传最新反馈
        for (int i = 0; i < N; ++i)
            arm[i].refresh_status();

        // 读取电机角度（上一次 refresh_status 收到的反馈）
        std::vector<float> pos = arm.get_all_positions();
        Eigen::VectorXd q = toEigen(pos);
        if (q.size() > nq) q.conservativeResize(nq);

        // 计算末端位姿
        const auto T = computeFK(robot, q);
        const auto& p = T.translation();
        const Eigen::Vector3d rpy = T.rotation().eulerAngles(2, 1, 0).reverse();

        // 打印
        std::printf("关节 [rad]:");
        for (int i = 0; i < nq; ++i)
            std::printf(" %s=%.4f", names[i].c_str(), q[i]);
        std::printf("\n末端 pos[%.4f %.4f %.4f] rpy[%.4f %.4f %.4f]\n\n",
                    p[0], p[1], p[2], rpy[0], rpy[1], rpy[2]);
        fflush(stdout);

        usleep(static_cast<useconds_t>(dt * 1e6));
    }

    return 0;
}

