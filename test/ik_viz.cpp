#include <iostream>
#include <sstream>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <random>
#include <unistd.h>
#include <sys/wait.h>
#include "kinematics/robot_model.h"
#include "kinematics/forward_kinematics.h"
#include "kinematics/inverse_kinematics.h"

static std::string toJson(const Eigen::VectorXd& v) {
    std::ostringstream ss;
    ss << "[";
    for (int i = 0; i < v.size(); ++i) { if (i) ss << ","; ss << v[i]; }
    ss << "]";
    return ss.str();
}

static std::string toJson(const Eigen::Matrix3d& R) {
    std::ostringstream ss;
    ss << "[";
    for (int i = 0; i < 3; ++i) {
        if (i) ss << ",";
        ss << "[" << R(i,0) << "," << R(i,1) << "," << R(i,2) << "]";
    }
    ss << "]";
    return ss.str();
}

int main() {
    rebot::RobotModel robot(URDF_PATH);

    // 创建管道：pipe_fd[0]=读端(子进程stdin), pipe_fd[1]=写端(父进程写)
    int pipe_fd[2];
    if (pipe(pipe_fd) != 0) {
        std::cerr << "pipe() 失败\n";
        return 1;
    }

    pid_t pid = fork();
    if (pid < 0) {
        std::cerr << "fork() 失败\n";
        return 1;
    }

    if (pid == 0) {
        // 子进程：将管道读端接到 stdin，stdout/stderr 保持不变（直接输出到终端）
        close(pipe_fd[1]);
        dup2(pipe_fd[0], STDIN_FILENO);
        close(pipe_fd[0]);

        std::string viewer = std::string(PROJECT_SOURCE_DIR) + "/test/ik_viz_viewer.py";
        execl(PYTHON_BIN, PYTHON_BIN, viewer.c_str(), nullptr);
        std::cerr << "execl 失败\n";
        _exit(1);
    }

    // 父进程：关闭读端，通过写端与子进程通信
    close(pipe_fd[0]);
    FILE* py = fdopen(pipe_fd[1], "w");

    // 先发送 URDF 路径（第一行）
    fprintf(py, "%s\n", URDF_PATH);
    fflush(py);

    // 等待 MeshCat 初始化（子进程会打印地址到终端）
    sleep(2);

    rebot::IKParams params;
    params.tolerance = 1e-4;
    params.max_iter  = 2000;

    Eigen::VectorXd q_seed = robot.neutralConfig();

    std::cout << "\n输入格式: x y z [roll pitch yaw]  (单位: 米 / 弧度)\n";
    std::cout << "输入 q 或 Ctrl+D 退出\n\n";

    std::string line;
    while (true) {
        std::cout << "目标位姿> ";
        std::cout.flush();
        if (!std::getline(std::cin, line)) break;

        size_t s = line.find_first_not_of(" \t");
        if (s == std::string::npos) continue;
        line = line.substr(s);
        if (line == "q" || line == "quit" || line == "exit") break;

        std::istringstream iss(line);
        double x = 0, y = 0, z = 0, ro = 0, pi_v = 0, ya = 0;
        if (!(iss >> x >> y >> z)) {
            std::cerr << "  格式错误，请输入: x y z [roll pitch yaw]\n";
            continue;
        }
        iss >> ro >> pi_v >> ya;

        Eigen::Matrix3d R =
           (Eigen::AngleAxisd(ya,   Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pi_v, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(ro,   Eigen::Vector3d::UnitX())).toRotationMatrix();
        pinocchio::SE3 target(R, Eigen::Vector3d(x, y, z));

        // 先用上次的解作为初始猜测
        rebot::IKResult res = rebot::solveIK(robot, target, q_seed, params);

        // 若未收敛，用随机初始构型多次重试，取误差最小的结果
        if (!res.success) {
            static std::mt19937 rng(42);
            const auto& lo = robot.model().lowerPositionLimit;
            const auto& hi = robot.model().upperPositionLimit;
            const int nq = robot.nq();
            for (int retry = 0; retry < 8; ++retry) {
                Eigen::VectorXd q_rand(nq);
                for (int j = 0; j < nq; ++j) {
                    double l = std::isfinite(lo[j]) ? lo[j] : -M_PI;
                    double h = std::isfinite(hi[j]) ? hi[j] :  M_PI;
                    q_rand[j] = std::uniform_real_distribution<double>(l, h)(rng);
                }
                rebot::IKResult r = rebot::solveIK(robot, target, q_rand, params);
                if (r.error < res.error) res = r;
                if (res.success) break;
            }
        }

        std::cout << "  IK: success=" << res.success
                  << "  iters="       << res.iterations
                  << "  err="         << res.error        << "\n"
                  << "  q = "         << res.q.transpose()<< "\n";
        if (!res.success)
            std::cerr << "  警告：IK 未完全收敛，显示最佳近似结果\n";

        q_seed = res.q;

        const auto& p = target.translation();
        std::ostringstream msg;
        msg << "{\"target\":{\"xyz\":[" << p[0] << "," << p[1] << "," << p[2] << "],"
            << "\"R\":"   << toJson(target.rotation()) << "},"
            << "\"q\":"   << toJson(res.q) << "}\n";

        fputs(msg.str().c_str(), py);
        fflush(py);
    }

    fclose(py);
    waitpid(pid, nullptr, 0);
    return 0;
}
