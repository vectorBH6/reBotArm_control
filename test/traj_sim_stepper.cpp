/**
 * traj_sim_stepper — 末端步进控制仿真
 * 输入: x y z [ro pi ya]  输出: EndMotionStepper 每步在线 CLIK
 */

#include <iostream>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <chrono>
#include <unistd.h>
#include <sys/wait.h>

#include "kinematics/robot_model.h"
#include "kinematics/forward_kinematics.h"
#include "kinematics/end_motion_stepper.h"

using Clock = std::chrono::steady_clock;
using Ms    = std::chrono::duration<double, std::milli>;

#ifdef PYTHON_BIN
static void sendTraj(FILE* viz, const std::string& name, double dt,
                     const std::vector<Eigen::VectorXd>& qs,
                     const std::vector<Eigen::Vector3d>& path) {
    std::ostringstream ss;
    ss << std::setprecision(6) << "{\"name\":\"" << name << "\",\"dt\":" << dt << ",\"q_list\":[";
    for (size_t i = 0; i < qs.size(); ++i) {
        if (i) ss << ',';
        ss << '[';
        for (int j = 0; j < qs[i].size(); ++j) { if (j) ss << ','; ss << qs[i][j]; }
        ss << ']';
    }
    ss << "],\"path\":[";
    for (size_t i = 0; i < path.size(); ++i) {
        if (i) ss << ',';
        ss << '[' << path[i][0] << ',' << path[i][1] << ',' << path[i][2] << ']';
    }
    ss << "]}\n";
    fputs(ss.str().c_str(), viz);
    fflush(viz);
}
#endif

int main(int argc, char** argv) {
    const std::string urdf = (argc > 1) ? argv[1] : URDF_PATH;
    rebot::RobotModel robot(urdf);

    FILE* viz = nullptr;
    pid_t pid = -1;
#ifdef PYTHON_BIN
    int pipe_fd[2];
    if (pipe(pipe_fd) == 0 && (pid = fork()) == 0) {
        close(pipe_fd[1]);
        dup2(pipe_fd[0], STDIN_FILENO);
        close(pipe_fd[0]);
        execl(PYTHON_BIN, PYTHON_BIN,
              (std::string(PROJECT_SOURCE_DIR) + "/test/traj_sim_viewer.py").c_str(), nullptr);
        _exit(1);
    }
    if (pid > 0) {
        close(pipe_fd[0]);
        viz = fdopen(pipe_fd[1], "w");
        fprintf(viz, "%s\n", urdf.c_str());
        fflush(viz);
        sleep(3);
    }
#endif

    rebot::StepperConfig cfg;
    rebot::EndMotionStepper stepper(robot, cfg);
    Eigen::VectorXd q = robot.neutralConfig();

    std::cout << "URDF: " << urdf << "\ndt: " << cfg.dt << " s  速度限: "
              << cfg.cart_speed << " m/s  " << cfg.rot_speed << " rad/s\n"
              << "输入: x y z [ro pi ya]  q 退出\n\n";

    std::string line;
    while (true) {
        const auto T_cur = rebot::computeFK(robot, q);
        const auto& p = T_cur.translation();
        Eigen::Vector3d rpy = T_cur.rotation().eulerAngles(2, 1, 0).reverse();

        std::cout << std::fixed << std::setprecision(3)
                  << "末端 pos[" << p[0] << ' ' << p[1] << ' ' << p[2] << "]"
                  << " rpy[" << rpy[0] << ' ' << rpy[1] << ' ' << rpy[2] << "]> ";
        std::cout.flush();

        if (!std::getline(std::cin, line)) break;
        { size_t s = line.find_first_not_of(" \t"); if (s == std::string::npos) continue; line = line.substr(s); }
        if (line == "q" || line == "quit") break;

        std::istringstream iss(line);
        double x, y, z, ro = 0, pi_v = 0, ya = 0;
        if (!(iss >> x >> y >> z)) { std::cerr << "格式: x y z [ro pi ya]\n"; continue; }
        iss >> ro >> pi_v >> ya;

        Eigen::Matrix3d R =
            (Eigen::AngleAxisd(ya, Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(pi_v, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(ro, Eigen::Vector3d::UnitX())).toRotationMatrix();
        const pinocchio::SE3 T_tgt(R, Eigen::Vector3d(x, y, z));

        double dist = (T_tgt.translation() - p).norm();
        Eigen::Quaterniond qa(T_cur.rotation()), qb(R);
        if (qa.dot(qb) < 0) qb.coeffs() = -qb.coeffs();
        double angle_err = 2.0 * std::acos(std::min(1.0, std::abs((qb * qa.inverse()).w())));
        std::cout << "目标 pos[" << x << ' ' << y << ' ' << z << "] rpy[" << ro << ' ' << pi_v << ' ' << ya << "]\n"
                  << "预计步数≥" << static_cast<int>(std::max(dist / (cfg.cart_speed * cfg.dt),
                          angle_err / (cfg.rot_speed * cfg.dt))) << " / " << cfg.max_steps << "\n";

        stepper.setTarget(T_tgt);

        std::vector<Eigen::VectorXd> qs;
        std::vector<Eigen::Vector3d> path;
        qs.reserve(cfg.max_steps);
        path.reserve(cfg.max_steps);

        auto t0 = Clock::now();
        while (!stepper.done() && static_cast<int>(qs.size()) < cfg.max_steps) {
            q = stepper.step(q);
            qs.push_back(q);
            path.push_back(rebot::computeFK(robot, q).translation());
        }
        double solve_ms = Ms(Clock::now() - t0).count();

        double sim_s = qs.size() * cfg.dt;
        std::cout << (stepper.done() ? "✓ 到达" : "✗ 超时")
                  << " steps=" << qs.size() << " 仿真=" << std::fixed << std::setprecision(2) << sim_s << " s"
                  << " CPU=" << std::setprecision(1) << solve_ms << " ms"
                  << " 误差=" << std::scientific << stepper.error() << "\n";

#ifdef PYTHON_BIN
        if (viz && !qs.empty()) {
            sendTraj(viz, "[" + std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z) + "]",
                     cfg.dt, qs, path);
        }
#endif
    }

#ifdef PYTHON_BIN
    if (viz) { fprintf(viz, "{\"cmd\":\"exit\"}\n"); fflush(viz); fclose(viz); }
    if (pid > 0) waitpid(pid, nullptr, 0);
#endif
    return 0;
}
