#pragma once

/**
 * 仿真与 viewer.py 之间的唯一数据通道：管道写入「一行一条 JSON」。
 * 仅依赖底层 kinematics，不依赖 ArmController。
 */

#include "kinematics/robot_model.h"
#include "kinematics/forward_kinematics.h"
#include "kinematics/inverse_kinematics.h"
#include "kinematics/trajectory_planner_geodesic.h"

#include <chrono>
#include <cstdio>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <sys/wait.h>
#include <unistd.h>

namespace rebot {
namespace sim {

struct Pipe {
    FILE*   out = nullptr;
    pid_t   pid = -1;
};

inline bool spawn(Pipe* p, const char* urdf_path)
{
    int fd[2];
    if (::pipe(fd) != 0) return false;
    pid_t child = fork();
    if (child < 0) return false;
    if (child == 0) {
        close(fd[1]);
        dup2(fd[0], STDIN_FILENO);
        close(fd[0]);
        std::string py = std::string(PROJECT_SOURCE_DIR) + "/example/sim/viewer.py";
        execl(PYTHON_BIN, PYTHON_BIN, py.c_str(), nullptr);
        _exit(127);
    }
    close(fd[0]);
    p->out = fdopen(fd[1], "w");
    p->pid = child;
    if (!p->out) return false;
    fprintf(p->out, "%s\n", urdf_path);
    fflush(p->out);
    std::this_thread::sleep_for(std::chrono::seconds(3));
    return true;
}

inline void stop(Pipe* p)
{
    if (!p || !p->out) return;
    fputs("{\"cmd\":\"exit\"}\n", p->out);
    fflush(p->out);
    fclose(p->out);
    p->out = nullptr;
    if (p->pid > 0) {
        waitpid(p->pid, nullptr, 0);
        p->pid = -1;
    }
}

inline void send_line(Pipe* p, const std::string& line)
{
    if (!p || !p->out) return;
    fputs(line.c_str(), p->out);
    if (line.empty() || line.back() != '\n') fputc('\n', p->out);
    fflush(p->out);
}

inline std::string pack_ik_json(const pinocchio::SE3& T, const Eigen::VectorXd& q)
{
    std::ostringstream ss;
    ss << std::setprecision(9);
    const auto& p = T.translation();
    const Eigen::Matrix3d& R = T.rotation();
    ss << "{\"target\":{\"xyz\":[" << p[0] << "," << p[1] << "," << p[2] << "],\"R\":[";
    for (int i = 0; i < 3; ++i) {
        if (i) ss << ',';
        ss << "[" << R(i, 0) << "," << R(i, 1) << "," << R(i, 2) << "]";
    }
    ss << "]},\"q\":[";
    for (int i = 0; i < q.size(); ++i) {
        if (i) ss << ',';
        ss << q[i];
    }
    ss << "]}\n";
    return ss.str();
}

inline std::string pack_traj_json(RobotModel&                             robot,
                                  const std::string&                      name,
                                  double                                  dt,
                                  const std::vector<JointTrajectoryPoint>& traj)
{
    std::ostringstream ss;
    ss << std::setprecision(6) << std::fixed;
    ss << "{\"name\":\"" << name << "\",\"dt\":" << dt << ",\"q_list\":[";
    for (size_t i = 0; i < traj.size(); ++i) {
        if (i) ss << ',';
        ss << '[';
        for (int j = 0; j < traj[i].q.size(); ++j) {
            if (j) ss << ',';
            ss << traj[i].q[j];
        }
        ss << ']';
    }
    ss << "],\"path\":[";
    for (size_t i = 0; i < traj.size(); ++i) {
        if (i) ss << ',';
        const auto& p = computeFK(robot, traj[i].q).translation();
        ss << '[' << p[0] << ',' << p[1] << ',' << p[2] << ']';
    }
    ss << "]}\n";
    return ss.str();
}

}  // namespace sim
}  // namespace rebot
