/**
 * traj_sim_geodesic — 测地线轨迹规划仿真
 * 输入: x y z [ro pi ya]  输出: planJointSpaceTrajectory（测地线+CLIK）
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
#include "kinematics/inverse_kinematics.h"
#include "kinematics/trajectory_planner_geodesic.h"

using Clock = std::chrono::steady_clock;
using Ms    = std::chrono::duration<double, std::milli>;

#ifdef PYTHON_BIN
static void sendTraj(FILE* viz, rebot::RobotModel& robot,
                     const std::string& name, double dt,
                     const std::vector<rebot::JointTrajectoryPoint>& traj) {
    std::ostringstream ss;
    ss << std::setprecision(6)
       << "{\"name\":\"" << name << "\",\"dt\":" << dt << ",\"q_list\":[";
    for (size_t i = 0; i < traj.size(); ++i) {
        if (i) ss << ',';
        ss << '[';
        const auto& q = traj[i].q;
        for (int j = 0; j < q.size(); ++j) { 
            if (j) ss << ','; 
            ss << q[j]; 
        }
        ss << ']';
    }
    ss << "],\"path\":[";
    for (size_t i = 0; i < traj.size(); ++i) {
        if (i) ss << ',';
        pinocchio::SE3 pose = rebot::computeFK(robot, traj[i].q);
        const auto& p = pose.translation();
        ss << '[' << p[0] << ',' << p[1] << ',' << p[2] << ']';
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
        std::string viewer = std::string(PROJECT_SOURCE_DIR) + "/test/traj_sim_viewer.py";
        execl(PYTHON_BIN, PYTHON_BIN, viewer.c_str(), nullptr);
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

    rebot::TrajPlanParams params;
    params.dt = 0.02;
    params.profile = rebot::TrajProfile::MIN_JERK;
    rebot::IKParams ik_params;
    ik_params.max_iter = 200;
    ik_params.tolerance = 1e-4;
    ik_params.step_size = 0.8;

    Eigen::VectorXd q = robot.neutralConfig();

    std::cout << "URDF : " << urdf << '\n'
              << "dt   : " << params.dt << " s\n"
              << "IK   : max_iter=" << ik_params.max_iter << " tol=" << ik_params.tolerance << '\n'
              << "输入: x y z [ro pi ya]  q 退出\n\n";

    std::string line;
    while (true) {
        const auto T_cur = rebot::computeFK(robot, q);
        const auto& p    = T_cur.translation();
        Eigen::Vector3d rpy = T_cur.rotation().eulerAngles(2, 1, 0).reverse();

        std::cout << std::fixed << std::setprecision(3)
                  << "当前 pos[" << p[0] << ' ' << p[1] << ' ' << p[2] << "]"
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

        std::cout << "目标 pos[" << x << ' ' << y << ' ' << z << "] rpy[" << ro << ' ' << pi_v << ' ' << ya << "]\n";

        auto t0 = Clock::now();
        rebot::IKResult ik_result = rebot::solveIK(robot, T_tgt, q, ik_params);
        double ik_ms = Ms(Clock::now() - t0).count();

        if (!ik_result.success) {
            std::cout << "IK 失败 误差=" << std::scientific << ik_result.error << "\n\n";
            continue;
        }

        double pos_dist = (T_tgt.translation() - p).norm();
        Eigen::Quaterniond qa(T_cur.rotation()), qb(R);
        if (qa.dot(qb) < 0) qb.coeffs() = -qb.coeffs();
        double angle_dist = 2.0 * std::acos(std::min(1.0, std::abs((qb * qa.inverse()).w())));
        double duration = std::max(0.5, std::max(pos_dist / 0.2, angle_dist / 1.0));

        std::cout << "  [1/3] IK 成功 " << std::fixed << ik_ms << " ms\n"
                  << "  [2/3] 规划测地线 (时长=" << duration << " s)..." << std::flush;

        t0 = Clock::now();
        std::vector<rebot::JointTrajectoryPoint> traj = rebot::planJointSpaceTrajectory(
            robot, q, ik_result.q, duration, params, ik_params, 0.1);
        double plan_ms = Ms(Clock::now() - t0).count();
        std::cout << " 完成 " << plan_ms << " ms 点数=" << traj.size() << "\n";

        std::cout << "  [3/3] 统计..." << std::flush;
        t0 = Clock::now();
        rebot::TrajStats stats = rebot::computeTrajStats(robot, traj, T_cur, T_tgt, duration, params);
        std::cout << " 成功率=" << std::fixed << std::setprecision(1) << (stats.success_rate * 100)
                  << "% 最大误差=" << std::scientific << stats.max_ik_error << "\n";

        if (!traj.empty()) q = traj.back().q;

#ifdef PYTHON_BIN
        if (viz && !traj.empty()) {
            std::string tag = "tgt[" + std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z) + "]";
            sendTraj(viz, robot, tag, params.dt, traj);
        }
#endif
        std::cout << '\n';
    }

#ifdef PYTHON_BIN
    if (viz) { fprintf(viz, "{\"cmd\":\"exit\"}\n"); fflush(viz); fclose(viz); }
    if (pid > 0) waitpid(pid, nullptr, 0);
#endif
    return 0;
}
