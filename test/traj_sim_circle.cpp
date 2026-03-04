/**
 * traj_sim_circle — 末端 YZ 平面画圆仿真（正一圈反一圈循环，防电机超圈）
 * 圆心 (0.4, 0, 0.35)，半径 25cm，姿态 (3.14, 0, 0)。先轨迹规划到起点，画圆仅用逆运动学。
 * 可选 MeshCat 可视化：编译时定义 PYTHON_BIN / PROJECT_SOURCE_DIR 则启动 traj_sim_viewer.py
 */

#include <cmath>
#include <csignal>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <unistd.h>
#include <sys/wait.h>

#include "kinematics/robot_model.h"
#include "kinematics/forward_kinematics.h"
#include "kinematics/inverse_kinematics.h"
#include "kinematics/trajectory_planner_geodesic.h"

namespace {

constexpr double CX = 0.4, CY = 0.0, CZ = 0.35;
constexpr double RADIUS = 0.2;
constexpr double ROLL = 3.14, PITCH = 0.0, YAW = 0.0;
constexpr int POINTS_PER_CIRCLE = 128;
constexpr double DURATION_PER_CIRCLE = 5.0;
constexpr double VIZ_DT = DURATION_PER_CIRCLE / POINTS_PER_CIRCLE;
constexpr double THETA_START = M_PI / 2;  // 画圆起点 (0.4, 0, CZ+RADIUS)

inline Eigen::Matrix3d rpyToRot(double ro, double pi, double ya) {
    return (Eigen::AngleAxisd(ya, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pi, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(ro, Eigen::Vector3d::UnitX())).toRotationMatrix();
}

inline pinocchio::SE3 poseAt(double theta) {
    double y = CY + RADIUS * std::cos(theta);
    double z = CZ + RADIUS * std::sin(theta);
    return pinocchio::SE3(rpyToRot(ROLL, PITCH, YAW), Eigen::Vector3d(CX, y, z));
}

rebot::CartesianTrajectory buildForwardBackwardCircle() {
    rebot::CartesianTrajectory traj;
    const double dt = DURATION_PER_CIRCLE / POINTS_PER_CIRCLE;
    for (int i = 0; i <= POINTS_PER_CIRCLE; ++i) {
        double th = THETA_START + 2.0 * M_PI * i / POINTS_PER_CIRCLE;
        traj.addPoint(i * dt, poseAt(th));
    }
    for (int i = POINTS_PER_CIRCLE - 1; i >= 0; --i) {
        double th = THETA_START + 2.0 * M_PI * i / POINTS_PER_CIRCLE;
        traj.addPoint((2 * POINTS_PER_CIRCLE - i) * dt, poseAt(th));
    }
    return traj;
}

} // namespace

#ifdef PYTHON_BIN
static FILE* g_viz = nullptr;
static pid_t g_viz_pid = -1;

static void sendTraj(rebot::RobotModel& robot, const std::string& name,
                    const std::vector<rebot::JointTrajectoryPoint>& traj, double dt = VIZ_DT) {
    if (!g_viz || traj.empty()) return;
    std::ostringstream ss;
    ss << std::setprecision(6) << "{\"name\":\"" << name << "\",\"dt\":" << dt << ",\"q_list\":[";
    for (size_t i = 0; i < traj.size(); ++i) {
        if (i) ss << ',';
        ss << '[';
        for (int j = 0; j < traj[i].q.size(); ++j) { if (j) ss << ','; ss << traj[i].q[j]; }
        ss << ']';
    }
    ss << "],\"path\":[";
    for (size_t i = 0; i < traj.size(); ++i) {
        if (i) ss << ',';
        const auto& p = rebot::computeFK(robot, traj[i].q).translation();
        ss << '[' << p[0] << ',' << p[1] << ',' << p[2] << ']';
    }
    ss << "]}\n";
    fputs(ss.str().c_str(), g_viz);
    fflush(g_viz);
}

static void viz_exit(int) {
    if (g_viz) { fprintf(g_viz, "{\"cmd\":\"exit\"}\n"); fflush(g_viz); fclose(g_viz); g_viz = nullptr; }
    if (g_viz_pid > 0) { waitpid(g_viz_pid, nullptr, 0); g_viz_pid = -1; }
    _exit(0);
}
#endif

int main(int argc, char** argv) {
    const std::string urdf = (argc > 1) ? argv[1] : URDF_PATH;
    rebot::RobotModel robot(urdf);

#ifdef PYTHON_BIN
    if (signal(SIGINT, viz_exit) == SIG_ERR) { /* ignore */ }
    int pipe_fd[2];
    if (pipe(pipe_fd) == 0 && (g_viz_pid = fork()) == 0) {
        close(pipe_fd[1]);
        dup2(pipe_fd[0], STDIN_FILENO);
        close(pipe_fd[0]);
        execl(PYTHON_BIN, PYTHON_BIN, (std::string(PROJECT_SOURCE_DIR) + "/test/traj_sim_viewer.py").c_str(), nullptr);
        _exit(1);
    }
    if (g_viz_pid > 0) {
        close(pipe_fd[0]);
        g_viz = fdopen(pipe_fd[1], "w");
        fprintf(g_viz, "%s\n", urdf.c_str());
        fflush(g_viz);
        sleep(3);
    }
#endif

    rebot::IKParams ik_params;
    ik_params.max_iter = 200;
    ik_params.tolerance = 1e-4;
    ik_params.step_size = 0.8;

    rebot::TrajPlanParams plan_params;
    plan_params.dt = 0.02;
    plan_params.profile = rebot::TrajProfile::MIN_JERK;

    Eigen::VectorXd q = robot.neutralConfig();
    // 先移动到画圆起点 (0.4, 0, 0.4+0.25)，再画圆，避免仿真初态逻辑错误
    auto ik_result = rebot::solveIK(robot, poseAt(THETA_START), q, ik_params);
    if (!ik_result.success) {
        std::cerr << "起点 IK 失败\n";
        return 1;
    }
    Eigen::Vector3d p0 = rebot::computeFK(robot, q).translation();
    double move_duration = std::max(0.5, (p0 - Eigen::Vector3d(CX, CY, CZ + RADIUS)).norm() / 0.2);
    auto go_traj = rebot::planJointSpaceTrajectory(robot, q, ik_result.q, move_duration, plan_params, ik_params, 0.1);
    q = go_traj.back().q;
#ifdef PYTHON_BIN
    sendTraj(robot, "go_start", go_traj, plan_params.dt);
#endif

    rebot::CartesianTrajectory circle = buildForwardBackwardCircle();
    std::cout << "圆心 (" << CX << "," << CY << "," << CZ << ") 半径 " << RADIUS << " m YZ 平面 正反画圆循环\n";
    int cycle = 0;
    while (true) {
        std::vector<rebot::JointTrajectoryPoint> joint_traj;
        for (const auto& pt : circle.points()) {
            auto ik = rebot::solveIK(robot, pt.pose, q, ik_params);
            q = ik.q;
            joint_traj.push_back({pt.time, q, ik.success});
        }
        int ok = 0;
        for (const auto& pt : joint_traj) if (pt.ik_success) ++ok;
        std::cout << "周期 " << ++cycle << " IK 成功 " << ok << "/" << joint_traj.size() << "\n";
#ifdef PYTHON_BIN
        sendTraj(robot, "circle", joint_traj);
#endif
    }
    return 0;
}
