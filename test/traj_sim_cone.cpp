/**
 * traj_sim_cone — 末端定点，姿态在 RPY 空间形成圆锥闭合轨迹并往复循环
 * 定点位置固定在 (0.3, 0, 0)。
 * 期望姿态顺序：
 *   右下：RPY(3.14, 1.14,  0) →
 *   右侧：RPY(1.57, 1.14, -1.57) →
 *   顶部：RPY(3.14, 2.00,  0) →
 *   左侧：RPY(4.71, 1.14,  1.57) →
 *   回到右下：RPY(3.14, 1.14, 0)，形成圆锥一周，然后循环。
 * 程序先规划关节空间轨迹到起点 (3.14, 1.14, 0)，之后仅用 IK 跟随该闭合姿态轨迹。
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

constexpr double PX = 0.3, PY = 0.0, PZ = 0.0;
// 顶点/圆锥轴附近的姿态
constexpr double ROLL_TOP   = 3.14;
constexpr double PITCH_TOP  = 2.0;   // 顶部 (3.14, 2, 0)
constexpr double YAW_TOP    = 0.0;
// 圆锥底圈上的四个关键姿态（右下、右侧、左侧、闭合回右下）
constexpr double ROLL_RIGHT_DOWN = 3.14;
constexpr double PITCH_DOWN      = 1.14;
constexpr double YAW_RIGHT_DOWN  = 0.0;

constexpr double ROLL_RIGHT_SIDE = 1.57;
constexpr double PITCH_SIDE      = 1.14;
constexpr double YAW_RIGHT_SIDE  = -1.57;

constexpr double ROLL_LEFT_SIDE  = 4.71;   // 3π/2
constexpr double YAW_LEFT_SIDE   = 1.57;

constexpr int   SEGMENTS_PER_LOOP   = 4;    // 顶点与底圈 4 段
constexpr int   POINTS_PER_SEGMENT  = 64;
constexpr double DURATION_PER_LOOP  = 5.0;  // 圆锥一圈时长
constexpr double VIZ_DT             = DURATION_PER_LOOP / (SEGMENTS_PER_LOOP * POINTS_PER_SEGMENT);

inline Eigen::Matrix3d rpyToRot(double ro, double pi, double ya) {
    return (Eigen::AngleAxisd(ya, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pi, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(ro, Eigen::Vector3d::UnitX())).toRotationMatrix();
}

inline pinocchio::SE3 poseRPY(double roll, double pitch, double yaw) {
    return pinocchio::SE3(rpyToRot(roll, pitch, yaw), Eigen::Vector3d(PX, PY, PZ));
}

// 按 RPY 关键点构造圆锥一圈的笛卡尔轨迹：
// 右下 → 右侧 → 顶部 → 左侧 → 回到右下
rebot::CartesianTrajectory buildConeLoop() {
    using rebot::CartesianTrajectory;
    using rebot::TrajPlanParams;
    using rebot::TrajProfile;

    const pinocchio::SE3 pose_right_down = poseRPY(ROLL_RIGHT_DOWN, PITCH_DOWN, YAW_RIGHT_DOWN);   // (3.14, 1.14, 0)
    const pinocchio::SE3 pose_right_side = poseRPY(ROLL_RIGHT_SIDE, PITCH_SIDE, YAW_RIGHT_SIDE);   // (1.57, 1.14, -1.57)
    const pinocchio::SE3 pose_top        = poseRPY(ROLL_TOP,        PITCH_TOP,  YAW_TOP);          // (3.14, 2.0, 0)
    const pinocchio::SE3 pose_left_side  = poseRPY(ROLL_LEFT_SIDE,  PITCH_SIDE, YAW_LEFT_SIDE);    // (4.71, 1.14, 1.57)

    TrajPlanParams params;
    params.dt      = DURATION_PER_LOOP / (SEGMENTS_PER_LOOP * POINTS_PER_SEGMENT);
    params.profile = TrajProfile::MIN_JERK;

    const double seg_duration = DURATION_PER_LOOP / SEGMENTS_PER_LOOP;

    CartesianTrajectory traj;
    double t_offset = 0.0;

    auto append_segment = [&](const pinocchio::SE3& a, const pinocchio::SE3& b) {
        CartesianTrajectory seg =
            rebot::planCartesianGeodesicTrajectory(a, b, seg_duration, params);
        const auto& pts = seg.points();
        for (size_t i = 0; i < pts.size(); ++i) {
            // 避免重复添加相邻段的首点
            if (!traj.empty() && i == 0) continue;
            traj.addPoint(t_offset + pts[i].time, pts[i].pose);
        }
        t_offset += seg_duration;
    };

    // 半圈（右下 → 右侧 → 顶部）
    append_segment(pose_right_down, pose_right_side);
    append_segment(pose_right_side, pose_top);
    // 另一半（顶部 → 左侧 → 回到右下）
    append_segment(pose_top,       pose_left_side);
    append_segment(pose_left_side, pose_right_down);

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
    // 先把末端移动到圆锥起点姿态：右下 (3.14, 1.14, 0)
    auto ik_result = rebot::solveIK(robot, poseRPY(ROLL_RIGHT_DOWN, PITCH_DOWN, YAW_RIGHT_DOWN), q, ik_params);
    if (!ik_result.success) {
        std::cerr << "起点 IK 失败\n";
        return 1;
    }
    Eigen::Vector3d p0 = rebot::computeFK(robot, q).translation();
    double move_duration = std::max(0.5, (p0 - Eigen::Vector3d(PX, PY, PZ)).norm() / 0.2);
    auto go_traj = rebot::planJointSpaceTrajectory(robot, q, ik_result.q, move_duration, plan_params, ik_params, 0.1);
    q = go_traj.back().q;
#ifdef PYTHON_BIN
    sendTraj(robot, "go_start", go_traj, plan_params.dt);
#endif

    rebot::CartesianTrajectory cone = buildConeLoop();
    std::cout << "定点 (" << PX << "," << PY << "," << PZ
              << ") 姿态按 RPY(3.14,1.14,0)→(1.57,1.14,-1.57)→(3.14,2,0)→"
              << "(4.71,1.14,1.57)→(3.14,1.14,0) 形成圆锥并往复循环\n";
    int cycle = 0;
    while (true) {
        std::vector<rebot::JointTrajectoryPoint> joint_traj;
        for (const auto& pt : cone.points()) {
            auto ik = rebot::solveIK(robot, pt.pose, q, ik_params);
            q = ik.q;
            joint_traj.push_back({pt.time, q, ik.success});
        }
        int ok = 0;
        for (const auto& pt : joint_traj) if (pt.ik_success) ++ok;
        std::cout << "周期 " << ++cycle << " IK 成功 " << ok << "/" << joint_traj.size() << "\n";
#ifdef PYTHON_BIN
        sendTraj(robot, "cone", joint_traj);
#endif
    }
    return 0;
}
