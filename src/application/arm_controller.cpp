#include "application/arm_controller.h"

#include <Eigen/Geometry>
#include <unistd.h>
#include <cmath>

namespace rebot {

volatile bool ArmController::running_ = true;

// ─── 工具 ─────────────────────────────────────────────────────────────────────

pinocchio::SE3 ArmController::pose(double x, double y, double z,
                                    double roll, double pitch, double yaw)
{
    return make_pose(x, y, z, roll, pitch, yaw);
}

// 返回可执行文件所在目录的上一级（项目根），用于推断默认配置路径
static std::string exe_parent_dir()
{
    char buf[PATH_MAX]{};
    ssize_t n = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (n <= 0) return ".";
    std::string p(buf, n);
    p = p.substr(0, p.rfind('/'));
    return p.substr(0, p.rfind('/'));
}

// ─── 初始化 ───────────────────────────────────────────────────────────────────

bool ArmController::init_impl_(const std::string& dev,
                               const std::string& yaml_path,
                               const std::string& urdf_path,
                               bool monitor_only)
{
    // 1. 加载模型
    std::string urdf = urdf_path.empty()
#ifdef URDF_PATH
        ? std::string(URDF_PATH)
#else
        ? exe_parent_dir() + "/../urdf/reBot-DevArm_fixend_description/urdf/reBot-DevArm_fixend.urdf"
#endif
        : urdf_path;

    printf("[model] %s\n", urdf.c_str());
    robot = RobotModel(urdf);
    nq_   = robot.nq();

    plan_params.dt      = 1.0 / PLAN_HZ;
    plan_params.profile = TrajProfile::MIN_JERK;
    ik_params.max_iter  = 200;
    ik_params.tolerance = 1e-4;
    ik_params.step_size = 0.8;

    // 2. 连接执行器
    std::string yaml = yaml_path.empty()
        ? exe_parent_dir() + "/config/arm.yaml" : yaml_path;

    printf("[arm] 连接 %s ...\n", dev.c_str());
    try {
        arm_.reset(new actuator::ArmActuatorGroup(
            actuator::ArmActuatorGroup::from_yaml(dev, yaml)));
    } catch (const std::exception& e) {
        fprintf(stderr, "[错误] %s\n", e.what());
        return false;
    }

    const int N = static_cast<int>(arm_->size());
    if (N < nq_) {
        fprintf(stderr, "[错误] 执行器数(%d) < 关节数(%d)\n", N, nq_);
        arm_.reset();
        return false;
    }

    // 3a. 监视模式：读取当前状态后返回，不使能
    if (monitor_only) {
        monitor_only_ = true;
        q.resize(nq_);
        sync_q_();
        inited_ = true;
        return true;
    }

    // 3b. 正常模式：扫描连接 → 使能 → 同步 q → 启动控制循环
    monitor_only_ = false;
    auto connected = arm_->scan_connectivity();
    for (int i = 0; i < N; ++i)
        printf("  [%d] %s: %s\n", i, arm_->joint_names()[i].c_str(),
               connected[i] ? "已连接" : "无响应");

    printf("[arm] 使能...\n");
    arm_->enable_all();
    sleep(1);

    q.resize(nq_);
    sync_q_();
    targets_.resize(N);
    {
        std::lock_guard<std::mutex> lk(targets_mtx_);
        for (int i = 0; i < nq_ && i < N; ++i)
            targets_[i] = static_cast<float>(q[i]);
    }

    ctrl_loop_ = std::make_unique<ControlLoop>(1000.0, [this]() -> bool {
        std::vector<float> tgt;
        { std::lock_guard<std::mutex> lk(targets_mtx_); tgt = targets_; }
        float sl = speed_limit_.load(std::memory_order_relaxed);
        if (sl > 0.f)
            arm_->set_all_positions_with_speed_limit(tgt, sl);
        else
            arm_->set_all_positions(tgt);
        return true;
    });

    inited_ = true;
    return true;
}

bool ArmController::init(const std::string& dev,
                         const std::string& yaml_path,
                         const std::string& urdf_path)
{
    if (inited_) return true;
    running_ = true;

    struct sigaction sa{};
    sa.sa_handler = on_signal_;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT,  &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);

    if (!init_impl_(dev, yaml_path, urdf_path, false)) return false;
    printf("[arm] 就绪\n");
    return true;
}

bool ArmController::init_monitor(const std::string& dev,
                                 const std::string& yaml_path,
                                 const std::string& urdf_path)
{
    if (inited_) return true;
    running_ = true;
    return init_impl_(dev, yaml_path, urdf_path, true);
}

// ─── 硬件状态 ─────────────────────────────────────────────────────────────────

void ArmController::refresh_hw()
{
    if (!inited_ || !arm_) return;
    const int N = static_cast<int>(arm_->size());
    for (int i = 0; i < N; ++i)
        (*arm_)[i].refresh_status();
    sync_q_();
}

std::vector<std::string> ArmController::joint_names() const
{
    if (!arm_) return {};
    return arm_->joint_names();
}

// ─── 内部：硬件读写辅助 ───────────────────────────────────────────────────────

void ArmController::sync_q_()
{
    auto pos = arm_->get_all_positions();
    q.resize(pos.size());
    for (size_t i = 0; i < pos.size(); ++i) q[i] = pos[i];
    if (static_cast<int>(q.size()) > nq_) q.conservativeResize(nq_);
}

void ArmController::apply_q_to_targets_(const Eigen::VectorXd& q_val)
{
    std::lock_guard<std::mutex> lk(targets_mtx_);
    for (int i = 0; i < nq_ && i < static_cast<int>(q_val.size()); ++i)
        targets_[i] = static_cast<float>(q_val[i]);
}

// 按 plan_params.dt 的节拍逐帧下发轨迹；interruptible=true 时响应 Ctrl+C 中断
void ArmController::run_trajectory_(const std::vector<JointTrajectoryPoint>& traj,
                                    bool interruptible, float vel_limit)
{
    if (traj.empty()) return;
    speed_limit_.store(vel_limit, std::memory_order_relaxed);
    for (const auto& pt : traj) {
        if (interruptible && !running_) break;
        apply_q_to_targets_(pt.q);
        std::this_thread::sleep_for(std::chrono::duration<double>(plan_params.dt));
    }
    speed_limit_.store(-1.f, std::memory_order_relaxed);
}

// ─── 内部：轨迹规划 ───────────────────────────────────────────────────────────

// 单点测地线规划：当前 q → target
// duration <= 0 时根据末端直线距离 / LINEAR_SPEED 自动估算
std::vector<JointTrajectoryPoint> ArmController::plan_geodesic_(
    const pinocchio::SE3& target, double duration)
{
    IKResult ik = solveIK(robot, target, q, ik_params);
    if (!ik.success) return {};

    const pinocchio::SE3 T_cur = computeFK(robot, q);
    const pinocchio::SE3 T_end = computeFK(robot, ik.q);

    if (duration <= 0.0)
        duration = std::max(1.0,
            (target.translation() - T_cur.translation()).norm() / LINEAR_SPEED);

    return planJointSpaceTrajectory(robot, q, ik.q, duration,
                                    plan_params, ik_params, 0.1,
                                    &T_cur, &T_end);
}

// 多点测地线规划：依次规划各段，q 滚动推进，结果写入 out_segs
bool ArmController::plan_geodesic_multi_(
    const std::vector<pinocchio::SE3>&              poses,
    std::vector<std::vector<JointTrajectoryPoint>>& out_segs,
    double                                          segment_dur)
{
    out_segs.clear();
    out_segs.reserve(poses.size());

    Eigen::VectorXd q_cur  = q;
    pinocchio::SE3  T_cur  = computeFK(robot, q_cur);

    for (const auto& target : poses) {
        IKResult ik = solveIK(robot, target, q_cur, ik_params);
        if (!ik.success) return false;

        const pinocchio::SE3 T_end = computeFK(robot, ik.q);
        auto seg = planJointSpaceTrajectory(robot, q_cur, ik.q, segment_dur,
                                            plan_params, ik_params, 0.1,
                                            &T_cur, &T_end);
        if (!seg.empty()) {
            q_cur = seg.back().q;
            T_cur = T_end;
        }
        out_segs.push_back(std::move(seg));
    }
    q = q_cur;
    return true;
}

// ─── 实机执行 ─────────────────────────────────────────────────────────────────

bool ArmController::move_to_geodesic(const pinocchio::SE3& target)
{
    if (!inited_) return false;
    auto traj = plan_geodesic_(target);
    if (traj.empty()) return false;
    run_trajectory_(traj, true);
    q = traj.back().q;
    return true;
}

bool ArmController::move_to_ik(const pinocchio::SE3& target)
{
    if (!inited_) return false;
    IKResult ik = solveIK(robot, target, q, ik_params);
    if (!ik.success) return false;
    q = ik.q;
    apply_q_to_targets_(q);
    return true;
}

bool ArmController::move_through_geodesic(const std::vector<pinocchio::SE3>& poses)
{
    if (!inited_) return false;
    if (poses.empty()) return true;
    if (poses.size() == 1) return move_to_geodesic(poses[0]);

    std::vector<std::vector<JointTrajectoryPoint>> segs;
    Eigen::VectorXd q_save = q;

    if (!plan_geodesic_multi_(poses, segs)) {
        q = q_save;
        return false;
    }

    q = q_save;
    for (size_t i = 0; i < segs.size() && running_; ++i) {
        run_trajectory_(segs[i], true);
        if (!segs[i].empty()) q = segs[i].back().q;
    }
    return true;
}

// ─── 关闭 ─────────────────────────────────────────────────────────────────────

void ArmController::shutdown()
{
    if (!inited_) return;

    if (monitor_only_) {
        arm_.reset();
        inited_ = monitor_only_ = false;
        return;
    }

    printf("\n[arm] 回零...\n");
    const double dur = std::max(3.0, q.cwiseAbs().maxCoeff() / 0.3);
    const Eigen::VectorXd q_zero = Eigen::VectorXd::Zero(nq_);
    const pinocchio::SE3  T_zero = computeFK(robot, q_zero);
    auto traj = planJointSpaceTrajectory(robot, q, q_zero, dur,
                                         plan_params, ik_params,
                                         0.1, nullptr, &T_zero);
    run_trajectory_(traj, false, 0.6f);
    ctrl_loop_->stop();

    printf("[arm] 失能\n");
    arm_->disable_all();
    inited_ = false;
}

}  // namespace rebot
