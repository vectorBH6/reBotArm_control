#pragma once

/**
 * ArmController — 机械臂实机控制器
 *
 * 职责：URDF 加载、IK/FK、测地线轨迹规划、硬件驱动，一站式实机控制。
 * 核心状态（robot, q, ik_params, plan_params）均为 public，可直接读写与调试。
 *
 * 函数命名：
 *   move_to_XXX       — 规划 + 执行到目标位姿，XXX 标明所用算法
 *   move_through_XXX  — 规划 + 执行多点路径
 *   fk()              — 当前 q 的正运动学（便捷封装）
 *
 * 快速上手：
 *   ArmController arm_ctrl;
 *   arm_ctrl.init("/dev/ttyACM0");
 *   arm_ctrl.move_to_geodesic(ArmController::pose(0.4, 0, 0.3, M_PI, 0, 0));
 *   arm_ctrl.move_through_geodesic({pose1, pose2, pose3});
 *   while (arm_ctrl.running()) { ... }
 *   arm_ctrl.shutdown();
 *
 * 底层算法（computeFK / solveIK / planJointSpaceTrajectory 等）
 * 定义在 kinematics/ 中，可不经本类直接使用。
 */

#include "kinematics/robot_model.h"
#include "kinematics/forward_kinematics.h"
#include "kinematics/inverse_kinematics.h"
#include "kinematics/trajectory_planner_geodesic.h"
#include "actuator/arm_actuator_group.h"
#include "utils/control_loop.h"

#include <Eigen/Core>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <limits.h>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace rebot {

class ArmController {
public:

    // ── 常量 ─────────────────────────────────────────────────────────────────

    static constexpr double PLAN_HZ          = 50.0;  // 轨迹下发频率 [Hz]
    static constexpr double SEGMENT_DURATION = 1.5;   // 多点规划每段默认时长 [s]
    static constexpr double LINEAR_SPEED     = 0.2;   // 自动估算运动时长用的参考速度 [m/s]

    // ── 状态（可直接读写）────────────────────────────────────────────────────

    RobotModel      robot;       // pinocchio 模型与数据
    Eigen::VectorXd q;           // 当前关节角 [rad]
    IKParams        ik_params;   // IK 参数
    TrajPlanParams  plan_params; // 轨迹参数

    // ── 生命周期 ─────────────────────────────────────────────────────────────

    ArmController()  = default;
    ~ArmController() { shutdown(); }

    /** 完整初始化：加载模型 + 连接硬件 + 使能 + 启动 1kHz 控制循环。
     *  参数均有默认值：dev 默认 /dev/ttyACM0，yaml/urdf 空时自动推断路径。 */
    bool init(const std::string& dev       = "/dev/ttyACM0",
              const std::string& yaml_path = "",
              const std::string& urdf_path = "");

    /** 只读监视模式：加载模型 + 连接硬件，不使能、不启动控制循环。
     *  适用于 arm_state_monitor 等纯状态读取场景。 */
    bool init_monitor(const std::string& dev,
                      const std::string& yaml_path = "",
                      const std::string& urdf_path = "");

    /** Ctrl+C 信号后置 false，可用于主循环退出判断 */
    bool running() const { return running_; }

    /** 回零（关节角插值到 0）→ 停止控制循环 → 失能所有电机 */
    void shutdown();

    // ── 正运动学 ─────────────────────────────────────────────────────────────

    /** 当前关节角 q 的末端位姿 */
    pinocchio::SE3 fk() { return computeFK(robot, q); }

    /** 指定关节角的末端位姿 */
    pinocchio::SE3 fk(const Eigen::VectorXd& q_in) { return computeFK(robot, q_in); }

    // ── 实机执行 ─────────────────────────────────────────────────────────────

    /** 测地线轨迹规划 + 跟踪执行：SE(3) 空间最短路径平滑运动到 target */
    bool move_to_geodesic(const pinocchio::SE3& target);

    /** 纯 IK 直连：求解一次 IK 后直接下发关节角，无轨迹插值 */
    bool move_to_ik(const pinocchio::SE3& target);

    /** 依次经过 poses 中的每个位姿，各段使用测地线轨迹 */
    bool move_through_geodesic(const std::vector<pinocchio::SE3>& poses);

    // ── 硬件状态 ─────────────────────────────────────────────────────────────

    /** 主动查询所有电机状态并更新 q（监视模式下使用） */
    void refresh_hw();

    /** 返回各关节名称，顺序与配置文件一致 */
    std::vector<std::string> joint_names() const;

    // ── 工具 ─────────────────────────────────────────────────────────────────

    /** 由 x,y,z,roll,pitch,yaw(rad) 构造 SE3，ZYX 欧拉角顺序 */
    static pinocchio::SE3 pose(double x, double y, double z,
                               double roll = 0, double pitch = 0, double yaw = 0);

    // ── 预留扩展（按需取消注释并实现）──────────────────────────────────────
    // bool move_to_impedance(const pinocchio::SE3& target, double kp, double kd);
    // bool enable_gravity_comp(double kd_damp = 1.0f);

private:

    bool init_impl_(const std::string& dev, const std::string& yaml_path,
                    const std::string& urdf_path, bool monitor_only);

    // 内部规划（仅被 move_to_geodesic 调用）
    std::vector<JointTrajectoryPoint> plan_geodesic_(
        const pinocchio::SE3& target, double duration = 0.0);

    void sync_q_();
    void apply_q_to_targets_(const Eigen::VectorXd& q_val);
    void run_trajectory_(const std::vector<JointTrajectoryPoint>& traj,
                         bool interruptible, float vel_limit = -1.f);

    static volatile bool running_;
    static void on_signal_(int) { running_ = false; }

    std::unique_ptr<actuator::ArmActuatorGroup> arm_;
    std::unique_ptr<ControlLoop>                ctrl_loop_;
    std::vector<float>  targets_;
    std::mutex          targets_mtx_;
    std::atomic<float>  speed_limit_{-1.f};
    int  nq_           = 0;
    bool inited_       = false;
    bool monitor_only_ = false;
};

}  // namespace rebot
