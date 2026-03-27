/**
 * rebot_py — Python bindings for reBot arm control
 *
 * 将 rebot 命名空间下的运动学、动力学、轨迹规划、应用层控制器
 * 绑定为 Python 模块 `rebot_py`，支持 numpy 数组无拷贝互操作。
 *
 * SE3 在 Python 侧表示为 4×4 numpy 矩阵（np.ndarray, shape=(4,4), dtype=float64）。
 */

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "application/arm_controller.h"
#include "kinematics/robot_model.h"
#include "kinematics/forward_kinematics.h"
#include "kinematics/inverse_kinematics.h"
#include "kinematics/trajectory_planner_geodesic.h"
#include "dynamics/gravity_compensation.h"

#include <pinocchio/spatial/se3.hpp>
#include <Eigen/Core>
#include <vector>
#include <string>

namespace py = pybind11;
using namespace rebot;

// ─── SE3 <-> 4×4 numpy 转换工具 ──────────────────────────────────────────────

static Eigen::Matrix4d se3_to_mat4(const pinocchio::SE3& T)
{
    return T.toHomogeneousMatrix();
}

static pinocchio::SE3 mat4_to_se3(const Eigen::Matrix4d& m)
{
    return pinocchio::SE3(m.block<3,3>(0,0), m.block<3,1>(0,3));
}

// ─── pybind11 类型 caster：pinocchio::SE3 <-> 4×4 numpy ─────────────────────

namespace pybind11 { namespace detail {

template <> struct type_caster<pinocchio::SE3> {
public:
    PYBIND11_TYPE_CASTER(pinocchio::SE3, const_name("numpy.ndarray[4,4]"));

    bool load(handle src, bool /* convert */) {
        try {
            auto mat = src.cast<Eigen::Matrix4d>();
            value = mat4_to_se3(mat);
            return true;
        } catch (...) {
            return false;
        }
    }

    static handle cast(const pinocchio::SE3& src, return_value_policy, handle) {
        Eigen::Matrix4d mat = se3_to_mat4(src);
        return py::cast(mat).release();
    }
};

}}  // namespace pybind11::detail

// ─── 模块定义 ─────────────────────────────────────────────────────────────────

PYBIND11_MODULE(rebot_py, m)
{
    m.doc() = "reBot 机械臂 Python 控制库\n\n"
              "SE3 位姿在 Python 侧统一表示为 4×4 numpy 矩阵。\n"
              "Eigen::VectorXd 自动映射为 1-D numpy 数组。";

    // ── 工具函数 ─────────────────────────────────────────────────────────────

    m.def("make_pose", &make_pose,
          py::arg("x"), py::arg("y"), py::arg("z"),
          py::arg("roll") = 0.0, py::arg("pitch") = 0.0, py::arg("yaw") = 0.0,
          "由 x,y,z,roll,pitch,yaw(rad) 构造 4×4 位姿矩阵（ZYX 欧拉角）");

    // ── RobotModel ───────────────────────────────────────────────────────────

    py::class_<RobotModel>(m, "RobotModel",
        "URDF 机器人模型（Pinocchio 封装）")
        .def(py::init<>())
        .def(py::init<const std::string&, const std::string&>(),
             py::arg("urdf_path"),
             py::arg("end_frame") = "end_link")
        .def("nq", &RobotModel::nq, "关节自由度")
        .def("nv", &RobotModel::nv, "速度自由度")
        .def("neutral_config", &RobotModel::neutralConfig, "零位构型")
        .def("is_config_valid", &RobotModel::isConfigValid, py::arg("q"),
             "检查关节角是否在限位范围内")
        .def("clamp_config", &RobotModel::clampConfig, py::arg("q"),
             "将关节角截断到限位范围");

    // ── 正运动学 ─────────────────────────────────────────────────────────────

    m.def("compute_fk", &computeFK,
          py::arg("robot"), py::arg("q"),
          "正运动学：关节角 → 末端 4×4 位姿");

    m.def("compute_all_frame_poses", &computeAllFramePoses,
          py::arg("robot"), py::arg("q"),
          "计算所有 frame 的位姿（返回 list[4×4]）");

    m.def("compute_end_jacobian", &computeEndJacobian,
          py::arg("robot"), py::arg("q"),
          "末端雅可比矩阵 (6×nv)");

    // ── 逆运动学 ─────────────────────────────────────────────────────────────

    py::class_<IKParams>(m, "IKParams", "逆运动学参数")
        .def(py::init<>())
        .def_readwrite("max_iter",  &IKParams::max_iter)
        .def_readwrite("tolerance", &IKParams::tolerance)
        .def_readwrite("step_size", &IKParams::step_size)
        .def_readwrite("damping",   &IKParams::damping)
        .def("__repr__", [](const IKParams& p) {
            return "IKParams(max_iter=" + std::to_string(p.max_iter) +
                   ", tol=" + std::to_string(p.tolerance) +
                   ", step=" + std::to_string(p.step_size) +
                   ", damp=" + std::to_string(p.damping) + ")";
        });

    py::class_<IKResult>(m, "IKResult", "逆运动学求解结果")
        .def_readonly("q",          &IKResult::q)
        .def_readonly("success",    &IKResult::success)
        .def_readonly("error",      &IKResult::error)
        .def_readonly("iterations", &IKResult::iterations)
        .def("__repr__", [](const IKResult& r) {
            return std::string("IKResult(success=") + (r.success ? "True" : "False") +
                   ", error=" + std::to_string(r.error) +
                   ", iter=" + std::to_string(r.iterations) + ")";
        });

    m.def("solve_ik", &solveIK,
          py::arg("robot"), py::arg("target"), py::arg("q_init"),
          py::arg("params") = IKParams{},
          "阻尼最小二乘雅可比逆运动学（CLIK）");

    m.def("solve_ik_with_retry", &solveIKWithRetry,
          py::arg("robot"), py::arg("target"), py::arg("q_seed"),
          py::arg("params") = IKParams{}, py::arg("max_retries") = 8,
          "带随机重试的 IK，返回误差最小的结果");

    // ── 轨迹规划 ─────────────────────────────────────────────────────────────

    py::enum_<TrajProfile>(m, "TrajProfile", "轨迹速度曲线类型")
        .value("LINEAR",    TrajProfile::LINEAR)
        .value("MIN_JERK",  TrajProfile::MIN_JERK)
        .value("TRAPEZOID", TrajProfile::TRAPEZOID);

    py::class_<TrajPlanParams>(m, "TrajPlanParams", "轨迹规划参数")
        .def(py::init<>())
        .def_readwrite("dt",          &TrajPlanParams::dt)
        .def_readwrite("profile",     &TrajPlanParams::profile)
        .def_readwrite("accel_ratio", &TrajPlanParams::accel_ratio);

    py::class_<JointTrajectoryPoint>(m, "JointTrajectoryPoint", "关节轨迹点")
        .def_readonly("time",       &JointTrajectoryPoint::time)
        .def_readonly("q",          &JointTrajectoryPoint::q)
        .def_readonly("ik_success", &JointTrajectoryPoint::ik_success);

    py::class_<TrajStats>(m, "TrajStats", "轨迹质量统计")
        .def_readonly("total_points",  &TrajStats::total_points)
        .def_readonly("success_count", &TrajStats::success_count)
        .def_readonly("success_rate",  &TrajStats::success_rate)
        .def_readonly("max_ik_error",  &TrajStats::max_ik_error)
        .def_readonly("avg_ik_error",  &TrajStats::avg_ik_error);

    m.def("plan_cartesian_geodesic_trajectory", &planCartesianGeodesicTrajectory,
          py::arg("start_pose"), py::arg("end_pose"),
          py::arg("duration"),
          py::arg("params") = TrajPlanParams{},
          "测地线笛卡尔轨迹规划");

    m.def("plan_joint_space_trajectory",
          [](RobotModel& robot,
             const Eigen::VectorXd& q_start,
             const Eigen::VectorXd& q_end,
             double duration,
             const TrajPlanParams& params,
             const IKParams& ik_params,
             double null_gain)
          {
              return planJointSpaceTrajectory(
                  robot, q_start, q_end, duration,
                  params, ik_params, null_gain);
          },
          py::arg("robot"), py::arg("q_start"), py::arg("q_end"),
          py::arg("duration"),
          py::arg("params") = TrajPlanParams{},
          py::arg("ik_params") = IKParams{},
          py::arg("null_gain") = 0.1,
          "关节空间测地线轨迹规划");

    m.def("compute_traj_stats",
          [](RobotModel& robot,
             const std::vector<JointTrajectoryPoint>& joint_traj,
             const Eigen::Matrix4d& T_start_mat,
             const Eigen::Matrix4d& T_end_mat,
             double duration,
             const TrajPlanParams& params)
          {
              pinocchio::SE3 T_start = mat4_to_se3(T_start_mat);
              pinocchio::SE3 T_end   = mat4_to_se3(T_end_mat);
              return computeTrajStats(robot, joint_traj, T_start, T_end,
                                     duration, params);
          },
          py::arg("robot"), py::arg("joint_traj"),
          py::arg("T_start"), py::arg("T_end"),
          py::arg("duration"),
          py::arg("params") = TrajPlanParams{},
          "统计关节轨迹相对笛卡尔目标的误差");

    // ── CartesianTrajectory ──────────────────────────────────────────────────

    py::class_<CartesianTrajectory>(m, "CartesianTrajectory", "笛卡尔轨迹")
        .def(py::init<>())
        .def("add_point", [](CartesianTrajectory& self, double t,
                             const Eigen::Matrix4d& pose_mat) {
            self.addPoint(t, mat4_to_se3(pose_mat));
        }, py::arg("t"), py::arg("pose"))
        .def("duration", &CartesianTrajectory::duration)
        .def("empty", &CartesianTrajectory::empty)
        .def("__len__", [](const CartesianTrajectory& self) {
            return self.points().size();
        });

    m.def("track_trajectory", &trackTrajectory,
          py::arg("robot"), py::arg("traj"), py::arg("q_init"),
          py::arg("ik_params") = IKParams{}, py::arg("null_gain") = 0.1,
          "CLIK 跟踪笛卡尔轨迹");

    // ── 重力补偿 ─────────────────────────────────────────────────────────────

    py::class_<GravityCompensator>(m, "GravityCompensator",
        "重力补偿力矩计算器（基于 Pinocchio RNEA）")
        .def(py::init<RobotModel&>(), py::arg("model"),
             py::keep_alive<1, 2>())
        .def("compute", &GravityCompensator::compute, py::arg("q"),
             "计算关节构型 q 对应的重力补偿力矩 [N·m]")
        .def("nq", &GravityCompensator::nq)
        .def("nv", &GravityCompensator::nv);

    // ── ArmController（应用层）────────────────────────────────────────────────

    py::class_<ArmController>(m, "ArmController",
        "机械臂实机控制器 — 一站式 URDF+IK/FK+轨迹+硬件控制\n\n"
        "快速上手:\n"
        "  arm = rebot_py.ArmController()\n"
        "  arm.init('/dev/ttyACM0')\n"
        "  arm.move_to_geodesic(rebot_py.make_pose(0.4, 0, 0.3, 3.14, 0, 0))\n"
        "  arm.shutdown()")
        .def(py::init<>())

        // 常量
        .def_readonly_static("PLAN_HZ",          &ArmController::PLAN_HZ)
        .def_readonly_static("SEGMENT_DURATION",  &ArmController::SEGMENT_DURATION)
        .def_readonly_static("LINEAR_SPEED",      &ArmController::LINEAR_SPEED)

        // 公开状态
        .def_readwrite("robot",       &ArmController::robot)
        .def_readwrite("ik_params",   &ArmController::ik_params)
        .def_readwrite("plan_params", &ArmController::plan_params)
        .def_property("q",
            [](const ArmController& self) { return self.q; },
            [](ArmController& self, const Eigen::VectorXd& v) { self.q = v; })

        // 生命周期
        .def("init", &ArmController::init,
             py::arg("dev") = "/dev/ttyACM0",
             py::arg("yaml_path") = "",
             py::arg("urdf_path") = "",
             "完整初始化：加载模型 + 连接硬件 + 使能 + 启动控制循环")
        .def("init_monitor", &ArmController::init_monitor,
             py::arg("dev"),
             py::arg("yaml_path") = "",
             py::arg("urdf_path") = "",
             "只读监视模式：加载模型 + 连接硬件，不使能")
        .def("running", &ArmController::running)
        .def("shutdown", &ArmController::shutdown,
             "回零 → 停止控制循环 → 失能电机")

        // 正运动学
        .def("fk", py::overload_cast<>(&ArmController::fk),
             "当前 q 的末端位姿（4×4 矩阵）")
        .def("fk", py::overload_cast<const Eigen::VectorXd&>(&ArmController::fk),
             py::arg("q"), "指定关节角的末端位姿")

        // 实机执行
        .def("move_to_geodesic", &ArmController::move_to_geodesic,
             py::arg("target"),
             "测地线轨迹平滑运动到目标位姿")
        .def("move_to_ik", &ArmController::move_to_ik,
             py::arg("target"),
             "纯 IK 直连到目标位姿（无轨迹插值，谨慎使用）")
        .def("move_through_geodesic",
             [](ArmController& self, const std::vector<Eigen::Matrix4d>& pose_mats) {
                 std::vector<pinocchio::SE3> poses;
                 poses.reserve(pose_mats.size());
                 for (const auto& m : pose_mats)
                     poses.push_back(mat4_to_se3(m));
                 return self.move_through_geodesic(poses);
             },
             py::arg("poses"),
             "依次经过多个位姿，各段使用测地线轨迹")

        // 硬件状态
        .def("refresh_hw", &ArmController::refresh_hw,
             "查询电机状态并更新 q")
        .def("joint_names", &ArmController::joint_names,
             "关节名称列表")

        // 工具
        .def_static("pose", &ArmController::pose,
             py::arg("x"), py::arg("y"), py::arg("z"),
             py::arg("roll") = 0.0, py::arg("pitch") = 0.0, py::arg("yaw") = 0.0,
             "由 x,y,z,roll,pitch,yaw(rad) 构造 4×4 位姿矩阵");
}
