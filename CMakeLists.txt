cmake_minimum_required(VERSION 3.10)
project(reBot_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ─── 路径配置 ──────────────────────────────────────────────────────────────────
set(PINOCCHIO_ROOT  /home/rvector/pinocchio)
set(PINOCCHIO_BUILD ${PINOCCHIO_ROOT}/build)
set(URDF_PATH       "${CMAKE_SOURCE_DIR}/models/reBot-DevArm_description/urdf/reBot-DevArm.urdf")

# ─── 依赖 ──────────────────────────────────────────────────────────────────────
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

set(PIN_INCLUDES
    ${PINOCCHIO_ROOT}/include
    ${PINOCCHIO_BUILD}/include
    ${EIGEN3_INCLUDE_DIR}
)
set(PIN_LIBS
    ${PINOCCHIO_BUILD}/src/libpinocchio_default.so
    ${PINOCCHIO_BUILD}/src/libpinocchio_parsers.so
)
set(PIN_RPATH ${PINOCCHIO_BUILD}/src)

# ─── 运动学库 ──────────────────────────────────────────────────────────────────
add_library(rebot_kinematics
    src/kinematics/robot_model.cpp
    src/kinematics/forward_kinematics.cpp
    src/kinematics/inverse_kinematics.cpp
    src/kinematics/end_motion_stepper.cpp
    src/kinematics/trajectory_planner_geodesic.cpp
)

target_include_directories(rebot_kinematics PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${PIN_INCLUDES}
)

target_link_libraries(rebot_kinematics PUBLIC ${PIN_LIBS})

set_target_properties(rebot_kinematics PROPERTIES
    BUILD_RPATH  "${PIN_RPATH}"
    INSTALL_RPATH "${PIN_RPATH}"
)

# ─── 仿真测试 ──────────────────────────────────────────────────────────────────
add_executable(kinematics_test test/kinematics_test.cpp)
target_link_libraries(kinematics_test rebot_kinematics)
target_compile_definitions(kinematics_test PRIVATE URDF_PATH="${URDF_PATH}")

set_target_properties(kinematics_test PROPERTIES
    BUILD_RPATH  "${PIN_RPATH}"
    INSTALL_RPATH "${PIN_RPATH}"
)

enable_testing()
add_test(NAME kinematics_test COMMAND kinematics_test)

# ─── 执行器抽象层 ──────────────────────────────────────────────────────────────
add_library(rebot_actuator STATIC
    src/actuator/arm_actuator_group.cpp
)
target_include_directories(rebot_actuator PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(rebot_actuator PUBLIC yaml-cpp)

# ─── 机械臂多关节 Web 拖动控制测试 ────────────────────────────────────────────
add_executable(arm_web_ctrl test/arm_web_ctrl.cpp)
target_link_libraries(arm_web_ctrl PRIVATE rebot_actuator)

# ─── IK 可视化测试 ─────────────────────────────────────────────────────────────
set(PYTHON_BIN "/home/rvector/anaconda3/envs/pinocchio/bin/python3")
add_executable(ik_viz test/ik_viz.cpp)
target_link_libraries(ik_viz rebot_kinematics)
target_compile_definitions(ik_viz PRIVATE
    URDF_PATH="${URDF_PATH}"
    PYTHON_BIN="${PYTHON_BIN}"
    PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)
set_target_properties(ik_viz PROPERTIES
    BUILD_RPATH  "${PIN_RPATH}"
    INSTALL_RPATH "${PIN_RPATH}"
)

# ─── 末端步进控制仿真 ─────────────────────────────────────────────────────────
add_executable(traj_sim_stepper test/traj_sim_stepper.cpp)
target_link_libraries(traj_sim_stepper rebot_kinematics)
target_compile_definitions(traj_sim_stepper PRIVATE
    URDF_PATH="${URDF_PATH}"
    PYTHON_BIN="${PYTHON_BIN}"
    PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)
set_target_properties(traj_sim_stepper PROPERTIES
    BUILD_RPATH  "${PIN_RPATH}"
    INSTALL_RPATH "${PIN_RPATH}"
)
add_test(NAME traj_sim_stepper COMMAND traj_sim_stepper)

# ─── 测地线轨迹规划仿真 ──────────────────────────────────────────────────────
add_executable(traj_sim_geodesic test/traj_sim_geodesic.cpp)
target_link_libraries(traj_sim_geodesic rebot_kinematics)
target_compile_definitions(traj_sim_geodesic PRIVATE
    URDF_PATH="${URDF_PATH}"
    PYTHON_BIN="${PYTHON_BIN}"
    PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)
set_target_properties(traj_sim_geodesic PROPERTIES
    BUILD_RPATH  "${PIN_RPATH}"
    INSTALL_RPATH "${PIN_RPATH}"
)
add_test(NAME traj_sim_geodesic COMMAND traj_sim_geodesic)

# ─── 末端画圆仿真（YZ 平面正反画圆循环）────────────────────────────────────────
add_executable(traj_sim_circle test/traj_sim_circle.cpp)
target_link_libraries(traj_sim_circle rebot_kinematics)
target_compile_definitions(traj_sim_circle PRIVATE
    URDF_PATH="${URDF_PATH}"
    PYTHON_BIN="${PYTHON_BIN}"
    PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)
set_target_properties(traj_sim_circle PROPERTIES
    BUILD_RPATH  "${PIN_RPATH}"
    INSTALL_RPATH "${PIN_RPATH}"
)

# ─── 动力学库 ──────────────────────────────────────────────────────────────────
add_library(rebot_dynamics
    src/dynamics/gravity_compensation.cpp
)
target_include_directories(rebot_dynamics PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${PIN_INCLUDES}
)
target_link_libraries(rebot_dynamics PUBLIC rebot_kinematics)
set_target_properties(rebot_dynamics PROPERTIES
    BUILD_RPATH  "${PIN_RPATH}"
    INSTALL_RPATH "${PIN_RPATH}"
)

# ─── 重力补偿实机测试 ──────────────────────────────────────────────────────────
add_executable(gravity_comp_test test/gravity_comp_test.cpp)
target_link_libraries(gravity_comp_test PRIVATE rebot_dynamics rebot_actuator)
target_compile_definitions(gravity_comp_test PRIVATE URDF_PATH="${URDF_PATH}")
set_target_properties(gravity_comp_test PROPERTIES
    BUILD_RPATH  "${PIN_RPATH}"
    INSTALL_RPATH "${PIN_RPATH}"
)

# ─── 实机测地线轨迹控制 ───────────────────────────────────────────────────────
add_executable(arm_traj_ctrl test/arm_traj_ctrl.cpp)
target_link_libraries(arm_traj_ctrl PRIVATE rebot_kinematics rebot_actuator)
target_compile_definitions(arm_traj_ctrl PRIVATE URDF_PATH="${URDF_PATH}")
set_target_properties(arm_traj_ctrl PROPERTIES
    BUILD_RPATH  "${PIN_RPATH}"
    INSTALL_RPATH "${PIN_RPATH}"
)

# ─── 末端直线来回滑动 demo ────────────────────────────────────────────────────
add_executable(arm_line_slide test/arm_line_slide.cpp)
target_link_libraries(arm_line_slide PRIVATE rebot_kinematics rebot_actuator)
target_compile_definitions(arm_line_slide PRIVATE URDF_PATH="${URDF_PATH}")
set_target_properties(arm_line_slide PROPERTIES
    BUILD_RPATH  "${PIN_RPATH}"
    INSTALL_RPATH "${PIN_RPATH}"
)

# ─── 末端定点姿态摆动 demo（高精度）────────────────────────────────────────────
add_executable(arm_orient_swing test/arm_orient_swing.cpp)
target_link_libraries(arm_orient_swing PRIVATE rebot_kinematics rebot_actuator)
target_compile_definitions(arm_orient_swing PRIVATE URDF_PATH="${URDF_PATH}")
set_target_properties(arm_orient_swing PROPERTIES
    BUILD_RPATH  "${PIN_RPATH}"
    INSTALL_RPATH "${PIN_RPATH}"
)

# ─── 实机关节/末端状态监视 ─────────────────────────────────────────────────────
add_executable(arm_state_monitor test/arm_state_monitor.cpp)
target_link_libraries(arm_state_monitor PRIVATE rebot_kinematics rebot_actuator)
target_compile_definitions(arm_state_monitor PRIVATE URDF_PATH="${URDF_PATH}")
set_target_properties(arm_state_monitor PROPERTIES
    BUILD_RPATH  "${PIN_RPATH}"
    INSTALL_RPATH "${PIN_RPATH}"
)

# ─── 末端定点姿态锥形扫掠演示 ──────────────────────────────────────────────────
add_executable(traj_sim_cone test/traj_sim_cone.cpp)
target_link_libraries(traj_sim_cone rebot_kinematics)
target_compile_definitions(traj_sim_cone PRIVATE
    URDF_PATH="${URDF_PATH}"
    PYTHON_BIN="${PYTHON_BIN}"
    PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)
set_target_properties(traj_sim_cone PROPERTIES
    BUILD_RPATH  "${PIN_RPATH}"
    INSTALL_RPATH "${PIN_RPATH}"
)