cmake_minimum_required(VERSION 3.10)
project(reBot_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ─── 路径配置 ──────────────────────────────────────────────────────────────────
if(NOT PINOCCHIO_ROOT)
  if(DEFINED ENV{PINOCCHIO_ROOT} AND NOT "$ENV{PINOCCHIO_ROOT}" STREQUAL "")
    set(PINOCCHIO_ROOT "$ENV{PINOCCHIO_ROOT}")
  endif()
endif()
set(PINOCCHIO_ROOT "${PINOCCHIO_ROOT}" CACHE PATH "Pinocchio 根目录（含 include 与 build）")
if(NOT PINOCCHIO_ROOT)
  message(FATAL_ERROR "未设置 PINOCCHIO_ROOT：请 cmake -DPINOCCHIO_ROOT=... 或 export PINOCCHIO_ROOT")
endif()
set(PINOCCHIO_BUILD ${PINOCCHIO_ROOT}/build)
set(URDF_PATH       "${CMAKE_SOURCE_DIR}/urdf/reBot-DevArm_fixend_description/urdf/reBot-DevArm_fixend.urdf")

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

set(PYTHON_BIN "/home/rvector/anaconda3/envs/pinocchio/bin/python3")

# ─── 仿真 example 辅助函数（只链 rebot_kinematics，不链 rebot_application）────
function(add_sim_viz_exe _name)
  add_executable(${_name} example/sim/${_name}.cpp)
  target_link_libraries(${_name} PRIVATE rebot_kinematics)
  target_include_directories(${_name} PRIVATE
    ${CMAKE_SOURCE_DIR}/example/sim
    ${CMAKE_SOURCE_DIR}/include
    ${PIN_INCLUDES}
  )
  target_compile_definitions(${_name} PRIVATE
    URDF_PATH="${URDF_PATH}"
    PYTHON_BIN="${PYTHON_BIN}"
    PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
  )
  set_target_properties(${_name} PROPERTIES
    BUILD_RPATH "${PIN_RPATH}"
    INSTALL_RPATH "${PIN_RPATH}"
  )
endfunction()

# ─── 运动学库 ──────────────────────────────────────────────────────────────────
add_library(rebot_kinematics
    src/kinematics/robot_model.cpp
    src/kinematics/forward_kinematics.cpp
    src/kinematics/inverse_kinematics.cpp
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

# ─── 执行器抽象层 ──────────────────────────────────────────────────────────────
add_library(rebot_actuator STATIC
    src/actuator/arm_actuator_group.cpp
)
target_include_directories(rebot_actuator PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(rebot_actuator PUBLIC yaml-cpp)

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

# ─── 应用层库：实机控制器 ──────────────────────────────────────────────────────
add_library(rebot_application STATIC
    src/application/arm_controller.cpp
)
target_include_directories(rebot_application PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(rebot_application PUBLIC rebot_kinematics rebot_actuator)
target_compile_definitions(rebot_application PUBLIC URDF_PATH="${URDF_PATH}")

# ─── example/real：实机应用 ───────────────────────────────────────────────────
add_executable(arm_traj_ctrl example/real/arm_traj_ctrl.cpp)
target_link_libraries(arm_traj_ctrl PRIVATE rebot_application)
target_compile_definitions(arm_traj_ctrl PRIVATE URDF_PATH="${URDF_PATH}")
set_target_properties(arm_traj_ctrl PROPERTIES
    BUILD_RPATH  "${PIN_RPATH}"
    INSTALL_RPATH "${PIN_RPATH}"
)

add_executable(arm_traj_triangle example/real/arm_traj_triangle.cpp)
target_link_libraries(arm_traj_triangle PRIVATE rebot_application)
target_compile_definitions(arm_traj_triangle PRIVATE URDF_PATH="${URDF_PATH}")
set_target_properties(arm_traj_triangle PROPERTIES
    BUILD_RPATH  "${PIN_RPATH}"
    INSTALL_RPATH "${PIN_RPATH}"
)

add_executable(arm_state_monitor example/real/arm_state_monitor.cpp)
target_link_libraries(arm_state_monitor PRIVATE rebot_application)
target_compile_definitions(arm_state_monitor PRIVATE URDF_PATH="${URDF_PATH}")
set_target_properties(arm_state_monitor PROPERTIES
    BUILD_RPATH  "${PIN_RPATH}"
    INSTALL_RPATH "${PIN_RPATH}"
)

# ─── example/real：调试工具（不经 ArmController）──────────────────────────────
add_executable(arm_web_ctrl example/real/arm_web_ctrl.cpp)
target_link_libraries(arm_web_ctrl PRIVATE rebot_actuator)

add_executable(gravity_comp_single_joint_test example/real/gravity_comp_single_joint_test.cpp)
target_link_libraries(gravity_comp_single_joint_test PRIVATE rebot_dynamics rebot_actuator)
target_compile_definitions(gravity_comp_single_joint_test PRIVATE URDF_PATH="${URDF_PATH}")
set_target_properties(gravity_comp_single_joint_test PROPERTIES
    BUILD_RPATH  "${PIN_RPATH}"
    INSTALL_RPATH "${PIN_RPATH}"
)

# ─── example/sim：仿真 / 可视化（只链 rebot_kinematics）──────────────────────
add_sim_viz_exe(ik_viz)
add_sim_viz_exe(traj_sim_geodesic)
add_test(NAME traj_sim_geodesic COMMAND traj_sim_geodesic)
add_sim_viz_exe(traj_sim_circle)
add_sim_viz_exe(traj_sim_cone)
