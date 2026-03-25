cmake_minimum_required(VERSION 3.10)
project(reBot_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ─── 路径配置 ──────────────────────────────────────────────────────────────────
# robotpkg 默认安装路径
set(OPENROBOTS_ROOT "/opt/openrobots" CACHE PATH "robotpkg 安装根目录")

# Pinocchio 通过 robotpkg 安装，使用 pkg-config 查找
list(APPEND CMAKE_PREFIX_PATH ${OPENROBOTS_ROOT})
set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH TRUE)

find_package(PkgConfig REQUIRED)
pkg_check_modules(PINOCCHIO REQUIRED pinocchio)

find_package(Eigen3 REQUIRED)

# yaml-cpp 手动配置（robotpkg 安装，无 cmake/pkg-config 配置）
find_path(YAML_CPP_INCLUDE_DIR NAMES yaml-cpp/yaml.h PATHS ${OPENROBOTS_ROOT}/include)
find_library(YAML_CPP_LIBRARY NAMES yaml-cpp PATHS ${OPENROBOTS_ROOT}/lib)

set(URDF_PATH "${CMAKE_SOURCE_DIR}/urdf/reBot-DevArm_fixend_description/urdf/reBot-DevArm_fixend.urdf")

set(PIN_INCLUDES
    ${PINOCCHIO_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)
set(PIN_LIBS
    ${PINOCCHIO_LIBRARIES}
)
set(PIN_RPATH ${OPENROBOTS_ROOT}/lib)

# 添加 robotpkg 库路径
link_directories(${OPENROBOTS_ROOT}/lib)

set(PYTHON_BIN "${CMAKE_SOURCE_DIR}/.venv/bin/python")

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
target_include_directories(rebot_actuator PUBLIC ${CMAKE_SOURCE_DIR}/include ${YAML_CPP_INCLUDE_DIR})
target_link_libraries(rebot_actuator PUBLIC ${YAML_CPP_LIBRARY})

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
