#!/usr/bin/env python3
"""
MeshCat 可视化渲染进程（example/sim 专用）

功能：
    - 接收 C++ 仿真程序通过管道发送的 JSON 指令
    - 使用 Pinocchio + MeshCat 渲染机械臂 3D 模型
    - 支持轨迹播放和 IK 目标可视化

通信协议：
    stdin 第 1 行：URDF 文件路径（用于加载机器人模型）
    后续每行：一条 JSON 指令
    
    支持的 JSON 指令：
    
    1. 退出指令
       {"cmd": "exit"}
       
    2. 轨迹播放指令
       {
         "name": "轨迹名称",
         "dt": 时间步长（秒）,
         "q_list": [[q1_0, q2_0, ...], [q1_1, q2_1, ...], ...],
         "path": [[x0,y0,z0], [x1,y1,z1], ...]  // 可选，末端参考路径
       }
       
    3. IK 可视化指令
       {
         "target": {"xyz": [x,y,z], "R": [[...], [...], [...]]},
         "q": [q1, q2, ...]
       }

使用方式：
    由 C++ 程序通过 viz_bridge.h 的 spawn() 函数自动启动
    不建议手动运行
"""
import sys
import os
import json
import time
import atexit
import re

import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
import meshcat.geometry as mcg

# ─────────────────────────────────────────────────────────────────────────────
# 初始化：读取 URDF 并创建 MeshCat 可视化器
# ─────────────────────────────────────────────────────────────────────────────

# 从 stdin 读取 URDF 路径（协议要求：第一行）
urdf = sys.stdin.readline().strip()
if not urdf:
    sys.exit("错误：未收到 URDF 路径")

# 处理 package:// 路径（创建临时符号链接以便 Pinocchio 加载 mesh）
with open(urdf) as _f:
    _content = _f.read()
_pkg_names = re.findall(r"package://([^/]+)/", _content)
_pkg_name = _pkg_names[0] if _pkg_names else "reBot-DevArm_description_fixend"
_pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(urdf)))
_pkg_link = f"/tmp/{_pkg_name}"

# 清理旧的符号链接并创建新链接
if os.path.islink(_pkg_link):
    os.unlink(_pkg_link)
os.symlink(_pkg_dir, _pkg_link)
atexit.register(lambda: os.path.islink(_pkg_link) and os.unlink(_pkg_link))

# 加载 Pinocchio 模型和可视化几何
model = pin.buildModelFromUrdf(urdf)
geom = pin.buildGeomFromUrdf(model, urdf, pin.GeometryType.VISUAL, package_dirs=["/tmp"])

# 创建 MeshCat 可视化器并在浏览器中打开
viz = MeshcatVisualizer(model, geom, geom)
viz.initViewer(open=True)
viz.loadViewerModel()
viz.display(pin.neutral(model))

print(f"MeshCat 地址: {viz.viewer.url()}", flush=True)
print("等待 JSON 指令 (轨迹 / IK)...", flush=True)


# ─────────────────────────────────────────────────────────────────────────────
# 渲染函数
# ─────────────────────────────────────────────────────────────────────────────

def draw_path(points_xyz: list, node_name: str, color: int = 0x00aaff) -> None:
    """
    在场景中绘制 3D 折线路径
    
    Args:
        points_xyz: 三维点列表 [[x,y,z], ...]
        node_name: MeshCat 节点名称（用于更新或删除）
        color: RGB 十六进制颜色值（默认浅蓝色）
    """
    if len(points_xyz) < 2:
        return
    pts = np.array(points_xyz, dtype=np.float32).T
    line = mcg.Line(mcg.PointsGeometry(pts), mcg.LineBasicMaterial(color=color, linewidth=2))
    viz.viewer[node_name].set_object(line)


def play_trajectory(name: str, dt: float, q_list: list, path: list) -> None:
    """
    播放关节轨迹动画
    
    工作流程：
        1. 绘制参考路径（灰色）
        2. 逐帧显示机械臂姿态（按 dt 间隔）
        3. 同步绘制已走路径（绿色）
    
    Args:
        name: 轨迹名称（用于日志输出）
        dt: 帧间时间间隔 [秒]
        q_list: 关节角序列 [[q1,...,qn], ...]
        path: 末端位置序列 [[x,y,z], ...]（可为空）
    """
def play_trajectory(name: str, dt: float, q_list: list, path: list) -> None:
    """
    播放关节轨迹动画
    
    工作流程：
        1. 绘制参考路径（灰色）
        2. 逐帧显示机械臂姿态（按 dt 间隔）
        3. 同步绘制已走路径（绿色）
    
    Args:
        name: 轨迹名称（用于日志输出）
        dt: 帧间时间间隔 [秒]
        q_list: 关节角序列 [[q1,...,qn], ...]
        path: 末端位置序列 [[x,y,z], ...]（可为空）
    """
    print(f"[viewer] 播放轨迹: {name}  点数={len(q_list)}  dt={dt:.3f}s", flush=True)
    
    # 绘制完整参考路径（灰色）
    draw_path(path, "traj_path/ref", color=0x888888)
    
    # 逐帧播放并累积已走路径
    visited = []
    for i, q in enumerate(q_list):
        viz.display(np.array(q))
        if path and i < len(path):
            visited.append(path[i])
        draw_path(visited, "traj_path/actual", color=0x00cc44)
        time.sleep(dt)
    
    print(f"[viewer] 轨迹 '{name}' 完毕", flush=True)
    time.sleep(1.0)


def show_ik_pose(data: dict) -> None:
    """
    显示 IK 求解结果（目标位姿 + 对应关节角）
    
    可视化内容：
        - 目标位姿：三色坐标轴（RGB = XYZ）+ 红色球体标记
        - 机械臂：更新到求解出的关节角配置
    
    Args:
        data: 包含 "target" 和 "q" 字段的字典
              target: {"xyz": [x,y,z], "R": [[r11,r12,r13], [r21,r22,r23], [r31,r32,r33]]}
              q: [q1, q2, ..., qn]
    """
def show_ik_pose(data: dict) -> None:
    """
    显示 IK 求解结果（目标位姿 + 对应关节角）
    
    可视化内容：
        - 目标位姿：三色坐标轴（RGB = XYZ）+ 红色球体标记
        - 机械臂：更新到求解出的关节角配置
    
    Args:
        data: 包含 "target" 和 "q" 字段的字典
              target: {"xyz": [x,y,z], "R": [[r11,r12,r13], [r21,r22,r23], [r31,r32,r33]]}
              q: [q1, q2, ..., qn]
    """
    q = np.array(data["q"])
    tgt = data["target"]
    
    # 构建 4x4 齐次变换矩阵
    H = np.eye(4)
    H[:3, :3] = np.array(tgt["R"])
    H[:3, 3] = tgt["xyz"]
    
    # 显示目标坐标系（三色轴）
    viz.viewer["target/frame"].set_object(mcg.triad())
    viz.viewer["target/frame"].set_transform(H)
    
    # 显示目标位置标记（红色小球）
    viz.viewer["target/ball"].set_object(
        mcg.Sphere(0.015), mcg.MeshLambertMaterial(color=0xff3300))
    viz.viewer["target/ball"].set_transform(H)
    
    # 更新机械臂姿态
    viz.display(q)
    print("[viewer] IK 显示已更新", flush=True)


# ─────────────────────────────────────────────────────────────────────────────
# 主循环：解析并执行 JSON 指令
# ─────────────────────────────────────────────────────────────────────────────

try:
    for raw in sys.stdin:
        raw = raw.strip()
        if not raw:
            continue
        
        # 解析 JSON
        try:
            data = json.loads(raw)
        except json.JSONDecodeError as e:
            print(f"[viewer] JSON 解析错误: {e}", flush=True)
            continue

        # 处理退出指令
        if data.get("cmd") == "exit":
            print("[viewer] 收到退出指令", flush=True)
            break

        # 处理轨迹播放指令
        if "q_list" in data:
            name = data.get("name", "unnamed")
            dt = float(data.get("dt", 0.02))
            q_list = data.get("q_list", [])
            path = data.get("path", [])
            if not q_list:
                print(f"[viewer] 警告: 轨迹 '{name}' 无关节数据", flush=True)
                continue
            play_trajectory(name, dt, q_list, path)
            continue

        # 处理 IK 可视化指令
        if "target" in data and "q" in data:
            show_ik_pose(data)
            continue

        print("[viewer] 未识别的 JSON 格式，跳过", flush=True)

except KeyboardInterrupt:
    pass

# 退出前恢复到中位配置
viz.display(pin.neutral(model))
print("[viewer] 退出", flush=True)
