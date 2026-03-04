#!/usr/bin/env python3
"""
轨迹仿真 MeshCat 可视化查看器
由 traj_sim 通过管道调用。

stdin 协议：
  第 1 行: URDF 路径
  后续行: JSON 对象，两种类型：
    轨迹播放:  {"name":"...", "dt":0.02, "q_list":[[...], ...], "path":[[x,y,z], ...]}
    退出指令:  {"cmd":"exit"}
"""

import sys
import os
import json
import time
import atexit

import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
import meshcat.geometry as mcg
import meshcat.transformations as mct

# ─── 第 1 行：URDF 路径 ───────────────────────────────────────────────────────
urdf = sys.stdin.readline().strip()
if not urdf:
    sys.exit("错误：未收到 URDF 路径")

# 为 URDF 中 package:// 引用创建符号链接（与 ik_viz_viewer.py 保持一致）
project_dir = os.path.dirname(os.path.dirname(os.path.dirname(urdf)))
pkg_link = "/tmp/reBot-DevArm"
if not os.path.islink(pkg_link):
    os.symlink(os.path.join(project_dir, "reBot-DevArm_description"), pkg_link)
    atexit.register(os.unlink, pkg_link)

# ─── 加载机器人模型 ───────────────────────────────────────────────────────────
model = pin.buildModelFromUrdf(urdf)
geom  = pin.buildGeomFromUrdf(model, urdf, pin.GeometryType.VISUAL,
                               package_dirs=["/tmp"])

viz = MeshcatVisualizer(model, geom, geom)
viz.initViewer(open=True)
viz.loadViewerModel()
viz.display(pin.neutral(model))

print(f"MeshCat 地址: {viz.viewer.url()}", flush=True)
print("等待轨迹数据...", flush=True)


def draw_path(points_xyz: list, node_name: str, color: int = 0x00aaff) -> None:
    """在 MeshCat 中绘制三维轨迹线。"""
    if len(points_xyz) < 2:
        return
    pts = np.array(points_xyz, dtype=np.float32).T  # (3, N)
    line = mcg.Line(mcg.PointsGeometry(pts),
                    mcg.LineBasicMaterial(color=color, linewidth=2))
    viz.viewer[node_name].set_object(line)


def play_trajectory(name: str, dt: float, q_list: list, path: list) -> None:
    """播放关节角序列，同时显示轨迹线。"""
    print(f"[viewer] 播放轨迹: {name}  点数={len(q_list)}  dt={dt:.3f}s", flush=True)

    # 先绘制完整轨迹线（参考路径）
    draw_path(path, "traj_path/ref", color=0x888888)

    # 逐帧显示
    visited = []
    for i, q in enumerate(q_list):
        viz.display(np.array(q))
        if path and i < len(path):
            visited.append(path[i])
        draw_path(visited, "traj_path/actual", color=0x00cc44)
        time.sleep(dt)

    print(f"[viewer] 轨迹 '{name}' 播放完毕", flush=True)
    time.sleep(1.0)  # 停留 1 秒后等待下一条


# ─── 主循环：逐行读取 JSON 指令 ───────────────────────────────────────────────
try:
    for raw in sys.stdin:
        raw = raw.strip()
        if not raw:
            continue
        try:
            data = json.loads(raw)
        except json.JSONDecodeError as e:
            print(f"[viewer] JSON 解析错误: {e}", flush=True)
            continue

        # 退出指令
        if data.get("cmd") == "exit":
            print("[viewer] 收到退出指令", flush=True)
            break

        # 轨迹播放
        name    = data.get("name", "unnamed")
        dt      = float(data.get("dt", 0.02))
        q_list  = data.get("q_list", [])
        path    = data.get("path", [])

        if not q_list:
            print(f"[viewer] 警告：轨迹 '{name}' 没有关节角数据", flush=True)
            continue

        play_trajectory(name, dt, q_list, path)

except KeyboardInterrupt:
    pass

# 恢复零位
viz.display(pin.neutral(model))
print("[viewer] 退出", flush=True)
