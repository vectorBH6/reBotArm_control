#!/usr/bin/env python3
"""
MeshCat 统一渲染进程（example/sim 专用）
仿真侧渲染、轨迹播放、目标坐标系显示均在此完成；C++ 例程只链接 ArmController，
并通过管道写入下列 JSON（无额外工程头文件）。

由 C++ 启动：stdin 第 1 行为 URDF，后续每行一条 JSON。

协议：
  {"cmd":"exit"}  — 结束进程
  {"name","dt","q_list","path"}  — 播放关节轨迹（path 为末端位置折线，可空）
  {"target":{"xyz":[x,y,z],"R":[[...]]},"q":[...]}  — 显示 IK 目标与关节角（无插值）
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

# ─── 第 1 行：URDF ───────────────────────────────────────────────────────────
urdf = sys.stdin.readline().strip()
if not urdf:
    sys.exit("错误：未收到 URDF 路径")

with open(urdf) as _f:
    _content = _f.read()
_pkg_names = re.findall(r"package://([^/]+)/", _content)
_pkg_name = _pkg_names[0] if _pkg_names else "reBot-DevArm_description_fixend"
_pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(urdf)))
_pkg_link = f"/tmp/{_pkg_name}"
if os.path.islink(_pkg_link):
    os.unlink(_pkg_link)
os.symlink(_pkg_dir, _pkg_link)
atexit.register(lambda: os.path.islink(_pkg_link) and os.unlink(_pkg_link))

model = pin.buildModelFromUrdf(urdf)
geom = pin.buildGeomFromUrdf(model, urdf, pin.GeometryType.VISUAL, package_dirs=["/tmp"])

viz = MeshcatVisualizer(model, geom, geom)
viz.initViewer(open=True)
viz.loadViewerModel()
viz.display(pin.neutral(model))

print(f"MeshCat 地址: {viz.viewer.url()}", flush=True)
print("等待 JSON 指令 (轨迹 / IK)...", flush=True)


def draw_path(points_xyz: list, node_name: str, color: int = 0x00aaff) -> None:
    if len(points_xyz) < 2:
        return
    pts = np.array(points_xyz, dtype=np.float32).T
    line = mcg.Line(mcg.PointsGeometry(pts), mcg.LineBasicMaterial(color=color, linewidth=2))
    viz.viewer[node_name].set_object(line)


def play_trajectory(name: str, dt: float, q_list: list, path: list) -> None:
    print(f"[viewer] 播放轨迹: {name}  点数={len(q_list)}  dt={dt:.3f}s", flush=True)
    draw_path(path, "traj_path/ref", color=0x888888)
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
    q = np.array(data["q"])
    tgt = data["target"]
    H = np.eye(4)
    H[:3, :3] = np.array(tgt["R"])
    H[:3, 3] = tgt["xyz"]
    viz.viewer["target/frame"].set_object(mcg.triad())
    viz.viewer["target/frame"].set_transform(H)
    viz.viewer["target/ball"].set_object(
        mcg.Sphere(0.015), mcg.MeshLambertMaterial(color=0xff3300))
    viz.viewer["target/ball"].set_transform(H)
    viz.display(q)
    print("[viewer] IK 显示已更新", flush=True)


try:
    for raw in sys.stdin:
        raw = raw.strip()
        if not raw:
            continue
        try:
            data = json.loads(raw)
        except json.JSONDecodeError as e:
            print(f"[viewer] JSON 错误: {e}", flush=True)
            continue

        if data.get("cmd") == "exit":
            print("[viewer] exit", flush=True)
            break

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

        if "target" in data and "q" in data:
            show_ik_pose(data)
            continue

        print("[viewer] 未识别的 JSON，跳过", flush=True)

except KeyboardInterrupt:
    pass

viz.display(pin.neutral(model))
print("[viewer] 退出", flush=True)
