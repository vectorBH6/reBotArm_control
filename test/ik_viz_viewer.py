#!/usr/bin/env python3
"""
IK 交互可视化渲染桥
由 ik_viz 通过管道调用，stdin 协议：
  第 1 行: URDF 路径
  后续行: JSON {"target":{"xyz":[x,y,z],"R":[[...]]}, "q":[...]}
"""
import sys, os, json, atexit
import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
import meshcat.geometry as mcg

# --- 第 1 行：URDF 路径 ---
urdf = sys.stdin.readline().strip()
if not urdf:
    sys.exit("错误：未收到 URDF 路径")

project_dir = os.path.dirname(os.path.dirname(os.path.dirname(urdf)))

# 符号链接（程序退出时自动清理）
pkg_link = "/tmp/reBot-DevArm"
if not os.path.islink(pkg_link):
    os.symlink(os.path.join(project_dir, "reBot-DevArm_description"), pkg_link)
    atexit.register(os.unlink, pkg_link)

# 加载模型
model = pin.buildModelFromUrdf(urdf)
geom  = pin.buildGeomFromUrdf(model, urdf, pin.GeometryType.VISUAL,
                               package_dirs=["/tmp"])

# 初始化可视化器
viz = MeshcatVisualizer(model, geom, geom)
viz.initViewer(open=True)
viz.loadViewerModel()

# 先显示零位
viz.display(pin.neutral(model))
print(f"MeshCat 地址: {viz.viewer.url()}", flush=True)
print("等待位姿指令...", flush=True)

# --- 主循环：逐行读取 JSON 指令 ---
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

        q   = np.array(data["q"])
        tgt = data["target"]

        # 更新目标标记（红球 + 坐标轴）
        H = np.eye(4)
        H[:3, :3] = np.array(tgt["R"])
        H[:3,  3] = tgt["xyz"]
        viz.viewer["target/frame"].set_object(mcg.triad())
        viz.viewer["target/frame"].set_transform(H)
        viz.viewer["target/ball"].set_object(
            mcg.Sphere(0.015), mcg.MeshLambertMaterial(color=0xff3300))
        viz.viewer["target/ball"].set_transform(H)

        # 直接跳转到 IK 解（无插值）
        viz.display(q)
        print("[viewer] 已更新显示", flush=True)

except KeyboardInterrupt:
    pass

print("[viewer] 退出", flush=True)
