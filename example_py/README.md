# reBot Python 示例

## 前提

1. 已编译 `rebot_py` 模块：
   ```bash
   cd /path/to/reBotArm_control/build
   cmake .. -DPINOCCHIO_ROOT=...
   make rebot_py -j$(nproc)
   ```

2. 使用 pinocchio conda 环境：
   ```bash
   conda activate pinocchio
   ```

3. 编译产物 `rebot_py.cpython-*.so` 位于 `build/` 目录下。

## 示例列表

| 文件 | 说明 | 需要实机 |
|------|------|----------|
| `ik_fk_demo.py` | FK / IK / 轨迹规划 / 重力补偿纯算法演示 | 否 |
| `arm_traj_ctrl.py` | 交互式末端位姿输入 + 测地线轨迹执行 | 是 |
| `arm_state_monitor.py` | 50Hz 只读关节状态监视 | 是 |

## 运行

```bash
# 无需硬件的算法演示
cd example_py
python3 ik_fk_demo.py

# 实机控制（需 sudo 访问串口）
sudo python3 arm_traj_ctrl.py /dev/ttyACM0
sudo python3 arm_state_monitor.py /dev/ttyACM0
```

## API 速览

```python
import rebot_py
import numpy as np
import math

# 位姿创建（返回 4×4 numpy 矩阵）
T = rebot_py.make_pose(x, y, z, roll, pitch, yaw)
T = rebot_py.ArmController.pose(x, y, z, roll, pitch, yaw)

# 模型加载
robot = rebot_py.RobotModel("path/to/urdf")

# 正运动学
T = rebot_py.compute_fk(robot, q)  # q: numpy 1-D array

# 逆运动学
result = rebot_py.solve_ik(robot, target_pose, q_init)
# result.success, result.q, result.error, result.iterations

# 轨迹规划
traj = rebot_py.plan_joint_space_trajectory(robot, q_start, q_end, duration)
# traj[i].time, traj[i].q, traj[i].ik_success

# 重力补偿
comp = rebot_py.GravityCompensator(robot)
tau = comp.compute(q)

# 实机控制器
arm = rebot_py.ArmController()
arm.init("/dev/ttyACM0")
arm.move_to_geodesic(rebot_py.make_pose(0.4, 0, 0.3, math.pi, 0, 0))
arm.shutdown()
```
