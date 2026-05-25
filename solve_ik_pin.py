#!/usr/bin/env python3
"""Pinocchio 逆运动学求解器: 输入 Cartesian 坐标 + RPY 姿态角，输出关节角度。

用法:
    python3 solve_ik_pin.py x y z roll pitch yaw

示例:
    python3 solve_ik_pin.py 0.512 0 0.555 0 -0.2 0       # 牛肉在正前方
    python3 solve_ik_pin.py 0.512 0.1 0.555 0 -0.2 0     # 牛肉往右偏 10cm
"""
import sys
from pathlib import Path
import numpy as np
import pinocchio as pin

# 把 SDK 加入 Python 搜索路径
sdk_root = Path(__file__).resolve().parent / "src" / "reBotArmController_ROS2" / "third_party" / "reBotArm_control_py"
sys.path.insert(0, str(sdk_root))

from reBotArm_control_py.kinematics import load_robot_model, get_end_effector_frame_id
from reBotArm_control_py.kinematics.inverse_kinematics import solve_ik, IKParams

# 加载 Pinocchio 模型
model = load_robot_model()
data = model.createData()
ee_id = get_end_effector_frame_id(model)

# 解析命令行参数
if len(sys.argv) >= 4:
    x, y, z = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
    roll = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
    pitch = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0
    yaw = float(sys.argv[6]) if len(sys.argv) > 6 else 0.0
else:
    # 默认: 牛肉在正前方 51cm、下方 55cm
    x, y, z = 0.512, 0.0, 0.555
    roll, pitch, yaw = 0.0, -0.2, 0.0

# RPY → 旋转矩阵 → SE3 位姿
target = pin.SE3(pin.rpy.rpyToMatrix(roll, pitch, yaw), np.array([x, y, z]))

# 从零点开始搜索
q_seed = pin.neutral(model)

# IK 参数: 最大 500 次迭代，容差 1e-4，阻尼 1e-3
params = IKParams(max_iter=500, tolerance=1e-4, damping=1e-3, step_size=0.5)
result = solve_ik(model, data, ee_id, target, q_seed, params)

if result.success:
    q = np.degrees(result.q)
    print(f"✓ IK 成功")
    print(f"关节角 (rad): {[round(v, 3) for v in result.q]}")
    print(f"关节角 (deg): {[round(v, 1) for v in q]}")
    print(f"\n贴到 YAML 配置文件:")
    print(f"  strike_joints: {[round(v, 3) for v in result.q]}")
else:
    print("✗ IK 无解 — 该位姿不在机械臂工作空间内")
