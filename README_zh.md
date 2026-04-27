# reBotArm ROS2 SDK

<p align="center">
  <strong>ROS2 Jazzy · 机械臂控制 · 夹爪控制 · JointTrajectory · 自定义单电机调试接口</strong>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/ROS2-Jazzy-blue.svg" alt="ROS2 Jazzy">
  <img src="https://img.shields.io/badge/Python-3.12-blue.svg" alt="Python 3.12">
  <img src="https://img.shields.io/badge/Version-v0.0.2-brightgreen.svg" alt="Version v0.0.2">
  <img src="https://img.shields.io/badge/Platform-Ubuntu%2024.04+-orange.svg" alt="Ubuntu 24.04+">
  <img src="https://img.shields.io/badge/Controller-reBotArmController-green.svg" alt="reBotArmController">
</p>

---

## 项目介绍

当前版本：`v0.0.2`

`rebotarm_ros2` 是 reBotArm B601 机械臂的 ROS2 SDK 工作空间。它将现有的
`reBotArm_control_py` Python 控制库封装为 ROS2 topic、service 和 action，
作为二次开发、上层规划、可视化和单电机调试的统一入口。

当前工作空间包含三个 ROS2 包：

| 包 | 作用 |
|---|---|
| `rebotarm_msgs` | 自定义 msg / srv / action 接口 |
| `rebotarmcontroller` | 控制节点包，提供 `reBotArmController` 节点 |
| `rebotarm_bringup` | launch、配置、URDF、RViz 等启动资源 |

---

## 核心功能

- 发布机械臂状态：`/rebotarm/joint_states`、`/rebotarm/arm_status`
- 提供基础服务：`enable`、`disable`、`set_mode`、`set_zero`、`safe_home`
- 支持笛卡尔目标：`MoveToPoseIK` service、`MoveToPose` action
- 支持标准轨迹接口：`control_msgs/action/FollowJointTrajectory`
- 支持夹爪控制：`SetGripper` service、`GripperCommand` action
- 支持 per-joint sparse raw command：`JointMotorCmd`
- 复用 `reBotArm_control_py` 的 `RobotArm`、`ArmEndPos`、FK/IK 和夹爪配置加载

---

## 环境要求

| 项 | 要求 |
|---|---|
| 操作系统 | Ubuntu 24.04+ |
| ROS2 | Jazzy，安装路径 `/opt/ros/jazzy` |
| Python | 系统 Python 3.12 |
| Python 环境 | 不使用 conda |
| 底层 SDK | 推荐 `~/seeed/rebotarm_ros2/third_party/reBotArm_control_py/` |
| 默认串口 | `/dev/ttyACM0`，可通过 launch 参数覆盖 |

每次构建或运行前先 source ROS2：

```bash
source /opt/ros/jazzy/setup.bash
```

---

## 安装依赖

### Step 1. 安装 ROS2 依赖

```bash
sudo apt install -y \
  ros-jazzy-pinocchio \
  ros-jazzy-control-msgs \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-rviz2 \
  ros-jazzy-tf-transformations
```

### Step 2. 安装 motorbridge

`motorbridge` 必须从 PyPI 官方源安装，不要使用清华镜像：

```bash
python3 -m pip install --user --index-url https://pypi.org/simple motorbridge
```

如果 Ubuntu 24.04 报 `externally-managed-environment`，使用：

```bash
python3 -m pip install --user --break-system-packages \
  --index-url https://pypi.org/simple motorbridge
```

### Step 3. 获取底层 SDK

推荐把 `reBotArm_control_py` 放在 workspace 根目录的 `third_party/` 下，而不是
放进 `src/`。`src/` 只放 ROS2 包，第三方非 ROS Python SDK 放在 `third_party/`
更容易管理，也不会被 colcon 当作 ROS 包扫描。

```bash
cd ~/seeed/rebotarm_ros2
mkdir -p third_party
git clone https://github.com/vectorBH6/reBotArm_control_py.git third_party/reBotArm_control_py
```

`reBotArmController` 运行时会自动优先加载：

```text
~/seeed/rebotarm_ros2/third_party/reBotArm_control_py
```

如果该目录不存在，才 fallback 到开发环境中的：

```text
~/seeed/cameraws/sdk/reBotArm_control_py
```

### Step 4. 确认底层 SDK 可导入

如果还没有把 SDK 安装到系统 Python，可临时设置 `PYTHONPATH` 验证：

```bash
source /opt/ros/jazzy/setup.bash
export PYTHONPATH=$PWD/third_party/reBotArm_control_py:$PYTHONPATH
python3 -c "import rclpy, motorbridge, reBotArm_control_py; print('core imports OK')"
python3 -c "import pinocchio; print('pinocchio', pinocchio.__version__)"
python3 -c "from reBotArm_control_py.actuator import RobotArm; from reBotArm_control_py.controllers import ArmEndPos; from reBotArm_control_py.kinematics import compute_fk; from reBotArm_control_py.kinematics.inverse_kinematics import solve_ik; from reBotArm_control_py.actuator.gripper import load_cfg; print('SDK required APIs OK')"
```

---

## 构建工作空间

```bash
cd ~/seeed/rebotarm_ros2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

验证包和入口：

```bash
ros2 pkg executables rebotarmcontroller
```

期望输出：

```text
rebotarmcontroller reBotArmController
```

---

## 快速启动

### 启动完整系统

启动控制节点、`robot_state_publisher`，可选 RViz：

```bash
ros2 launch rebotarm_bringup bringup.launch.py
```

如果机械臂不在默认串口 `/dev/ttyACM0`：

```bash
ros2 launch rebotarm_bringup bringup.launch.py channel:=/dev/ttyACM1
```

启用 RViz：

```bash
ros2 launch rebotarm_bringup bringup.launch.py use_rviz:=true
```

### 只启动控制节点

```bash
ros2 launch rebotarm_bringup driver_only.launch.py
```

### 直接运行控制节点

```bash
ros2 run rebotarmcontroller reBotArmController
```

---

## 目录结构

```text
rebotarm_ros2/
├── README_zh.md
├── PLAN.md
├── instruction.md
└── src/
    ├── rebotarm_msgs/
    │   ├── msg/
    │   ├── srv/
    │   └── action/
    ├── rebotarmcontroller/
    │   ├── rebotarmcontroller/
    │   │   ├── rebotarm_controller.py
    │   │   ├── hardware_manager.py
    │   │   ├── ros_publishers.py
    │   │   ├── ros_services.py
    │   │   ├── ros_actions.py
    │   │   ├── motor_passthrough.py
    │   │   └── conversions.py
    │   └── examples/
    └── rebotarm_bringup/
        ├── launch/
        ├── config/
        ├── description/
        └── rviz/
```

---

## 常用接口

### 状态查看

```bash
ros2 topic echo /rebotarm/arm_status --once
ros2 topic hz /rebotarm/joint_states
```

### 基础服务

```bash
ros2 service call /rebotarm/enable std_srvs/srv/Trigger
ros2 service call /rebotarm/disable std_srvs/srv/Trigger
ros2 service call /rebotarm/safe_home std_srvs/srv/Trigger
ros2 service call /rebotarm/set_mode rebotarm_msgs/srv/SetMode "{mode: 'pos_vel'}"
```

### 移动到笛卡尔位姿

```bash
ros2 action send_goal /rebotarm/move_to_pose rebotarm_msgs/action/MoveToPose \
  "{target_pose: {position: {x: 0.30, y: 0.0, z: 0.30}, orientation: {w: 1.0}}, duration: 2.0}" \
  --feedback
```

### FollowJointTrajectory

```bash
ros2 action send_goal /rebotarm/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: ['joint1','joint2','joint3','joint4','joint5','joint6'],
    points: [{positions: [0,0,0,0,0,0], time_from_start: {sec: 2}},
             {positions: [0.3,0,0,0,0,0], time_from_start: {sec: 4}}]}}"
```

### 夹爪控制

```bash
ros2 service call /rebotarm/gripper/set rebotarm_msgs/srv/SetGripper \
  "{position: 0.05, max_effort: 0.5}"

ros2 action send_goal /rebotarm/gripper/command control_msgs/action/GripperCommand \
  "{command: {position: 0.0, max_effort: 1.0}}"
```

### 单关节 raw passthrough

```bash
ros2 topic pub --once /rebotarm/joints/joint1/cmd rebotarm_msgs/msg/JointMotorCmd \
  "{mode: 0, use_pos: true, use_kp: true, use_kd: true, pos: 0.0, kp: 80.0, kd: 4.0}"
```

---

## 配置说明

`rebotarm_bringup/config/` 提供默认配置：

| 文件 | 说明 |
|---|---|
| `arm.yaml` | 机械臂 6 个关节的电机、反馈 ID、控制参数 |
| `gripper.yaml` | 夹爪电机配置 |
| `driver_params.yaml` | ROS 参数示例 |

常用 launch 参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `arm_config` | bringup 内置 `arm.yaml` | 机械臂配置路径 |
| `gripper_config` | bringup 内置 `gripper.yaml` | 夹爪配置路径 |
| `channel` | 空字符串 | 留空使用 YAML，非空时覆盖串口 |
| `joint_state_rate` | `100.0` | `/rebotarm/joint_states` 发布频率 |
| `cmd_arbitration` | `reject` | 轨迹运行中 per-joint cmd 仲裁，`reject` 或 `preempt` |
| `arm_namespace` | `rebotarm` | ROS 命名空间前缀 |
| `use_rviz` | `false` | 是否启动 RViz |

---

## 示例脚本

示例脚本安装在 `rebotarmcontroller` 包的 share 目录：

```bash
ros2 pkg prefix rebotarmcontroller
```

源文件位于：

```text
src/rebotarmcontroller/examples/demo_move_to_pose.py
src/rebotarmcontroller/examples/demo_joint_passthrough.py
```

---

## 排障

### 找不到串口

如果启动时报：

```text
open serial port /dev/ttyACM0 failed: No such file or directory
```

说明默认串口不存在。先查看实际设备：

```bash
ls /dev/ttyACM*
```

然后用 `channel:=...` 覆盖：

```bash
ros2 launch rebotarm_bringup bringup.launch.py channel:=/dev/ttyACM1
```

### 权限不足

如果串口存在但无权限：

```bash
sudo usermod -a -G dialout $USER
```

重新登录后生效。

### RViz 模型不显示

确认 URDF mesh 路径已经是：

```text
package://rebotarm_bringup/description/meshes/...
```

---

## 已知状态

当前开发环境未接真机时，`reBotArmController` 会在硬件初始化阶段因为
`/dev/ttyACM0` 不存在而退出。接入真机并传入正确 `channel` 后，再运行完整
topic、service 和 action 端到端验证。
