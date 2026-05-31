# 捶打牛肉丸 — 全部代码详解

> 面向 Python 小白，零基础可读。

---

## 整体流程

```
你执行命令 → launch文件启动程序 → 程序读YAML配置 →
机械臂归零 → 算IK（笛卡尔坐标→关节角度） →
循环: 抬起→下砸→抬起→下砸... → 完成
```

涉及的 4 个关键文件：

| 文件 | 作用 |
|------|------|
| `pound_inverted.yaml` | 你的可调配方表 |
| `pound_inverted.launch.py` | 启动按钮 |
| `demo_common.py` | 工具箱（每个 demo 共用的基础功能） |
| `pound.py` | 锤肉主程序 |

---

## 一、YAML 配置文件

`src/rebotarm_moveit_demos/config/pound_inverted.yaml`

这个文件就是一张"配方表"，你改数字就能调行为。Python 程序运行时会自动读取它。

```yaml
pound_beef:                    # 节点名称
  ros__parameters:             # ROS2 参数标记
    group_name: arm            # 规划组名，固定填 arm
    joint_names:               # 6 个关节的名字
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
      - joint6
    home_joints: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # (1) 归零位置
    use_cartesian: true                            # (2) 笛卡尔模式开关
    strike_xyz: [0.512, 0.0, 0.555]               # (3) 牛肉坐标
    strike_rpy: [0.0, -0.2, 0.0]                  # (4) 下砸时的夹爪朝向
    lift_xyz: [0.240, 0.0, 0.05]                  # (5) 抬起坐标
    lift_rpy: [0.0, 0.8, 0.0]                     # (6) 抬起时的夹爪朝向
    strike_duration: 0.6    # (7) 下砸耗时（秒），越小越快
    lift_duration: 0.8      # (8) 抬起耗时（秒），越小越快
    pause_at_bottom: 0.1    # (9) 砸到底后停多久
    cycle_delay: 0.3        # (10) 两次间休息多久
    pound_count: 5          # (11) 总共砸几下
```

逐项解释：

**(1) `home_joints` — 归零位置**
`[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]` 表示 6 个关节全归零。倒装状态下机械臂垂直下垂，像一根直棍子挂着。

**(2) `use_cartesian` — 模式开关**
- `true`：笛卡尔模式。你填 `strike_xyz`（牛肉位置 x/y/z），程序自动用 Pinocchio IK 算出 6 个关节分别该转多少度。
- `false`：关节模式。你直接填 `lift_joints` / `strike_joints` 6 个关节角度，程序直接照做，不做 IK 计算。

**(3) `strike_xyz` — 牛肉位置**
`[0.512, 0.0, 0.555]` 表示在 `base_link` 坐标系下：
- X=0.512 米（前方 51.2cm）
- Y=0.0 米（正中间）
- Z=0.555 米（下方 55.5cm，倒装时 Z 越大 = 越深）

**改 X/Y 换牛肉位置，改 Z 调深浅。**

**(4) `strike_rpy` — 下砸时夹爪姿态**
`roll, pitch, yaw` 三个角度。打比方：你的手腕转、翻、扭。`[0.0, -0.2, 0.0]` 表示手腕微向前倾。

**(5) `lift_xyz` — 抬起位置**
`[0.240, 0.0, 0.05]` — **关键：XY 必须和下砸一样，只改 Z。**
Z=0.05 比下砸的 0.555 小得多，所以位置更高（倒装时 Z 越小越靠近 base = 越上方）。

**(6) `lift_rpy` — 抬起时夹爪姿态**

**(7~8) `strike_duration` / `lift_duration`**
机械臂走完这个动作给多长时间。0.6 秒下砸 = 比较快，0.8 秒抬起 = 稍慢点保证稳。

**(9~10) 暂停和间隔**
砸到底停 0.1 秒再抬起，每次循环间休息 0.3 秒。

**(11) `pound_count`**
总共砸 5 下。

---

## 二、Launch 文件 — 启动按钮

`src/rebotarm_moveit_demos/launch/pound_inverted.launch.py`

```python
def generate_launch_description():
    # 1. 找到 pound_inverted.yaml 的路径
    config_file = PathJoinSubstitution([
        FindPackageShare("rebotarm_moveit_demos"),
        "config",
        "pound_inverted.yaml",
    ])

    # 2. 启动一个 Node（就是跑 pound.py 里的程序）
    return LaunchDescription([
        Node(
            package="rebotarm_moveit_demos",  # 用哪个包
            executable="pound",               # 跑哪个程序
            name="pound_beef",                # 给它起个名字
            parameters=[config_file],         # 传 YAML 配置进去
        )
    ])
```

你执行 `ros2 launch ... pound_inverted.launch.py` 时，ROS2 就找到 `pound.py` 这个程序，把 `pound_inverted.yaml` 作为参数塞给它，然后运行。

**不需要你写代码，只需改 YAML 里的数字然后 `colcon build`。**

---

## 三、公共工具箱 — demo_common.py

`src/rebotarm_moveit_demos/rebotarm_moveit_demos/demo_common.py`

这是一个"父类"。所有 demo（pound、draw_square、pick_place）都继承它，共享里面的工具函数。

### 3.1 读 YAML 参数

```python
def _param(self, name):
    return self.node.get_parameter(name).value
```

你在 YAML 里写的 `strike_duration: 0.6`，Python 里调用 `self._param("strike_duration")` 就拿到 `0.6`。

### 3.2 获取当前关节角

```python
def current_joint_values(self, fallback_values, fallback_name):
    # 从 /joint_states 话题读取机械臂当前 6 个关节的角度
    # 如果 2 秒内读不到，就用 YAML 里的兜底值
```

### 3.3 构建轨迹

```python
def joint_trajectory(self, start_values, goal_values, duration_sec):
    # 构建一条"轨迹"：从 start_values（当前位置）到 goal_values（目标位置）
    # 要求在 duration_sec 秒内完成
```

打个比方：你告诉司机"从 A 到 B，开 3 秒"。这个函数生成的就是这么条路线。

### 3.4 执行轨迹

```python
def execute_trajectory(self, trajectory, timeout_sec):
    # 把轨迹发给 move_group，让它控制机械臂执行
    # 返回 True（成功）或 False（失败）
```

### 3.5 归零

```python
def go_home(self):
    # 把机械臂移动到 YAML 里定义的 home_joints 位置
    # 如果已经在零点（误差 0.01 rad 内），就跳过
```

### 3.6 订阅关节状态

```python
# demo_common 同时订阅两个话题：
#   /joint_states        — 仿真用
#   /rebotarm/joint_states — 真机用
# 机械臂每 10ms 发一次当前 6 个关节的角度，程序缓存下来随时读取
```

---

## 四、核心 — pound.py 详解

`src/rebotarm_moveit_demos/rebotarm_moveit_demos/pound.py`

### 4.1 初始化 `__init__()`

```python
class PoundBeef(MoveItDemoBase):   # 继承工具箱
    def __init__(self):
        super().__init__("pound_beef")  # 初始化父类

        # 从 YAML 读取参数，存成变量方便用
        self.strike_duration = float(self._param("strike_duration"))   # 0.6
        self.lift_duration   = float(self._param("lift_duration"))     # 0.8
        self.pound_count     = int(self._param("pound_count"))         # 5
        self.use_cartesian   = bool(self._param("use_cartesian"))      # True
```

`__init__` 是 Python 的"构造函数"——程序启动时自动执行一次，把所有配置参数读进来存好。

### 4.2 主流程 `run()`

```python
def run(self):
    # ① 等 MoveIt 就绪
    self.wait_for_execute_server()

    # ② 先把机械臂归零（直杆下垂）
    self.go_home()

    # ③ 读当前关节角
    current = self.current_joint_values(...)

    # ④ 算目标关节角
    if self.use_cartesian:
        lift_target, strike_target = self._setup_cartesian(current)
        # ↑ 笛卡尔模式: 你给的 xyz 坐标 → IK 算成关节角
    else:
        lift_target, strike_target = self._setup_joint_space(current)
        # ↑ 关节模式: 直接用 YAML 里的关节角

    # ⑤ 先移到抬起位置（准备姿势）
    self._move("接近抬起位姿", current, lift_target, 2.0)

    time.sleep(self.cycle_delay)

    # ⑥ 捶打循环
    for cycle in range(self.pound_count):   # 循环 5 次
        # ⑥a: 下砸 — 从 lift_target 到 strike_target
        self._move("下砸", lift_target, strike_target, self.strike_duration)
        time.sleep(self.pause_at_bottom)     # 停 0.1 秒

        # ⑥b: 抬起 — 从 strike_target 回到 lift_target
        self._move("抬起", strike_target, lift_target, self.lift_duration)
        time.sleep(self.cycle_delay)         # 休息 0.3 秒

    # 完成!
```

**用白话描述：**
1. 机械臂先归零（直直垂下）
2. 根据你给的牛肉 xyz 坐标，算出 6 个关节分别转多少度
3. 先慢慢移到抬起位（牛肉上方）
4. 然后：砸下去 → 停一下 → 抬起来 → 歇一下 → 重复 5 遍

### 4.3 笛卡尔坐标 → 关节角度（最难的部分）

```python
def _setup_cartesian(self, current):
    # 导入 Pinocchio（一个专业的运动学计算库）
    sdk_root = Path(__file__).resolve().parents[3] / "third_party" / "reBotArm_control_py"
    sys.path.insert(0, str(sdk_root))
    from reBotArm_control_py.kinematics import load_robot_model
    from reBotArm_control_py.kinematics.inverse_kinematics import solve_ik

    # 加载机械臂的 3D 数学模型
    model = load_robot_model()

    # 从 YAML 读取坐标
    lift_xyz   = [0.240, 0.0, 0.05]      # 抬起位置
    strike_xyz = [0.512, 0.0, 0.555]     # 下砸位置

    # 先解抬起位的关节角（以当前位置为"种子"）
    lift_target = _solve("抬起", lift_xyz, lift_rpy, current)
    # 再解下砸位的关节角（以抬起解为"种子"，这样解更稳定）
    strike_target = _solve("下砸", strike_xyz, strike_rpy, lift_target)

    return lift_target, strike_target
```

**什么是 IK（逆运动学）？**

打个比方：
- 你要用手去抓桌上的水杯（你知道水杯的位置 = xyz 坐标）
- 但你的大脑需要算出：肩膀转多少度、肘弯多少度、手腕扭多少度
- 这个"从目标位置反算关节角度"的过程 = **逆运动学（IK）**

**什么是"种子"（seed）？**
IK 求解时要给一个"起始猜测"。比如你大概知道答案在 0.3 附近，就给它 0.3 当参考点。用上一个解当下一个的种子，能避免"跨越式"求解失败。

### 4.4 关节空间模式（不用 IK）

```python
def _setup_joint_space(self, current):
    # 直接从 YAML 读，不用算
    lift   = [0.0, -0.2, -0.3, 0.0, 0.0, 0.0]
    strike = [0.0, -0.4, -0.5, 0.0, 0.0, 0.0]
    return lift, strike
```

简单粗暴，你直接告诉机械臂每个关节转多少度，跳过 IK 计算。

### 4.5 发送轨迹到机械臂 `_move()`

```python
def _move(self, label, start, goal, duration):
    # 构建轨迹: 起点 → 终点，用时 duration 秒
    traj = self.joint_trajectory(start, goal, duration)
    # 发给 MoveIt，MoveIt 再发给硬件控制器执行
    return self.execute_trajectory(traj, duration + self.result_timeout)
```

---

## 五、牛肉块可视化 — spawn_beef.py

`src/rebotarm_moveit_demos/rebotarm_moveit_demos/spawn_beef.py`

这个程序只在 **RViz 可视化**里显示一个红色长方体，**不影响机械臂实际操作**。

```python
# 构建一个 BOX 形状，尺寸 0.15m宽 × 0.2m长 × 0.05m高
obj = CollisionObject(
    id="beef_block",
    primitives=[SolidPrimitive(type=BOX, dimensions=[0.15, 0.20, 0.05])],
    primitive_poses=[Pose(position=Point(x=0.512, y=0.0, z=0.555))],
)
# 设置红色
req.scene.object_colors = [
    ObjectColor(id="beef_block", color=ColorRGBA(r=1, g=0, b=0, a=1))
]
# 通过 /apply_planning_scene 服务发给 move_group，RViz 实时显示
```

---

## 六、数据流总结

```
                    你的输入
                       │
              pound_inverted.yaml
                 (xyz坐标/速度/次数)
                       │
                       ▼
                  pound.py
                 ┌─────────┐
                 │ 读 YAML │
                 │ 归零     │
                 │ IK 求解  │  ← Pinocchio 把 xyz 变成关节角
                 │ 构建轨迹 │
                 │ 循环捶打 │
                 └────┬────┘
                      │ 发送轨迹 (ExecuteTrajectory action)
                      ▼
                 move_group
                 (MoveIt2 规划器)
                      │
                      ▼
            /rebotarm/follow_joint_trajectory
                 (硬件控制器 action)
                      │
                      ▼
              reBotArmController
              (CAN 总线 → 6 个 DAMIAO 电机)
```

---

## 七、控制命令速查

| 你要做什么 | 改哪里 | 命令 |
|-----------|--------|------|
| 换牛肉位置 | `pound_inverted.yaml` 的 `strike_xyz` | 改完 `colcon build` |
| 调速度 | `strike_duration` / `lift_duration` | 越小越快 |
| 多锤几下 | `pound_count` | 改数字 |
| 切关节模式 | `use_cartesian: false` | 手动设 `lift_joints` / `strike_joints` |
| 换夹爪朝向 | `strike_rpy` / `lift_rpy` | roll/pitch/yaw 三个角 |
| 测试一个坐标能不能到 | — | `python3 solve_ik_pin.py x y z r p y` |
