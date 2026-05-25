from __future__ import annotations

import sys
import time

import rclpy
from rebotarm_moveit_demos.demo_common import MoveItDemoBase


class PoundBeef(MoveItDemoBase):
    """捶打牛肉丸 — 关节空间循环抬起→下砸→抬起。

    两种模式:
      - 笛卡尔模式 (use_cartesian=true):  读 strike_xyz/lift_xyz → Pinocchio IK 自动解关节角
      - 关节空间 (use_cartesian=false): 直接读 lift_joints/strike_joints
    """

    def __init__(self) -> None:
        super().__init__("pound_beef")

        self.strike_duration = float(self._param("strike_duration"))
        self.lift_duration = float(self._param("lift_duration"))
        self.pound_count = int(self._param("pound_count"))
        self.pause_at_bottom = float(self._param("pause_at_bottom"))
        self.cycle_delay = float(self._param("cycle_delay"))
        self.result_timeout = float(self._param("result_timeout"))
        self.use_cartesian = bool(self._param("use_cartesian"))

    # ------------------------------------------------------------------
    def run(self) -> bool:
        if not self.wait_for_execute_server():
            return False

        # 先归零到直杆下垂姿态
        if not self.go_home():
            return False

        # 读取当前关节角（从 /joint_states，读不到则用配置文件兜底值）
        current = self.current_joint_values(
            self._param("default_joints"), "default_joints"
        )
        self.node.get_logger().info(
            f"当前关节角: {[round(v, 3) for v in current]}"
        )

        # 根据模式选择关节角来源
        if self.use_cartesian:
            # 笛卡尔模式: 读 xyz+rpy → Pinocchio IK 求解
            lift_target, strike_target = self._setup_cartesian(current)
        else:
            # 关节空间模式: 直接读配置文件里的关节角
            lift_target, strike_target = self._setup_joint_space(current)

        if lift_target is None or strike_target is None:
            return False

        # 先移动到抬起位姿
        if not self._move("接近抬起位姿", current, lift_target, 2.0):
            return False

        # 等控制器稳定
        time.sleep(self.cycle_delay)

        self.node.get_logger().info(
            f"开始捶打 {self.pound_count} 次  "
            f"下砸={self.strike_duration}s  抬起={self.lift_duration}s"
        )

        # 捶打循环: 下砸 → 底部暂停 → 抬起 → 循环间隔
        for cycle in range(self.pound_count):
            if not self._move(
                f"下砸 {cycle + 1}",
                lift_target,
                strike_target,
                self.strike_duration,
            ):
                return False

            # 砸到底短暂停留（模拟锤子击中后的停顿）
            time.sleep(self.pause_at_bottom)

            if not self._move(
                f"抬起 {cycle + 1}",
                strike_target,
                lift_target,
                self.lift_duration,
            ):
                return False

            # 两次捶打之间的间隙
            time.sleep(self.cycle_delay)

        self.node.get_logger().info(
            f"完成 — 共 {self.pound_count} 次捶打"
        )
        return True

    # ------------------------------------------------------------------
    def _setup_joint_space(
        self, current: list[float]
    ) -> tuple[list[float] | None, list[float] | None]:
        """关节空间模式: 直接从 YAML 读关节角。"""
        lift = [float(v) for v in self._param("lift_joints")]
        strike = [float(v) for v in self._param("strike_joints")]
        self.node.get_logger().info(
            f"抬起关节角: {[round(v, 3) for v in lift]}\n"
            f"下砸关节角: {[round(v, 3) for v in strike]}"
        )
        return lift, strike

    def _setup_cartesian(
        self, current: list[float]
    ) -> tuple[list[float] | None, list[float] | None]:
        """笛卡尔模式: 用 Pinocchio IK 从 xyz+rpy 解关节角，不用 KDL。"""
        import sys
        from pathlib import Path
        import numpy as np
        import pinocchio as pin

        # 找到 SDK 路径并加入 Python 搜索路径
        sdk_root = Path(__file__).resolve().parents[3] / "third_party" / "reBotArm_control_py"
        if str(sdk_root) not in sys.path:
            sys.path.insert(0, str(sdk_root))

        from reBotArm_control_py.kinematics import load_robot_model, get_end_effector_frame_id
        from reBotArm_control_py.kinematics.inverse_kinematics import solve_ik, IKParams

        # 加载 Pinocchio 模型
        model = load_robot_model()
        data = model.createData()
        ee_id = get_end_effector_frame_id(model)
        ik_params = IKParams(max_iter=500, tolerance=1e-4, damping=1e-3, step_size=0.5)

        def _solve(标签, xyz, rpy, 种子关节角):
            """Pinocchio IK: 位姿 → 关节角。"""
            # 从 RPY + XYZ 构造 SE3 位姿
            目标位姿 = pin.SE3(
                pin.rpy.rpyToMatrix(rpy[0], rpy[1], rpy[2]),
                np.array(xyz, dtype=np.float64),
            )
            # IK 求解
            结果 = solve_ik(model, data, ee_id, 目标位姿, np.array(种子关节角, dtype=np.float64), ik_params)
            if 结果.success:
                q = [float(v) for v in 结果.q]
                self.node.get_logger().info(f"{标签} IK 解: {[round(v, 3) for v in q]}")
                return q
            self.node.get_logger().error(f"{标签} IK 无解")
            return None

        # 从 YAML 读取笛卡尔坐标
        lift_xyz = [float(v) for v in self._param("lift_xyz")]
        lift_rpy = [float(v) for v in self._param("lift_rpy")]
        strike_xyz = [float(v) for v in self._param("strike_xyz")]
        strike_rpy = [float(v) for v in self._param("strike_rpy")]

        # 先解抬起位姿（以当前位置为种子），再解下砸位姿（以抬起解为种子）
        lift_target = _solve("抬起", lift_xyz, lift_rpy, current)
        if lift_target is None:
            return None, None
        strike_target = _solve("下砸", strike_xyz, strike_rpy, lift_target)
        if strike_target is None:
            return None, None
        return lift_target, strike_target

    # ------------------------------------------------------------------
    def _move(
        self,
        label: str,
        start: list[float],
        goal: list[float],
        duration: float,
    ) -> bool:
        """构建两关节点轨迹并发送到 MoveIt 执行。"""
        traj = self.joint_trajectory(start, goal, duration)
        self.node.get_logger().info(
            f"移动 {label} ({duration:.2f}s)"
        )
        return self.execute_trajectory(traj, duration + self.result_timeout)


def main() -> None:
    rclpy.init()
    demo = PoundBeef()
    try:
        ok = demo.run()
    except Exception as exc:
        demo.node.get_logger().error(str(exc))
        ok = False
    finally:
        demo.node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
