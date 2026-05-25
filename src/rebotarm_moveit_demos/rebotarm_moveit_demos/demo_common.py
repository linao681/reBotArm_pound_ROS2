from __future__ import annotations

import time

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import (
    CollisionObject,
    MoveItErrorCodes,
    PlanningSceneWorld,
    RobotState,
    RobotTrajectory,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPositionIK
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class MoveItDemoBase:
    """所有 Demo 的基类 — 封装 MoveIt 通信、轨迹执行、归零、场景重置。"""

    def __init__(self, node_name: str) -> None:
        # 创建 ROS2 节点，自动从 YAML 配置文件声明参数
        self.node = rclpy.create_node(
            node_name,
            automatically_declare_parameters_from_overrides=True,
        )
        self.group_name = str(self._param("group_name"))
        self.joint_names = [str(name) for name in self._param("joint_names")]
        self._latest_joint_positions: dict[str, float] = {}

        # MoveIt 的轨迹执行 action 客户端
        self._execute = ActionClient(self.node, ExecuteTrajectory, "/execute_trajectory")
        # MoveIt 的 IK 服务客户端（当前 PoundBeef 笛卡尔模式不用这个，用 Pinocchio）
        self._ik = self.node.create_client(GetPositionIK, "/compute_ik")
        # 规划场景服务（用于清除碰撞物体）
        self._scene_svc = self.node.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )
        # 碰撞物体发布器（用于 REMOVE 操作）
        self._collision_pub = self.node.create_publisher(
            CollisionObject, "/collision_object", 10
        )
        # 订阅关节状态（仿真和真机两个话题都订）
        self.node.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_cb,
            qos_profile_sensor_data,
        )
        self.node.create_subscription(
            JointState,
            "/rebotarm/joint_states",
            self._joint_state_cb,
            qos_profile_sensor_data,
        )

    def _param(self, name: str):
        """读 ROS2 参数（来自 YAML 配置文件）。"""
        return self.node.get_parameter(name).value

    @staticmethod
    def duration(seconds: float) -> Duration:
        """秒数 → ROS2 Duration 消息。"""
        return Duration(sec=int(seconds), nanosec=int((seconds % 1.0) * 1_000_000_000))

    def wait_for_execute_server(self) -> bool:
        """等待 MoveIt 的 /execute_trajectory action 服务就绪。"""
        if self._execute.wait_for_server(timeout_sec=10.0):
            return True
        self.node.get_logger().error("MoveIt /execute_trajectory 不可用")
        return False

    def wait_for_ik_service(self) -> bool:
        """等待 MoveIt 的 /compute_ik 服务就绪（仅 KDL 模式使用）。"""
        if self._ik.wait_for_service(timeout_sec=10.0):
            return True
        self.node.get_logger().error("MoveIt /compute_ik 不可用")
        return False

    def wait(self, future, timeout_sec: float) -> bool:
        """自旋等待异步 future 完成，超时返回 False。"""
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return False
            rclpy.spin_once(self.node, timeout_sec=min(0.1, remaining))
        return future.done()

    def _joint_state(self, joint_values: list[float], is_diff: bool = False) -> RobotState:
        """构建 MoveIt RobotState（is_diff=False 表示绝对关节角）。"""
        return RobotState(
            is_diff=is_diff,
            joint_state=JointState(name=list(self.joint_names), position=list(joint_values)),
        )

    def _joint_state_cb(self, message: JointState) -> None:
        """收到 /joint_states 时更新缓存。"""
        for name, position in zip(message.name, message.position):
            self._latest_joint_positions[name] = float(position)

    def current_joint_values(
        self,
        fallback_values: list[float],
        fallback_name: str,
        log_current: bool = False,
    ) -> list[float]:
        """读取当前关节角。读不到则用兜底值（YAML 里的 default_joints 等）。"""
        deadline = time.monotonic() + 2.0
        while rclpy.ok() and not all(
            name in self._latest_joint_positions for name in self.joint_names
        ):
            if time.monotonic() >= deadline:
                self.node.get_logger().warn(
                    "等待 /joint_states 超时，使用配置的 "
                    f"{fallback_name}"
                )
                return fallback_values
            rclpy.spin_once(self.node, timeout_sec=0.1)

        current = [self._latest_joint_positions[name] for name in self.joint_names]
        if log_current:
            self.node.get_logger().info(
                f"当前关节状态: {[round(value, 4) for value in current]}"
            )
        return current

    def joint_trajectory(
        self,
        start_values: list[float],
        goal_values: list[float],
        duration_sec: float,
    ) -> RobotTrajectory:
        """构建两关节点轨迹: 起始位姿 → 目标位姿，时长 duration_sec 秒。"""
        return RobotTrajectory(
            joint_trajectory=JointTrajectory(
                joint_names=list(self.joint_names),
                points=[
                    JointTrajectoryPoint(
                        positions=list(start_values),
                        time_from_start=self.duration(0.0),
                    ),
                    JointTrajectoryPoint(
                        positions=list(goal_values),
                        time_from_start=self.duration(duration_sec),
                    ),
                ],
            )
        )

    def joint_trajectory_points(
        self,
        joint_values: list[list[float]],
        duration_sec: float,
    ) -> RobotTrajectory:
        """构建多关节点轨迹: 多个途经点均匀分布在时长内。"""
        step_duration = duration_sec / max(len(joint_values) - 1, 1)
        return RobotTrajectory(
            joint_trajectory=JointTrajectory(
                joint_names=list(self.joint_names),
                points=[
                    JointTrajectoryPoint(
                        positions=list(values),
                        time_from_start=self.duration(index * step_duration),
                    )
                    for index, values in enumerate(joint_values)
                ],
            )
        )

    def compute_ik_joint_target(
        self,
        pose_stamped: PoseStamped,
        seed_values: list[float],
        ik_link_name: str,
        timeout_sec: float,
        avoid_collisions: bool,
        label: str,
        warn_only: bool = False,
    ) -> list[float] | None:
        """通过 MoveIt /compute_ik（KDL 求解器）算 IK。PoundBeef 不用此方法。"""
        request = GetPositionIK.Request()
        request.ik_request.group_name = self.group_name
        request.ik_request.robot_state = self._joint_state(seed_values, is_diff=False)
        request.ik_request.avoid_collisions = avoid_collisions
        request.ik_request.ik_link_name = ik_link_name
        request.ik_request.pose_stamped = pose_stamped
        request.ik_request.timeout = self.duration(timeout_sec)

        future = self._ik.call_async(request)
        log = self.node.get_logger().warn if warn_only else self.node.get_logger().error
        if not self.wait(future, timeout_sec):
            log(f"计算 {label} IK 超时")
            return None

        response = future.result()
        if response is None or response.error_code.val != MoveItErrorCodes.SUCCESS:
            code = response.error_code.val if response is not None else "empty"
            log(f"计算 {label} IK 失败: {code}")
            return None

        joint_map = dict(
            zip(response.solution.joint_state.name, response.solution.joint_state.position)
        )
        return [float(joint_map[name]) for name in self.joint_names]

    def execute_trajectory(self, trajectory: RobotTrajectory, timeout_sec: float) -> bool:
        """发送轨迹到 MoveIt /execute_trajectory 并等待执行完成。"""
        send_future = self._execute.send_goal_async(
            ExecuteTrajectory.Goal(trajectory=trajectory)
        )
        if not self.wait(send_future, 5.0):
            self.node.get_logger().error("发送 ExecuteTrajectory 目标超时")
            return False

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().error("ExecuteTrajectory 目标被拒")
            return False

        result_future = goal_handle.get_result_async()
        if not self.wait(result_future, timeout_sec):
            self.node.get_logger().error(
                f"ExecuteTrajectory 在 {timeout_sec:.1f}s 内未返回"
            )
            return False

        action_result = result_future.result()
        if action_result is None:
            self.node.get_logger().error("ExecuteTrajectory 返回空结果")
            return False

        result = action_result.result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.node.get_logger().error(
                f"ExecuteTrajectory 失败，错误码 {result.error_code.val}"
            )
            return False
        return True

    def reset_scene(self) -> None:
        """清除规划场景中的所有碰撞物体（防止上轮 Demo 残留物体干扰）。"""
        if not self._scene_svc.wait_for_service(timeout_sec=3.0):
            self.node.get_logger().warn("/apply_planning_scene 不可用")
            return

        req = ApplyPlanningScene.Request()
        req.scene.is_diff = True
        req.scene.world = PlanningSceneWorld(collision_objects=[])
        self._scene_svc.call_async(req)

        self.node.get_logger().info("场景已重置")

    def go_home(self) -> bool:
        """机械臂归零 — 先清场景，再执行到 home_joints 的轨迹。"""
        self.reset_scene()
        home = [float(v) for v in self._param("home_joints")]
        current = self.current_joint_values(home, "home_joints")
        if all(abs(c - h) < 0.01 for c, h in zip(current, home)):
            self.node.get_logger().info("已在零点，跳过归零")
            return True
        self.node.get_logger().info(
            f"归零 {[round(v,3) for v in current]} → {[round(v,3) for v in home]}"
        )
        return self.execute_trajectory(
            self.joint_trajectory(current, home, 2.0), 10.0
        )
