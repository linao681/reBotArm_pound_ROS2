#!/usr/bin/env python3
"""在仿真场景中加载一块红色长方体肉泥，放在倒装机械臂面前下方。

肉泥尺寸: 宽0.15m(X) × 长0.2m(Y) × 高0.05m(Z)
位置: Y 范围 -0.1~0.1，默认中心 X=0.512, Y=0, 顶面 Z=0.555

用法:
    source ~/ros2_ws/install/setup.bash
    ros2 run rebotarm_moveit_demos spawn_beef
"""

import rclpy
from geometry_msgs.msg import Pose, Point, Quaternion
from moveit_msgs.msg import CollisionObject, ObjectColor
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA, Header


def main():
    rclpy.init()
    node = rclpy.create_node("spawn_beef", allow_undeclared_parameters=True,
                             automatically_declare_parameters_from_overrides=True)

    # 肉泥尺寸: X=宽, Y=长, Z=高
    strike_z = float(node.get_parameter("z").value if node.has_parameter("z") else 0.555)
    cx = float(node.get_parameter("x").value if node.has_parameter("x") else 0.512)
    cy = float(node.get_parameter("y").value if node.has_parameter("y") else 0.0)
    wx = float(node.get_parameter("wx").value if node.has_parameter("wx") else 0.15)
    wy = float(node.get_parameter("wy").value if node.has_parameter("wy") else 0.20)
    hz = float(node.get_parameter("hz").value if node.has_parameter("hz") else 0.05)

    # box 中心 = 顶面下方半个高度
    cz = strike_z + hz * 0.5

    # 等待 /apply_planning_scene 服务就绪
    scene_cli = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    if not scene_cli.wait_for_service(timeout_sec=5.0):
        node.get_logger().error("/apply_planning_scene 服务不可用，请先启动仿真")
        rclpy.shutdown()
        return

    # 构建红色碰撞物体
    obj = CollisionObject()
    obj.header = Header(frame_id="base_link")
    obj.id = "beef_block"
    obj.operation = CollisionObject.ADD
    obj.primitives = [
        SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[wx, wy, hz])
    ]
    obj.primitive_poses = [
        Pose(position=Point(x=cx, y=cy, z=cz),
             orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
    ]

    # 通过 ApplyPlanningScene 服务一次性添加物体+颜色
    req = ApplyPlanningScene.Request()
    req.scene.is_diff = True
    req.scene.world.collision_objects = [obj]
    req.scene.object_colors = [
        ObjectColor(id="beef_block",
                    color=ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0))
    ]

    future = scene_cli.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

    if future.result() and future.result().success:
        node.get_logger().info("肉泥已加载 — 红色长方体 宽0.15m×长0.2m×高0.05m")
    else:
        node.get_logger().error("加载肉泥失败")

    # 打印可捶打坐标范围
    half_x = wx * 0.5
    half_y = wy * 0.5
    node.get_logger().info(
        f"\n========== 肉泥顶面可捶打范围 ==========\n"
        f"  X: [{cx - half_x:.3f}, {cx + half_x:.3f}]\n"
        f"  Y: [{cy - half_y:.3f}, {cy + half_y:.3f}]\n"
        f"  Z:  {strike_z:.3f} (顶面)\n"
        f"\n贴到 pound_inverted.yaml:\n"
        f"  中心:  strike_xyz: [{cx:.3f}, {cy:.3f}, {strike_z:.3f}]\n"
        f"  右前:  strike_xyz: [{cx + half_x:.3f}, {cy + half_y:.3f}, {strike_z:.3f}]\n"
        f"  右后:  strike_xyz: [{cx + half_x:.3f}, {cy - half_y:.3f}, {strike_z:.3f}]\n"
        f"  左前:  strike_xyz: [{cx - half_x:.3f}, {cy + half_y:.3f}, {strike_z:.3f}]\n"
        f"  左后:  strike_xyz: [{cx - half_x:.3f}, {cy - half_y:.3f}, {strike_z:.3f}]\n"
        f"========================================="
    )

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
