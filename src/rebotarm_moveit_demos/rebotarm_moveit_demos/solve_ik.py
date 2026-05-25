#!/usr/bin/env python3
"""One-shot IK solver: given Cartesian xyz, print joint angles for pound config.

Usage:
    ros2 run rebotarm_moveit_demos solve_ik --ros-args -p x:=0.15 -p y:=0.0 -p z:=0.80
"""

import sys
import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from std_msgs.msg import Header
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from builtin_interfaces.msg import Duration


def main():
    rclpy.init()
    node = rclpy.create_node("solve_ik", allow_undeclared_parameters=True,
                             automatically_declare_parameters_from_overrides=True)

    x = node.get_parameter("x").value if node.has_parameter("x") else 0.15
    y = node.get_parameter("y").value if node.has_parameter("y") else 0.0
    z = node.get_parameter("z").value if node.has_parameter("z") else 0.80
    frame = "base_link"

    node.get_logger().info(f"target: [{x}, {y}, {z}] in {frame}")

    ik = node.create_client(GetPositionIK, "/compute_ik")
    if not ik.wait_for_service(timeout_sec=10.0):
        node.get_logger().error("IK service not available")
        return

    joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
                   "gripper_joint1", "gripper_joint2"]

    request = GetPositionIK.Request()
    request.ik_request.group_name = "arm"
    request.ik_request.robot_state.joint_state.name = joint_names
    request.ik_request.robot_state.joint_state.position = [0.0] * 8
    request.ik_request.robot_state.is_diff = False
    request.ik_request.avoid_collisions = False
    request.ik_request.ik_link_name = "gripper_tcp"
    request.ik_request.pose_stamped = PoseStamped(
        header=Header(frame_id=frame),
        pose=Pose(position=Point(x=float(x), y=float(y), z=float(z)),
                   orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)),
    )
    request.ik_request.timeout = Duration(sec=5, nanosec=0)

    future = ik.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

    resp = future.result()
    if resp is None or resp.error_code.val != MoveItErrorCodes.SUCCESS:
        code = resp.error_code.val if resp else "no response"
        node.get_logger().error(f"IK failed: {code}")
        return

    solution = [float(resp.solution.joint_state.position[i]) for i in range(6)]
    rpy = [round(v, 3) for v in solution]
    node.get_logger().info(f"joint angles: {rpy}")

    # Print in YAML format for easy copy-paste
    print(f"\nstrike_joints: {rpy}")
    print(f"lift_joints:   {[round(v - 0.0, 3) for v in solution]}  # adjust z offset for lift")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
