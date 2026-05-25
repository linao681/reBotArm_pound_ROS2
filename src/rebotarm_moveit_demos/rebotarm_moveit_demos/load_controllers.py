#!/usr/bin/env python3
"""仿真启动时自动加载、配置、激活三个控制器。

三个控制器:
  - joint_state_broadcaster: 发布 /joint_states
  - rebotarm_controller:     JointTrajectoryController（执行轨迹）
  - gripper_controller:      夹爪 JointTrajectoryController

流程: 等待 controller_manager 服务 → load → configure → switch(activate)
绕过 spawner 的 bug，直接用底层 service call。
"""

import time
import rclpy
from rclpy.executors import ExternalShutdownException
from controller_manager_msgs.srv import ConfigureController, LoadController, SwitchController


def main():
    rclpy.init()
    node = rclpy.create_node("load_controllers")
    logger = node.get_logger()

    # 三个 service 客户端
    load_cli = node.create_client(LoadController, "/controller_manager/load_controller")
    cfg_cli = node.create_client(
        ConfigureController, "/controller_manager/configure_controller"
    )
    switch_cli = node.create_client(
        SwitchController, "/controller_manager/switch_controller"
    )

    controllers = [
        "joint_state_broadcaster",
        "rebotarm_controller",
        "gripper_controller",
    ]

    # 等待 controller_manager 三个服务就绪（最久等 60 秒）
    logger.info("等待 controller_manager 服务...")
    for svc, name in [(load_cli, "load"), (cfg_cli, "configure"), (switch_cli, "switch")]:
        deadline = time.monotonic() + 60.0
        while time.monotonic() < deadline:
            if svc.wait_for_service(timeout_sec=2.0):
                break
            logger.info(f"还在等 {name}_controller...")

    def _call(client, request):
        """发送 service 请求并等待结果。"""
        future = client.call_async(request)
        try:
            rclpy.spin_until_future_complete(node, future, timeout_sec=15.0)
            return future.result()
        except ExternalShutdownException:
            return None

    # 步骤 1: 加载
    for ctrl in controllers:
        req = LoadController.Request()
        req.name = ctrl
        res = _call(load_cli, req)
        logger.info(f"加载 {ctrl}: {'成功' if (res and res.ok) else '失败'}")

    # 步骤 2: 配置
    for ctrl in controllers:
        req = ConfigureController.Request()
        req.name = ctrl
        res = _call(cfg_cli, req)
        if res is None or not res.ok:
            logger.error(f"配置 {ctrl} 失败")
            rclpy.shutdown()
            return
        logger.info(f"配置 {ctrl} 成功")

    # 步骤 3: 激活
    req = SwitchController.Request()
    req.activate_controllers = controllers
    req.strictness = SwitchController.Request.BEST_EFFORT
    res = _call(switch_cli, req)
    if res is not None and res.ok:
        logger.info("三个控制器全部激活")
    else:
        logger.warn(f"激活完成: {res}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
