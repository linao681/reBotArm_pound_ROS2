#!/usr/bin/env python3
"""Load, configure, and activate all controllers for the demo simulation."""

import time
import rclpy
from rclpy.executors import ExternalShutdownException
from controller_manager_msgs.srv import ConfigureController, LoadController, SwitchController


def main():
    rclpy.init()
    node = rclpy.create_node("load_controllers")
    logger = node.get_logger()

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

    logger.info("waiting for controller_manager services...")
    for svc, name in [(load_cli, "load"), (cfg_cli, "configure"), (switch_cli, "switch")]:
        deadline = time.monotonic() + 60.0
        while time.monotonic() < deadline:
            if svc.wait_for_service(timeout_sec=2.0):
                break
            logger.info(f"still waiting for {name}_controller...")

    def _call(client, request):
        future = client.call_async(request)
        try:
            rclpy.spin_until_future_complete(node, future, timeout_sec=15.0)
            return future.result()
        except ExternalShutdownException:
            return None

    for ctrl in controllers:
        req = LoadController.Request()
        req.name = ctrl
        res = _call(load_cli, req)
        logger.info(f"loaded {ctrl}: {res.ok if res else 'no_response'}")

    for ctrl in controllers:
        req = ConfigureController.Request()
        req.name = ctrl
        res = _call(cfg_cli, req)
        if res is None or not res.ok:
            logger.error(f"configure {ctrl} failed")
            rclpy.shutdown()
            return
        logger.info(f"configured {ctrl}")

    req = SwitchController.Request()
    req.activate_controllers = controllers
    req.strictness = SwitchController.Request.BEST_EFFORT
    res = _call(switch_cli, req)
    if res is not None and res.ok:
        logger.info("all controllers activated")
    else:
        logger.warn(f"activate finished: {res}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
