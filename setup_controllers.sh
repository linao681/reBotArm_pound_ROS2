#!/bin/bash
# 启动仿真后运行此脚本加载控制器
# 用法: source ~/ros2_ws/setup_controllers.sh

source /opt/ros/humble/setup.bash

echo "=== 加载控制器 ==="
for ctrl in joint_state_broadcaster rebotarm_controller gripper_controller; do
    ros2 service call /controller_manager/load_controller \
        controller_manager_msgs/srv/LoadController "{name: $ctrl}" | tail -1
done

echo "=== 配置控制器 ==="
for ctrl in joint_state_broadcaster rebotarm_controller gripper_controller; do
    ros2 service call /controller_manager/configure_controller \
        controller_manager_msgs/srv/ConfigureController "{name: $ctrl}" | tail -1
done

echo "=== 激活控制器 ==="
for ctrl in joint_state_broadcaster rebotarm_controller gripper_controller; do
    ros2 control set_controller_state "$ctrl" active 2>&1 | tail -1
done

echo "=== 完成 ==="
ros2 control list_controllers
