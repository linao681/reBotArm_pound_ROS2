import os
import signal

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown, matches_action
from launch.events.process import SignalProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="moveit.rviz",
        description="RViz 配置文件路径",
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="是否启动 RViz",
    )
    mount_height = LaunchConfiguration("mount_height", default="0.9")

    # 构建 MoveIt 配置
    moveit_config = (
        MoveItConfigsBuilder("rebotarm", package_name="rebotarm_moveit_config")
        .robot_description(file_path="config/rebotarm.urdf.xacro")
        .robot_description_semantic(file_path="config/rebotarm.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # MoveIt 规划器
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # RViz 可视化
    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("rebotarm_moveit_config"),
            "launch",
            LaunchConfiguration("rviz_config"),
        ]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    # 倒装机械臂: roll=π 绕 X 轴翻转 180°, Z 轴朝下, 像人手从高处挂下来
    # mount_height 控制安装高度 (米), 默认 0.9m 相当于吊在半空
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=[
            "0", "0", mount_height,   # x, y, z: 抬高安装位
            "0", "0", "3.14159",       # yaw, pitch, roll: 翻转 180°
            "world", "base_link",
        ],
    )

    # 从 URDF 发布 tf 树
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control 模拟硬件节点
    ros2_controllers_path = os.path.join(
        get_package_share_directory("rebotarm_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="screen",
    )

    # 延迟 3 秒等 controller_manager 完全启动，然后自动加载三个控制器
    controller_loader = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="rebotarm_moveit_demos",
                executable="load_controllers",
                name="load_controllers",
                output="screen",
            )
        ],
    )

    # 级联关闭: RViz→move_group→ros2_control_node→shutdown
    return LaunchDescription(
        [
            DeclareLaunchArgument("mount_height", default_value="0.9"),
            rviz_config_arg,
            use_rviz_arg,
            static_tf_node,
            robot_state_publisher_node,
            ros2_control_node,
            controller_loader,
            move_group_node,
            rviz_node,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=rviz_node,
                    on_exit=[
                        EmitEvent(
                            event=SignalProcess(
                                signal_number=signal.SIGINT,
                                process_matcher=matches_action(move_group_node),
                            )
                        )
                    ],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=move_group_node,
                    on_exit=[
                        EmitEvent(
                            event=SignalProcess(
                                signal_number=signal.SIGINT,
                                process_matcher=matches_action(ros2_control_node),
                            )
                        )
                    ],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=ros2_control_node,
                    on_exit=[
                        EmitEvent(event=Shutdown(reason="ros2_control_node 退出"))
                    ],
                )
            ),
        ]
    )
