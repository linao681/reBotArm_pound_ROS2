# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

reBotArm B601-DM 6-DOF robotic arm ROS2 integration with MoveIt2 simulation and demo applications. The repository wraps a proprietary Python SDK (`reBotArm_control_py`) into standard ROS2 interfaces (topics, services, actions).

- ROS2 Distro: **Humble** (Ubuntu 22.04, Python 3.10)
- Build system: **colcon** with `--symlink-install`

## Build & Run

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Build all or specific packages
colcon build --symlink-install
colcon build --symlink-install --packages-select rebotarm_moveit_demos
```

## Simulation Startup

**Upright config (arm on ground, pointing up):**
```bash
ros2 launch rebotarm_moveit_config demo.launch.py use_rviz:=true
```

**Inverted config (arm hung from above, pointing down — beef pounding):**
```bash
ros2 launch rebotarm_moveit_config demo_inverted.launch.py use_rviz:=true
```

**If controllers fail to load (joint_states not publishing, execute_trajectory times out):**
```bash
source ~/ros2_ws/setup_controllers.sh
```

**Spawn beef block (red collision object for pounding):**
```bash
ros2 run rebotarm_moveit_demos spawn_beef
```

## Demo Apps

```bash
ros2 launch rebotarm_moveit_demos pound.launch.py          # upright pounding
ros2 launch rebotarm_moveit_demos pound_inverted.launch.py  # inverted pounding (beef)
ros2 launch rebotarm_moveit_demos draw_square.launch.py     # rectangle drawing
ros2 launch rebotarm_moveit_demos pick_place.launch.py      # pick and place
```

## Architecture

### 5 ROS2 Packages

| Package | Build | Role |
|---------|-------|------|
| `rebotarm_msgs` | ament_cmake | Custom .msg/.srv/.action IDL |
| `rebotarmcontroller` | ament_python | Core controller node (hardware only) |
| `rebotarm_bringup` | ament_python | Launch files, URDF, config YAMLs |
| `rebotarm_moveit_config` | ament_cmake | MoveIt2 config (URDF xacro, SRDF, kinematics, controllers) |
| `rebotarm_moveit_demos` | ament_python | Demo applications (pound, draw_square, pick_place) |

### Simulation vs Real Hardware

**Simulation:** `move_group` → `ros2_control_node` (mock_components/GenericSystem) → virtual motors. The `reBotArmController` node is NOT started.

**Real hardware:** `reBotArmController` node runs, communicates via CAN bus through `motorbridge` → `reBotArm_control_py` SDK → 6 Damiao motors + 1 gripper motor. MoveIt connects via topic remapping to the controller's action servers.

### Key Classes (rebotarm_moveit_demos)

- `MoveItDemoBase` (`demo_common.py`): Shared base class for all demos. Provides `go_home()`, `reset_scene()`, `execute_trajectory()`, `joint_trajectory()`, `current_joint_values()`. All parameters auto-declared from YAML config via `automatically_declare_parameters_from_overrides=True`.

- `PoundBeef` (`pound.py`): Beef pounding demo. Two modes:
  - **Cartesian** (`use_cartesian: true`): Reads `strike_xyz`/`lift_xyz` from YAML → Pinocchio IK solves joint angles (NOT KDL/MoveIt IK)
  - **Joint-space** (`use_cartesian: false`): Directly uses `lift_joints`/`strike_joints` from YAML

### Critical Config Files

- `pound_inverted.yaml` — inverted pounding parameters (the main config being tuned)
- `moveit_controllers.yaml` — `allowed_start_tolerance: 1.0`, `allowed_execution_duration_scaling: 4.0`
- `ompl_planning.yaml` — adapters must use `default_planner_request_adapters/*` (NOT `planning`); `AddTimeOptimalParameterization` in request_adapters
- `kinematics.yaml` — KDL solver timeout=2.0, but Pinocchio IK is preferred for inverted poses (KDL fails with -31)

### SDK Integration

The SDK (`third_party/reBotArm_control_py/`) provides Pinocchio-based kinematics/dynamics. Used in two ways:
1. `pound.py` imports it directly via `sys.path` manipulation for IK solving
2. `hardware_manager.py` (in rebotarmcontroller) uses it at runtime for FK, gravity compensation, motor control

### External Tool

`~/ros2_ws/solve_ik_pin.py` — standalone Pinocchio IK solver. Usage: `python3 solve_ik_pin.py x y z roll pitch yaw`. Prints joint angles in YAML format.

## Known Issues & Fixes Applied

1. **KDL IK solver fails for inverted arm**: Always returns -31 (NO_IK_SOLUTION). Workaround: use Pinocchio IK via SDK.
2. **Spawner "Controller already loaded" bug**: controller_manager spawner skips `load_controller`. Fixed by bypassing spawner entirely — `load_controllers.py` calls services directly: load → configure → activate.
3. **MoveItErrorCodes.message doesn't exist in Humble**: Removed all `.message` accesses from error logging.
4. **`_joint_state()` default `is_diff=True`**: Caused doubled joint values and OMPL "start tree could not be initialized". Changed default to `is_diff=False`.
5. **Multiple move_group instances**: Leftover zombie processes block `/execute_trajectory` action. Always run `pkill -9 -f "move_group|ros2_control_node|rviz2"` before restarting simulation.
6. **FastDDS SHM port conflicts**: Clean `/dev/shm/fastrtps_port*` between simulation restarts.
7. **`strike_xyz` and `lift_xyz` must have same X and Y**: Diagonal pounding loses force; vertical pounding (same XY, only Z changes) delivers maximum impact.
