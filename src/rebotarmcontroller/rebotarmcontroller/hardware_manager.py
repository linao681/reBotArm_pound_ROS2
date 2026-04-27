from __future__ import annotations

import threading
import time
import sys
from pathlib import Path
from typing import Optional

import numpy as np
import yaml

from .conversions import fk_to_pose, pose_to_xyz_rpy

_G_MAX_DIST_M = 0.09
_G_ANGLE_OPEN = -5.0
_G_OPEN_SOFT_LIMIT = -4.9
_G_ARRIVE_TOL = 0.12
_G_TAU_MAX = 1.5
_G_KP_MOVE = 5.0
_G_KD_MOVE = 1.0
_G_DEFAULT_FORCE = 0.30
_G_CTRL_RATE = 500.0


class HardwareManager:
    """Owns the single RobotArm instance used by the ROS driver."""

    def __init__(
        self,
        arm_cfg: Optional[str] = None,
        gripper_cfg: Optional[str] = None,
        channel: str = "",
    ) -> None:
        self._sdk_root = self._ensure_rebot_sdk_in_syspath()

        from reBotArm_control_py.actuator import RobotArm
        from reBotArm_control_py.controllers import ArmEndPos

        cfg_path = Path(arm_cfg).expanduser() if arm_cfg else self.default_arm_cfg()
        cfg_path = self._arm_cfg_with_channel(cfg_path, channel)
        self._arm = RobotArm(cfg_path=str(cfg_path))

        self._gripper_cfg_path = (
            Path(gripper_cfg).expanduser() if gripper_cfg else self.default_gripper_cfg()
        )
        self._gripper_cfg = None
        self._gripper_mot = None
        self._gripper_ctrl = None
        self._gripper_target_angle = 0.0
        self._gripper_target_effort = _G_DEFAULT_FORCE
        self._gripper_active = False
        self._gripper_pos = 0.0
        self._gripper_vel = 0.0
        self._gripper_torque = 0.0
        self._gripper_loop_stop = threading.Event()
        self._gripper_loop_thread: threading.Thread | None = None
        self._gripper_loop_running = False
        self._gripper_lock = threading.Lock()

        self._endpos_ctrl = ArmEndPos(self._arm)
        self._connected = False
        self._enabled = False
        self._state_machine = "IDLE"
        self._error_codes: list[str] = []

        self._patch_arm_bus_lock()

    def default_arm_cfg(self) -> Path:
        return self._sdk_root / "config" / "arm.yaml"

    def default_gripper_cfg(self) -> Path:
        return self._sdk_root / "config" / "gripper.yaml"

    @staticmethod
    def _workspace_root() -> Path:
        return Path(__file__).resolve().parents[3]

    @classmethod
    def _sdk_candidates(cls) -> list[Path]:
        workspace = cls._workspace_root()
        return [
            workspace / "third_party" / "reBotArm_control_py",
            workspace / "sdk" / "reBotArm_control_py",
            Path.home() / "seeed" / "cameraws" / "sdk" / "reBotArm_control_py",
        ]

    @classmethod
    def _ensure_rebot_sdk_in_syspath(cls) -> Path:
        for root in cls._sdk_candidates():
            if (root / "reBotArm_control_py").is_dir():
                root_str = str(root)
                if root_str not in sys.path:
                    sys.path.insert(0, root_str)
                return root
        candidates = "\n".join(f"  - {path}" for path in cls._sdk_candidates())
        raise FileNotFoundError(
            "Cannot find reBotArm_control_py. Clone it into one of:\n"
            f"{candidates}"
        )

    @staticmethod
    def _arm_cfg_with_channel(cfg_path: Path, channel: str) -> Path:
        if not channel:
            return cfg_path
        with open(cfg_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        data["channel"] = channel
        tmp_dir = Path("/tmp") / "rebotarm_ros2"
        tmp_dir.mkdir(parents=True, exist_ok=True)
        tmp_path = tmp_dir / "arm_channel_override.yaml"
        with open(tmp_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, sort_keys=False)
        return tmp_path

    @property
    def arm(self):
        return self._arm

    @property
    def endpos_ctrl(self):
        return self._endpos_ctrl

    @property
    def joint_names(self) -> list[str]:
        return list(self._arm.joint_names)

    @property
    def mode(self) -> str:
        return str(self._arm.mode)

    @property
    def enabled(self) -> bool:
        return self._enabled

    @property
    def control_loop_active(self) -> bool:
        return bool(self._arm.control_loop_active)

    @property
    def has_gripper(self) -> bool:
        return self._gripper_mot is not None

    @property
    def state_machine(self) -> str:
        return self._state_machine

    @property
    def error_codes(self) -> list[str]:
        return list(self._error_codes)

    def set_state_machine(self, state: str) -> None:
        if state not in ("IDLE", "TRAJ_RUNNING", "LOWLEVEL_STREAMING"):
            raise ValueError(f"unsupported state machine value: {state}")
        self._state_machine = state

    def connect(self) -> None:
        if self._connected:
            return
        self._endpos_ctrl.start()
        self._connected = True
        self._enabled = True
        self.init_gripper(str(self._gripper_cfg_path))

    def shutdown(self) -> None:
        if not self._connected:
            return
        try:
            self._stop_gripper_loop()
            self._endpos_ctrl.end()
        finally:
            self._connected = False
            self._enabled = False

    def get_joint_state(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        return self._arm.get_state()

    def enable(self) -> None:
        from motorbridge import Mode

        self._arm.enable()
        if self._gripper_mot is not None:
            self._gripper_mot.ensure_mode(Mode.MIT, 1000)
        self._enabled = True
        if self.mode == "pos_vel" and not self.control_loop_active:
            self._start_pos_vel_loop()
        self.set_state_machine("IDLE")

    def disable(self) -> None:
        self._stop_control_loop()
        self._arm.disable()
        self._enabled = False
        self.set_state_machine("IDLE")

    def set_mode(self, mode: str) -> bool:
        mode = mode.strip().lower()
        if mode not in ("mit", "pos_vel", "vel"):
            raise ValueError(f"unsupported mode: {mode}")

        self._stop_control_loop()
        if mode == "mit":
            ok = self._arm.mode_mit()
        elif mode == "pos_vel":
            ok = self._arm.mode_pos_vel()
            if self._enabled:
                self._start_pos_vel_loop()
        else:
            ok = self._arm.mode_vel()
        self.set_state_machine("IDLE")
        return bool(ok)

    def set_zero(self, joint_name: str = "") -> bool:
        self._stop_control_loop()
        if joint_name:
            ok = self._arm.set_zero_single(joint_name)
        else:
            self._arm.set_zero()
            ok = True
        self._enabled = False
        self.set_state_machine("IDLE")
        return bool(ok)

    def safe_home(self) -> None:
        self.ensure_pos_vel_control()
        self._endpos_ctrl.safe_home()

    def ensure_pos_vel_control(self) -> None:
        if self.mode != "pos_vel":
            self._stop_control_loop()
            self._arm.mode_pos_vel()
        if not self._enabled:
            self._arm.enable()
            self._enabled = True
        if not self.control_loop_active:
            self._start_pos_vel_loop()

    def move_to_pose_ik(self, pose) -> tuple[bool, np.ndarray]:
        self.ensure_pos_vel_control()
        x, y, z, roll, pitch, yaw = pose_to_xyz_rpy(pose)
        ok = self._endpos_ctrl.move_to_ik(x, y, z, roll, pitch, yaw)
        return bool(ok), self._endpos_ctrl._q_target.copy()

    def move_to_pose_traj(self, pose, duration: float) -> bool:
        self.ensure_pos_vel_control()
        x, y, z, roll, pitch, yaw = pose_to_xyz_rpy(pose)
        return bool(self._endpos_ctrl.move_to_traj(x, y, z, roll, pitch, yaw, duration))

    def set_joint_target(self, joint_names: list[str], positions: list[float]) -> None:
        if set(joint_names) != set(self.joint_names):
            raise ValueError(f"trajectory joints must match {self.joint_names}")
        ordered = np.zeros(len(self.joint_names), dtype=np.float64)
        by_name = {name: float(pos) for name, pos in zip(joint_names, positions)}
        for i, name in enumerate(self.joint_names):
            ordered[i] = by_name[name]
        self._endpos_ctrl._q_target[:] = ordered

    def send_joint_motor_cmd(self, joint_name: str, cmd) -> None:
        if joint_name not in self._arm._motor_map:
            raise KeyError(f"unknown joint: {joint_name}")

        mot = self._arm._motor_map[joint_name]
        jc = next(j for j in self._arm._joints if j.name == joint_name)
        state = mot.get_state()

        pos = float(cmd.pos) if cmd.use_pos else float(state.pos if state is not None else 0.0)
        vel = float(cmd.vel) if cmd.use_vel else float(state.vel if state is not None else 0.0)
        kp = float(cmd.kp) if cmd.use_kp else float(jc.kp)
        kd = float(cmd.kd) if cmd.use_kd else float(jc.kd)
        tau = float(cmd.tau) if cmd.use_tau else 0.0
        vlim = float(cmd.vlim) if cmd.use_vlim else float(jc.vlim)

        if int(cmd.mode) == 0:
            mot.send_mit(pos, vel, kp, kd, tau)
        elif int(cmd.mode) == 1:
            mot.send_pos_vel(pos, vlim)
        elif int(cmd.mode) == 2:
            if not hasattr(mot, "send_vel"):
                raise RuntimeError(f"{joint_name} does not support send_vel")
            mot.send_vel(vel)
        else:
            raise ValueError(f"unsupported JointMotorCmd mode: {cmd.mode}")
        self.set_state_machine("LOWLEVEL_STREAMING")

    def current_pose(self):
        from reBotArm_control_py.kinematics import compute_fk

        q, _, _ = self.get_joint_state()
        position, rotation, _ = compute_fk(self._endpos_ctrl._model, q)
        return fk_to_pose(position, rotation)

    def motion_active(self) -> bool:
        return bool(getattr(self._endpos_ctrl, "_moving", False))

    def motion_progress(self) -> float:
        traj = getattr(self._endpos_ctrl, "_traj", [])
        if not traj:
            return 1.0
        idx = float(getattr(self._endpos_ctrl, "_traj_idx", 0))
        return max(0.0, min(1.0, idx / float(len(traj))))

    def cancel_motion(self) -> None:
        self._endpos_ctrl._stop_send.set()
        self._endpos_ctrl._moving = False

    def get_joint_status_codes(self) -> list[int]:
        codes: list[int] = []
        for name in self.joint_names:
            try:
                st = self._arm._motor_map[name].get_state()
                codes.append(int(st.status_code if st is not None else 0))
            except Exception:
                codes.append(0)
        return codes

    def init_gripper(self, cfg_path: str) -> None:
        from motorbridge import CallError, Mode
        from reBotArm_control_py.actuator.gripper import load_cfg as load_gripper_cfg

        gcfg = load_gripper_cfg(cfg_path)
        gc = gcfg["gripper"]
        self._gripper_cfg = gc

        vendor = gc.vendor
        if vendor not in self._arm._ctrl_map:
            raise RuntimeError(
                f"gripper vendor={vendor!r} cannot share the arm Controller"
            )
        ctrl = self._arm._ctrl_map[vendor]

        if vendor == "damiao":
            self._gripper_mot = ctrl.add_damiao_motor(gc.motor_id, gc.feedback_id, gc.model)
        elif vendor == "myactuator":
            self._gripper_mot = ctrl.add_myactuator_motor(gc.motor_id, gc.feedback_id, gc.model)
        elif vendor == "robstride":
            self._gripper_mot = ctrl.add_robstride_motor(gc.motor_id, gc.feedback_id, gc.model)
        else:
            raise ValueError(f"unsupported gripper vendor: {vendor!r}")

        self._gripper_ctrl = ctrl

        self._patch_controller_bus(ctrl)
        self._wrap_motor_bus(self._gripper_mot, ctrl._bus_lock)

        try:
            ctrl.enable_all()
            time.sleep(0.3)
        except CallError:
            pass
        self._gripper_mot.ensure_mode(Mode.MIT, 1000)
        self._start_gripper_loop()

    def set_gripper_target(self, position_m: float, max_effort: float = 0.0) -> None:
        if self._gripper_mot is None:
            raise RuntimeError("gripper is not initialized")
        distance = float(np.clip(position_m, 0.0, _G_MAX_DIST_M))
        target = max((distance / _G_MAX_DIST_M) * _G_ANGLE_OPEN, _G_OPEN_SOFT_LIMIT)
        effort = _G_DEFAULT_FORCE if max_effort <= 0.0 else float(max_effort)
        with self._gripper_lock:
            self._gripper_target_angle = target
            self._gripper_target_effort = float(np.clip(effort, 0.05, _G_TAU_MAX))
            self._gripper_active = True

    def wait_gripper_target(self, timeout: float = 3.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._gripper_lock:
                target = self._gripper_target_angle
            if abs(self._gripper_pos - target) < _G_ARRIVE_TOL:
                return True
            time.sleep(0.02)
        return False

    def set_gripper_position(self, position_m: float, max_effort: float = 0.0) -> tuple[bool, float]:
        self.set_gripper_target(position_m, max_effort)
        reached = self.wait_gripper_target()
        return reached, self.gripper_position_m()

    def get_gripper_state(self) -> tuple[float, float, float, int]:
        status = 0
        if self._gripper_mot is not None:
            try:
                st = self._gripper_mot.get_state()
                if st is not None:
                    status = int(st.status_code)
            except Exception:
                status = 0
        return self._gripper_pos, self._gripper_vel, self._gripper_torque, status

    def gripper_position_m(self) -> float:
        distance = (self._gripper_pos / _G_ANGLE_OPEN) * _G_MAX_DIST_M
        return float(np.clip(distance, 0.0, _G_MAX_DIST_M))

    def gripper_reached_target(self) -> bool:
        with self._gripper_lock:
            if not self._gripper_active:
                return True
            target = self._gripper_target_angle
        return abs(self._gripper_pos - target) < _G_ARRIVE_TOL

    def send_gripper_motor_cmd(self, cmd) -> None:
        if self._gripper_mot is None or self._gripper_cfg is None:
            raise RuntimeError("gripper is not initialized")
        state = self._gripper_mot.get_state()
        pos = float(cmd.pos) if cmd.use_pos else float(state.pos if state is not None else 0.0)
        vel = float(cmd.vel) if cmd.use_vel else float(state.vel if state is not None else 0.0)
        kp = float(cmd.kp) if cmd.use_kp else float(self._gripper_cfg.kp)
        kd = float(cmd.kd) if cmd.use_kd else float(self._gripper_cfg.kd)
        tau = float(cmd.tau) if cmd.use_tau else 0.0
        vlim = float(cmd.vlim) if cmd.use_vlim else float(self._gripper_cfg.vlim)

        if int(cmd.mode) == 0:
            self._gripper_mot.send_mit(pos, vel, kp, kd, tau)
        elif int(cmd.mode) == 1:
            self._gripper_mot.send_pos_vel(pos, vlim)
        elif int(cmd.mode) == 2:
            if not hasattr(self._gripper_mot, "send_vel"):
                raise RuntimeError("gripper does not support send_vel")
            self._gripper_mot.send_vel(vel)
        else:
            raise ValueError(f"unsupported JointMotorCmd mode: {cmd.mode}")
        with self._gripper_lock:
            self._gripper_active = False

    def _patch_arm_bus_lock(self) -> None:
        for ctrl in self._arm._ctrl_map.values():
            self._patch_controller_bus(ctrl)

        if not hasattr(self._arm, "_bus_lock_patched"):
            for jc in self._arm._joints:
                mot = self._arm._motor_map[jc.name]
                ctrl = self._arm._ctrl_map[jc.vendor]
                self._wrap_motor_bus(mot, ctrl._bus_lock)
            self._arm._bus_lock_patched = True

    @staticmethod
    def _patch_controller_bus(ctrl) -> None:
        if not hasattr(ctrl, "_bus_lock"):
            ctrl._bus_lock = threading.RLock()
        if hasattr(ctrl, "_bus_lock_patched"):
            return
        lock = ctrl._bus_lock

        def _wrap(fn, _lock=lock):
            def _locked(*args, **kwargs):
                with _lock:
                    return fn(*args, **kwargs)

            return _locked

        for attr in ("poll_feedback_once", "enable_all", "disable_all"):
            if hasattr(ctrl, attr):
                wrapped = _wrap(getattr(ctrl, attr))
                wrapped._rebotarm_locked = True
                setattr(ctrl, attr, wrapped)
        ctrl._bus_lock_patched = True

    @staticmethod
    def _wrap_motor_bus(mot, lock) -> None:
        def _wrap(fn, _lock=lock):
            def _locked(*args, **kwargs):
                with _lock:
                    return fn(*args, **kwargs)

            return _locked

        for attr in (
            "send_pos_vel",
            "send_mit",
            "send_vel",
            "request_feedback",
            "ensure_mode",
            "write_register_f32",
            "set_zero_position",
        ):
            if hasattr(mot, attr) and not hasattr(getattr(mot, attr), "_rebotarm_locked"):
                wrapped = _wrap(getattr(mot, attr))
                wrapped._rebotarm_locked = True
                setattr(mot, attr, wrapped)

    def _start_pos_vel_loop(self) -> None:
        if self.control_loop_active:
            return
        self._arm.start_control_loop(self._endpos_ctrl._loop_cb)
        self._endpos_ctrl._running = True

    def _stop_control_loop(self) -> None:
        self._arm.stop_control_loop()
        self._endpos_ctrl._running = False

    def _gripper_safe_mit(
        self,
        pos: float,
        vel: float,
        kp: float,
        kd: float,
        tau_ff: float = 0.0,
    ) -> None:
        if self._gripper_mot is None or self._gripper_ctrl is None:
            return
        pos_cmd = float(np.clip(pos, _G_OPEN_SOFT_LIMIT, 0.0))
        pos_term = kp * (pos_cmd - self._gripper_pos) + kd * (-self._gripper_vel)
        tau_safe = float(np.clip(pos_term + tau_ff, -_G_TAU_MAX, _G_TAU_MAX)) - pos_term
        lock = getattr(self._gripper_ctrl, "_bus_lock", None)
        try:
            if lock:
                with lock:
                    self._gripper_mot.send_mit(pos_cmd, vel, kp, kd, tau_safe)
                    self._gripper_mot.request_feedback()
                    self._gripper_ctrl.poll_feedback_once()
            else:
                self._gripper_mot.send_mit(pos_cmd, vel, kp, kd, tau_safe)
                self._gripper_mot.request_feedback()
                self._gripper_ctrl.poll_feedback_once()
        except Exception:
            pass

    def _gripper_tick(self) -> None:
        try:
            st = self._gripper_mot.get_state()
            if st is not None:
                self._gripper_pos = float(st.pos)
                self._gripper_vel = float(st.vel)
                self._gripper_torque = float(st.torq)
        except Exception:
            pass

        with self._gripper_lock:
            target = self._gripper_target_angle
            effort = self._gripper_target_effort
            active = self._gripper_active
        if not active:
            try:
                self._gripper_mot.request_feedback()
                self._gripper_ctrl.poll_feedback_once()
            except Exception:
                pass
            return
        tau_ff = effort if abs(target) < 1e-6 else 0.0
        self._gripper_safe_mit(target, 0.0, _G_KP_MOVE, _G_KD_MOVE, tau_ff)

    def _gripper_loop(self) -> None:
        dt = 1.0 / _G_CTRL_RATE
        last = time.perf_counter()
        while not self._gripper_loop_stop.is_set():
            now = time.perf_counter()
            if now - last >= dt:
                last += dt
                self._gripper_tick()
            else:
                time.sleep(1e-4)

    def _start_gripper_loop(self) -> None:
        if self._gripper_loop_running:
            return
        self._gripper_loop_stop.clear()
        self._gripper_loop_thread = threading.Thread(
            target=self._gripper_loop,
            name="rebotarm-gripper-loop",
            daemon=True,
        )
        self._gripper_loop_thread.start()
        self._gripper_loop_running = True

    def _stop_gripper_loop(self) -> None:
        if not self._gripper_loop_running:
            return
        self._gripper_loop_stop.set()
        if self._gripper_loop_thread is not None:
            self._gripper_loop_thread.join(timeout=1.0)
            self._gripper_loop_thread = None
        self._gripper_loop_running = False
