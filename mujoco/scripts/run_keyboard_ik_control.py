#!/usr/bin/env python3
from __future__ import annotations

import argparse
import ctypes
import ctypes.util
import json
import math
import sys
import threading
import time
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

MUJOCO_DIR = Path(__file__).resolve().parents[1]
if str(MUJOCO_DIR) not in sys.path:
    sys.path.insert(0, str(MUJOCO_DIR))

from opendoge_mujoco.action_gait import BodyCommand, GaitConfig, TrotCycloidGait
from opendoge_mujoco.foot_track_gait import FootTrackGait, FootTrackParams
from opendoge_mujoco.imu_feedback import IMUFeedbackConfig, IMUReader, IMUStabilizer
from opendoge_mujoco.leg_ik import OpenDogeLegIK, load_leg_geometries_from_urdf
from opendoge_mujoco.position_controller import JointCommand, PDPositionController

KEY_ESCAPE = 256
KEY_SPACE = 32
KEY_LEFT = 263
KEY_RIGHT = 262
KEY_DOWN = 264
KEY_UP = 265
KEY_LEFT_CONTROL = 341
KEY_RIGHT_CONTROL = 345

XK_SPACE = 0x0020
XK_R = 0x0052
XK_R_LOWER = 0x0072
XK_ESCAPE = 0xFF1B
XK_LEFT = 0xFF51
XK_UP = 0xFF52
XK_RIGHT = 0xFF53
XK_DOWN = 0xFF54
XK_CONTROL_L = 0xFFE3
XK_CONTROL_R = 0xFFE4


def load_config(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def resolve_from_config(config_path: Path, value: str) -> Path:
    path = Path(value).expanduser()
    if path.is_absolute():
        return path
    return (config_path.parent / path).resolve()


def joint_group(joint_name: str) -> str:
    if "_hip_" in joint_name:
        return "hip"
    if "_thigh_" in joint_name:
        return "thigh"
    if "_calf_" in joint_name:
        return "calf"
    raise ValueError(f"Cannot infer gain group for joint: {joint_name}")


def expand_gains(joint_names: list[str], gains_config: dict[str, Any]) -> tuple[dict[str, float], dict[str, float]]:
    kp: dict[str, float] = {}
    kd: dict[str, float] = {}
    for name in joint_names:
        group = joint_group(name)
        kp[name] = float(gains_config[group]["kp"])
        kd[name] = float(gains_config[group]["kd"])
    return kp, kd


def ordered_values(joint_names: list[str], values: dict[str, float]) -> np.ndarray:
    missing = [name for name in joint_names if name not in values]
    if missing:
        raise ValueError(f"Missing joint values for: {', '.join(missing)}")
    return np.array([values[name] for name in joint_names], dtype=np.float64)


def initialize_pose(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    controller: PDPositionController,
    default_q: np.ndarray,
    base_pose: dict[str, Any],
) -> None:
    mujoco.mj_resetData(model, data)

    free_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "float_base")
    if free_joint_id >= 0:
        qpos_addr = model.jnt_qposadr[free_joint_id]
        data.qpos[qpos_addr : qpos_addr + 3] = np.array(base_pose["position"], dtype=np.float64)
        data.qpos[qpos_addr + 3 : qpos_addr + 7] = np.array(base_pose["quaternion"], dtype=np.float64)

    data.qpos[controller.qpos_addr] = controller.clamp_to_joint_limits(default_q)
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0
    mujoco.mj_forward(model, data)


def base_position(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    free_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "float_base")
    if free_joint_id < 0:
        return np.zeros(3, dtype=np.float64)
    qpos_addr = model.jnt_qposadr[free_joint_id]
    return data.qpos[qpos_addr : qpos_addr + 3].copy()


def base_yaw(model: mujoco.MjModel, data: mujoco.MjData) -> float:
    free_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "float_base")
    if free_joint_id < 0:
        return 0.0
    qpos_addr = model.jnt_qposadr[free_joint_id]
    w, x, y, z = data.qpos[qpos_addr + 3 : qpos_addr + 7]
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def base_pitch(model: mujoco.MjModel, data: mujoco.MjData) -> float:
    free_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "float_base")
    if free_joint_id < 0:
        return 0.0
    qpos_addr = model.jnt_qposadr[free_joint_id]
    w, x, y, z = data.qpos[qpos_addr + 3 : qpos_addr + 7]
    value = 2.0 * (w * y - z * x)
    return math.asin(max(-1.0, min(1.0, value)))


def command_from_key_state(up: bool, down: bool, left: bool, right: bool, turn_mode: bool) -> BodyCommand:
    command = BodyCommand()
    command.vx = float(up) - float(down)
    horizontal = float(left) - float(right)
    if turn_mode:
        command.yaw = horizontal
    else:
        command.vy = horizontal

    norm = math.hypot(command.vx, command.vy)
    if norm > 1.0:
        command.vx /= norm
        command.vy /= norm
    return command


class KeyboardCommand:
    def __init__(self, hold_timeout: float) -> None:
        self._lock = threading.Lock()
        self.hold_timeout = hold_timeout
        self.key_times: dict[int, float] = {}
        self.exit_requested = False
        self.reset_requested = False

    def on_key(self, key: int) -> None:
        now = time.monotonic()
        with self._lock:
            if key == KEY_ESCAPE:
                self.exit_requested = True
            elif key == KEY_SPACE:
                self.key_times.clear()
            elif key == ord("R"):
                self.key_times.clear()
                self.reset_requested = True
            elif key in {KEY_UP, KEY_DOWN, KEY_LEFT, KEY_RIGHT, KEY_LEFT_CONTROL, KEY_RIGHT_CONTROL}:
                self.key_times[key] = now

    def snapshot(self) -> tuple[BodyCommand, bool, bool, bool]:
        now = time.monotonic()
        with self._lock:
            expired = [key for key, key_time in self.key_times.items() if now - key_time > self.hold_timeout]
            for key in expired:
                del self.key_times[key]

            turn_mode = KEY_LEFT_CONTROL in self.key_times or KEY_RIGHT_CONTROL in self.key_times
            command = command_from_key_state(
                up=KEY_UP in self.key_times,
                down=KEY_DOWN in self.key_times,
                left=KEY_LEFT in self.key_times,
                right=KEY_RIGHT in self.key_times,
                turn_mode=turn_mode,
            )

            exit_requested = self.exit_requested
            reset_requested = self.reset_requested
            self.reset_requested = False
        return command, turn_mode, exit_requested, reset_requested


class X11KeyboardPoller:
    """Polls physical key state through Xlib without extra Python packages."""

    def __init__(self) -> None:
        lib_name = ctypes.util.find_library("X11")
        if not lib_name:
            raise RuntimeError("libX11 not found")

        self.x11 = ctypes.cdll.LoadLibrary(lib_name)
        self.x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
        self.x11.XOpenDisplay.restype = ctypes.c_void_p
        self.x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
        self.x11.XCloseDisplay.restype = ctypes.c_int
        self.x11.XQueryKeymap.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
        self.x11.XQueryKeymap.restype = ctypes.c_int
        self.x11.XKeysymToKeycode.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
        self.x11.XKeysymToKeycode.restype = ctypes.c_uint

        self.display = self.x11.XOpenDisplay(None)
        if not self.display:
            raise RuntimeError("cannot open X11 display")

        self.keycodes = {
            "escape": self._keycode(XK_ESCAPE),
            "space": self._keycode(XK_SPACE),
            "r": self._keycode(XK_R),
            "r_lower": self._keycode(XK_R_LOWER),
            "up": self._keycode(XK_UP),
            "down": self._keycode(XK_DOWN),
            "left": self._keycode(XK_LEFT),
            "right": self._keycode(XK_RIGHT),
            "ctrl_l": self._keycode(XK_CONTROL_L),
            "ctrl_r": self._keycode(XK_CONTROL_R),
        }
        self._last_reset_pressed = False

    def close(self) -> None:
        if self.display:
            self.x11.XCloseDisplay(self.display)
            self.display = None

    def snapshot(self) -> tuple[BodyCommand, bool, bool, bool]:
        keymap = ctypes.create_string_buffer(32)
        if not self.x11.XQueryKeymap(self.display, keymap):
            return BodyCommand(), False, False, False

        pressed = {name: self._pressed(keymap.raw, code) for name, code in self.keycodes.items()}
        turn_mode = pressed["ctrl_l"] or pressed["ctrl_r"]
        command = command_from_key_state(
            up=pressed["up"],
            down=pressed["down"],
            left=pressed["left"],
            right=pressed["right"],
            turn_mode=turn_mode,
        )
        reset_pressed = pressed["r"] or pressed["r_lower"]
        reset_requested = reset_pressed and not self._last_reset_pressed
        self._last_reset_pressed = reset_pressed
        stop_requested = pressed["space"]
        if stop_requested:
            command = BodyCommand()
        return command, turn_mode, pressed["escape"], reset_requested

    def _keycode(self, keysym: int) -> int:
        return int(self.x11.XKeysymToKeycode(self.display, keysym))

    @staticmethod
    def _pressed(keymap: bytes, keycode: int) -> bool:
        if keycode <= 0:
            return False
        return bool(keymap[keycode >> 3] & (1 << (keycode & 7)))

def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run OpenDoge keyboard action-IK position control.")
    parser.add_argument(
        "--config",
        type=Path,
        default=MUJOCO_DIR / "configs" / "position_control.json",
        help="Path to the JSON simulation config.",
    )
    parser.add_argument("--duration", type=float, default=None, help="Override simulation duration in seconds.")
    parser.add_argument("--model", type=Path, default=None, help="Override MuJoCo XML model path.")
    parser.add_argument("--urdf", type=Path, default=None, help="Override URDF path used for IK geometry.")
    parser.add_argument("--no-render", action="store_true", help="Run headless without the MuJoCo viewer.")
    parser.add_argument("--print-rate", type=float, default=2.0, help="Telemetry print rate in Hz.")
    parser.add_argument("--cmd-vx", type=float, default=0.0, help="Headless forward command in [-1, 1].")
    parser.add_argument("--cmd-vy", type=float, default=0.0, help="Headless lateral command in [-1, 1].")
    parser.add_argument("--cmd-yaw", type=float, default=0.0, help="Headless yaw command in [-1, 1].")
    parser.add_argument("--c-style", action="store_true", help="Use C-style foot track planner instead of TrotCycloidGait.")
    return parser


def command_text(command: BodyCommand, turn_mode: bool) -> str:
    mode = "turn" if turn_mode else "strafe"
    return f"mode={mode} vx={command.vx:+.1f} vy={command.vy:+.1f} yaw={command.yaw:+.1f}"


def main() -> int:
    args = build_parser().parse_args()
    config_path = args.config.resolve()
    config = load_config(config_path)
    ik_config = config["action_ik"]
    imu_config = config["imu_feedback"]

    model_path = args.model.resolve() if args.model else resolve_from_config(config_path, config["model_path"])
    urdf_path = args.urdf.resolve() if args.urdf else resolve_from_config(config_path, ik_config["urdf_path"])

    model = mujoco.MjModel.from_xml_path(str(model_path))
    model.opt.timestep = float(config["control_dt"])
    data = mujoco.MjData(model)
    imu_reader = IMUReader(model)
    imu_stabilizer = IMUStabilizer(
        IMUFeedbackConfig(
            enabled=bool(imu_config["enabled"]),
            heading_kp=float(imu_config["heading_kp"]),
            yaw_rate_kd=float(imu_config["yaw_rate_kd"]),
            max_yaw_correction=float(imu_config["max_yaw_correction"]),
            roll_kp=float(imu_config["roll_kp"]),
            roll_kd=float(imu_config["roll_kd"]),
            pitch_kp=float(imu_config["pitch_kp"]),
            pitch_kd=float(imu_config["pitch_kd"]),
            max_foot_z_correction=float(imu_config["max_foot_z_correction"]),
        )
    )

    joint_names = list(config["joint_order"])
    kp, kd = expand_gains(joint_names, config["gains"])
    controller = PDPositionController(model, joint_names, kp, kd)
    default_q = ordered_values(joint_names, config["default_joint_angles"])
    default_by_joint = dict(zip(joint_names, default_q.tolist()))

    legs = sorted({joint_name.split("_", 1)[0] for joint_name in joint_names})
    leg_geometries = load_leg_geometries_from_urdf(urdf_path, legs)
    ik = OpenDogeLegIK(leg_geometries, joint_names)
    nominal_feet = ik.nominal_feet(default_by_joint)
    if args.c_style:
        planner = FootTrackGait(nominal_feet, None)
        planner_display = f"C-style foot_track (h={planner.p.leg_high:.3f})"
    else:
        planner = TrotCycloidGait(
            nominal_feet,
            GaitConfig(
                cycle_time=float(ik_config["cycle_time"]),
                duty_factor=float(ik_config["duty_factor"]),
                step_height=float(ik_config["step_height"]),
                step_x=float(ik_config["step_x"]),
                step_y=float(ik_config["step_y"]),
                step_yaw=float(ik_config["step_yaw"]),
                rear_stance_height_offset=float(ik_config["rear_stance_height_offset"]),
            ),
        )
        planner_display = "TrotCycloidGait"

    initialize_pose(model, data, controller, default_q, config["base_pose"])
    imu_stabilizer.reset(imu_reader.read(data))
    last_q_des = default_q.copy()
    duration = float(config["duration"] if args.duration is None else args.duration)
    render = bool(config["render"]) and not args.no_render
    print_period = 1.0 / args.print_rate if args.print_rate > 0 else math.inf
    next_print = 0.0
    keyboard = KeyboardCommand(hold_timeout=float(ik_config["key_hold_timeout"]))
    max_joint_speed = float(ik_config["max_joint_speed_rad_s"])
    startup_blend_time = float(ik_config["startup_blend_time"])
    gait_start_time = 0.0
    gait_active = False
    headless_command = BodyCommand(
        vx=max(-1.0, min(1.0, args.cmd_vx)),
        vy=max(-1.0, min(1.0, args.cmd_vy)),
        yaw=max(-1.0, min(1.0, args.cmd_yaw)),
    )

    def step_once(command: BodyCommand, turn_mode: bool) -> np.ndarray:
        nonlocal gait_active, gait_start_time, last_q_des, next_print
        imu_state = imu_reader.read(data)
        command_active = max(abs(command.vx), abs(command.vy), abs(command.yaw)) > 1.0e-6
        corrected_command = imu_stabilizer.command(command, imu_state) if command_active else BodyCommand()
        if not command_active:
            imu_stabilizer.reset(imu_state)
        if command_active and not gait_active:
            gait_start_time = data.time
            imu_stabilizer.reset(imu_state)
        gait_active = command_active
        gait_time = data.time - gait_start_time if command_active else 0.0

        seed_by_joint = dict(zip(joint_names, last_q_des.tolist()))
        foot_targets = planner.targets(gait_time, corrected_command)
        blend = 1.0 if startup_blend_time <= 0.0 else min(1.0, gait_time / startup_blend_time)
        foot_positions = {
            leg: nominal_feet[leg] + blend * (target.position - nominal_feet[leg])
            for leg, target in foot_targets.items()
        }
        foot_positions = imu_stabilizer.feet(foot_positions, imu_state)
        q_ik = controller.clamp_to_joint_limits(ik.inverse_feet(foot_positions, seed_by_joint))
        max_step = max_joint_speed * model.opt.timestep
        q_step = np.clip(q_ik - last_q_des, -max_step, max_step)
        q_des = last_q_des + q_step
        dq_des = q_step / model.opt.timestep
        last_q_des = q_des

        tau = controller.apply(data, JointCommand(q_des=q_des, dq_des=dq_des))
        mujoco.mj_step(model, data)

        if data.time >= next_print:
            q_err = q_des - controller.joint_positions(data)
            print(
                f"t={data.time:7.3f}s  "
                f"{command_text(command, turn_mode)}  "
                f"yaw_fb={corrected_command.yaw - command.yaw:+.2f}  "
                f"max_abs_q_err={np.max(np.abs(q_err)):.4f}rad  "
                f"max_abs_dq_des={np.max(np.abs(dq_des)):.2f}rad/s  "
                f"max_abs_tau={np.max(np.abs(tau)):.3f}Nm  "
                f"base_xy=({base_position(model, data)[0]:+.3f},{base_position(model, data)[1]:+.3f})  "
                f"imu_pitch={imu_state.pitch:+.3f}rad  "
                f"imu_yaw={imu_state.yaw:+.3f}rad"
            )
            next_print += print_period
        return tau

    if render:
        from mujoco import viewer as mujoco_viewer

        try:
            key_source = X11KeyboardPoller()
            print("Keyboard backend: X11 polling, real press/release.")
        except RuntimeError as exc:
            key_source = keyboard
            print(f"Keyboard backend: MuJoCo callback fallback ({exc}); release uses key_hold_timeout.")

        print(f"Planner: {planner_display}")
        print("Keyboard: hold Up/Down forward/back, hold Left/Right strafe, hold Ctrl+Left/Right yaw, release to stand, Space stop, R reset, Esc exit.")
        viewer = None
        try:
            viewer = mujoco_viewer.launch_passive(model, data, key_callback=keyboard.on_key)
            while viewer.is_running() and (duration <= 0.0 or data.time < duration):
                loop_start = time.monotonic()
                command, turn_mode, exit_requested, reset_requested = key_source.snapshot()
                if exit_requested:
                    break
                    if reset_requested:
                        initialize_pose(model, data, controller, default_q, config["base_pose"])
                        imu_stabilizer.reset(imu_reader.read(data))
                        last_q_des = default_q.copy()

                step_once(command, turn_mode)
                viewer.set_texts(
                    (
                        mujoco.mjtFontScale.mjFONTSCALE_150,
                        mujoco.mjtGridPos.mjGRID_TOPLEFT,
                        f"OpenDoge IK demo [{planner_display}]",
                        command_text(command, turn_mode),
                    )
                )
                viewer.sync()

                sleep_time = model.opt.timestep - (time.monotonic() - loop_start)
                if sleep_time > 0:
                    time.sleep(sleep_time)
        finally:
            if viewer is not None:
                viewer.close()
                deadline = time.monotonic() + 2.0
                while viewer.is_running() and time.monotonic() < deadline:
                    time.sleep(0.01)
            if isinstance(key_source, X11KeyboardPoller):
                key_source.close()
    else:
        while duration <= 0.0 or data.time < duration:
            step_once(headless_command, abs(headless_command.yaw) > 1.0e-6)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
