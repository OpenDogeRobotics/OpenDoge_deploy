#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

MUJOCO_DIR = Path(__file__).resolve().parents[1]
if str(MUJOCO_DIR) not in sys.path:
    sys.path.insert(0, str(MUJOCO_DIR))

from opendoge_mujoco.position_controller import JointCommand, PDPositionController


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


def sine_command(
    t: float,
    joint_names: list[str],
    default_q: np.ndarray,
    sine_config: dict[str, Any],
) -> JointCommand:
    frequency_hz = float(sine_config["frequency_hz"])
    omega = 2.0 * math.pi * frequency_hz
    amplitudes = sine_config["amplitude_rad"]
    q_des = default_q.copy()
    dq_des = np.zeros_like(default_q)

    for i, name in enumerate(joint_names):
        group = joint_group(name)
        amp = float(amplitudes[group])
        leg = name.split("_", 1)[0]
        phase = 0.0 if leg in {"FL", "RR"} else math.pi
        signal = math.sin(omega * t + phase)
        d_signal = omega * math.cos(omega * t + phase)
        q_des[i] += amp * signal
        dq_des[i] = amp * d_signal

    return JointCommand(q_des=q_des, dq_des=dq_des)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run OpenDoge MuJoCo PD position control.")
    parser.add_argument(
        "--config",
        type=Path,
        default=MUJOCO_DIR / "configs" / "position_control.json",
        help="Path to the JSON simulation config.",
    )
    parser.add_argument(
        "--mode",
        choices=("stand", "sine"),
        default="stand",
        help="Position target mode. 'stand' holds the default pose; 'sine' adds joint sinusoid motion.",
    )
    parser.add_argument("--duration", type=float, default=None, help="Override simulation duration in seconds.")
    parser.add_argument("--model", type=Path, default=None, help="Override MuJoCo XML model path.")
    parser.add_argument("--no-render", action="store_true", help="Run headless without the MuJoCo viewer.")
    parser.add_argument("--print-rate", type=float, default=2.0, help="Telemetry print rate in Hz.")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    config_path = args.config.resolve()
    config = load_config(config_path)

    model_path = args.model.resolve() if args.model else resolve_from_config(config_path, config["model_path"])
    model = mujoco.MjModel.from_xml_path(str(model_path))
    model.opt.timestep = float(config["control_dt"])
    data = mujoco.MjData(model)

    joint_names = list(config["joint_order"])
    kp, kd = expand_gains(joint_names, config["gains"])
    controller = PDPositionController(model, joint_names, kp, kd)
    default_q = ordered_values(joint_names, config["default_joint_angles"])
    initialize_pose(model, data, controller, default_q, config["base_pose"])

    duration = float(config["duration"] if args.duration is None else args.duration)
    render = bool(config["render"]) and not args.no_render
    print_period = 1.0 / args.print_rate if args.print_rate > 0 else math.inf
    next_print = 0.0

    def step_once() -> None:
        nonlocal next_print
        if args.mode == "sine":
            command = sine_command(data.time, joint_names, default_q, config["sine_motion"])
        else:
            command = JointCommand(q_des=default_q, dq_des=np.zeros_like(default_q))

        tau = controller.apply(data, command)
        mujoco.mj_step(model, data)

        if data.time >= next_print:
            q_err = command.q_des - controller.joint_positions(data)
            print(
                f"t={data.time:7.3f}s  "
                f"max_abs_q_err={np.max(np.abs(q_err)):.4f}rad  "
                f"max_abs_tau={np.max(np.abs(tau)):.3f}Nm"
            )
            next_print += print_period

    if render:
        from mujoco import viewer as mujoco_viewer

        with mujoco_viewer.launch_passive(model, data) as viewer:
            while viewer.is_running() and (duration <= 0.0 or data.time < duration):
                loop_start = time.monotonic()
                step_once()
                viewer.sync()
                sleep_time = model.opt.timestep - (time.monotonic() - loop_start)
                if sleep_time > 0:
                    time.sleep(sleep_time)
    else:
        while duration <= 0.0 or data.time < duration:
            step_once()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
