#!/usr/bin/env python3
"""Write OpenDoge command.state from an Xbox-compatible joystick."""

from __future__ import annotations

import argparse
import os
import signal
import sys
import time

from opendoge_joystick import (
    DeviceLostError,
    LinuxJoystick,
    XboxCommandMapper,
    atomic_write,
    first_joystick_device,
    format_command_file,
)


STOP = False


def apply_button_profile(mapper: XboxCommandMapper, args: argparse.Namespace,
                         device_name: str) -> None:
    """Apply native Zikway 3537:1041 button layout when appropriate.

    Native js0 mapping (aligned with sim2sim and docs/lainlab_deploy_migration.md):
      A=0, B=1, X=3, Y=4, LB/START=6, RB/BACK=7.
    Hardware-only latching emergency stop: btn11, the key with three parallel
    horizontal bars to the right of Home (not used by sim2sim).
    Explicit --btn-* arguments always win.
    """
    use_zikway = args.profile == "zikway" or (
        args.profile == "auto" and "zikway" in (device_name or "").lower()
    )
    if not use_zikway:
        return
    if args.btn_a is None:
        mapper.btn_a = 0
    if args.btn_b is None:
        mapper.btn_b = 1
    if args.btn_x is None:
        mapper.btn_x = 3
    if args.btn_y is None:
        mapper.btn_y = 4
    if args.btn_start is None:
        mapper.btn_start = 6
    if args.btn_back is None:
        mapper.btn_back = 7
    if args.btn_rb is None:
        mapper.btn_rb = 11
    if args.btn_estop is None:
        mapper.btn_estop = 11


def handle_signal(_signum, _frame) -> None:
    global STOP
    STOP = True


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Bridge /dev/input/js* Xbox controller input to OpenDoge command.state."
    )
    parser.add_argument("--device", default=None, help="Joystick device, default: first /dev/input/js*")
    parser.add_argument("--output", default="/tmp/opendoge_command.state", help="Command file to write")
    parser.add_argument("--rate-hz", type=float, default=100.0, help="Command output update rate")
    parser.add_argument("--deadzone", type=float, default=0.08, help="Joystick axis deadzone")
    # Full baseline 0.6/0.4/1.0. Bring-up used 75% then 50%; reverted to 100%
    # on 2026-11-04. LainLab flat training starts at vx ±0.8 / vy ±0.5 /
    # yaw ±0.8, so vx/vy are inside; yaw 1.0 is at/above the initial range —
    # lower --max-yaw-rate if full-stick turning is unstable.
    parser.add_argument("--max-vx", type=float, default=0.45, help="Max forward speed command in m/s (default 50% 0.60)")
    parser.add_argument("--max-vy", type=float, default=0.25, help="Max lateral speed command in m/s (default 50% 0.40)")
    parser.add_argument("--max-yaw-rate", type=float, default=0.70, help="Max yaw rate command in rad/s (default 50% 1.00)")
    parser.add_argument("--axis-vy", type=int, default=0, help="Axis number for lateral command")
    parser.add_argument("--axis-vx", type=int, default=1, help="Axis number for forward command")
    parser.add_argument("--axis-yaw", type=int, default=2, help="Axis number for yaw command (right stick X)")
    parser.add_argument("--btn-a", type=int, default=None, help="Button number for A (default: 0)")
    parser.add_argument("--btn-b", type=int, default=None, help="Button number for B (default: 1)")
    parser.add_argument("--btn-x", type=int, default=None, help="Button number for X (default: 3)")
    parser.add_argument("--btn-y", type=int, default=None, help="Button number for Y (default: 4)")
    parser.add_argument("--btn-back", type=int, default=None, help="Button number for BACK (default: 6)")
    parser.add_argument("--btn-start", type=int, default=None, help="Button number for START (default: 7)")
    parser.add_argument("--btn-rb", type=int, default=None, help="Button number for RB (default: 5)")
    parser.add_argument(
        "--btn-estop",
        type=int,
        default=None,
        help="Latching emergency-stop button (Zikway profile: 11). "
             "Press once to set estop; press A to clear it.",
    )
    parser.add_argument(
        "--profile",
        choices=["auto", "default", "zikway"],
        default="auto",
        help="Button profile: auto detects Zikway 3537:1041 native js0; "
             "default uses Xbox/xboxdrv numbering",
    )
    parser.add_argument(
        "--require-rb",
        action="store_true",
        help="Require holding RB for active output; releasing RB returns command active=false",
    )
    parser.add_argument("--quiet", action="store_true", help="Do not print periodic status")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    device = args.device or first_joystick_device()
    if not device:
        print("No joystick found under /dev/input/js*", file=sys.stderr)
        return 1

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    mapper = XboxCommandMapper(
        max_vx=args.max_vx,
        max_vy=args.max_vy,
        max_yaw_rate=args.max_yaw_rate,
        deadzone=args.deadzone,
        axis_vy=args.axis_vy,
        axis_vx=args.axis_vx,
        axis_yaw=args.axis_yaw,
        require_rb=args.require_rb,
        btn_a=args.btn_a,
        btn_b=args.btn_b,
        btn_x=args.btn_x,
        btn_y=args.btn_y,
        btn_back=args.btn_back,
        btn_start=args.btn_start,
        btn_rb=args.btn_rb,
        btn_estop=args.btn_estop,
    )

    period_s = 1.0 / max(args.rate_hz, 1.0)
    next_status_s = 0.0
    last_command = None
    header_printed = False

    # Outer reconnection loop — survives xboxdrv restarts and dongle re-plugs.
    while not STOP:
        current_device = device or first_joystick_device()
        if not current_device:
            if not header_printed:
                print(f"Waiting for /dev/input/js* ...", file=sys.stderr)
                header_printed = True
            time.sleep(1)
            continue

        try:
            with LinuxJoystick(current_device) as joystick:
                name = joystick.name() or "unknown joystick"
                apply_button_profile(mapper, args, name)
                print(f"Joystick: {current_device} ({name})")
                print(f"Profile:  {args.profile} (Zikway native mapping applied when detected)")
                print(f"Writing:  {args.output}")
                print(f"Rate:     {args.rate_hz} Hz")
                print(f"Button mapping: A=btn{mapper.btn_a} B=btn{mapper.btn_b} X=btn{mapper.btn_x} Y=btn{mapper.btn_y} BACK=btn{mapper.btn_back} START=btn{mapper.btn_start} ESTOP=btn{mapper.btn_estop}")
                print(f"Axis mapping:   vx=axis{mapper.axis_vx} vy=axis{mapper.axis_vy} yaw=axis{mapper.axis_yaw}")
                print("Controls:")
                print("  Left stick   -> vx (前后) / vy (左右)")
                print("  Right stick  -> yaw_rate (转向)")
                print("  A            -> 启动机器人 + 位置控制模式 (position_control)")
                print("  B            -> 退出位置控制/RL，回到阻尼 (active=false；不切断电机输出)")
                print("  X            -> 进入 RL 推理状态 (rl_inference)")
                print("  Y            -> 退出 RL 推理状态")
                print("  BACK         -> 急停 (estop)")
                print("  START        -> 切换使能")
                if args.require_rb:
                    print("  RB (hold)    -> 使能开关 (死手)")
                print()
                header_printed = True

                while not STOP:
                    loop_start_s = time.monotonic()
                    mapper.update(joystick.read_events())
                    command = mapper.command()
                    atomic_write(args.output, format_command_file(command))
                    last_command = command

                    now_s = time.monotonic()
                    if not args.quiet and now_s >= next_status_s:
                        mode_parts = []
                        if command.active:
                            mode_parts.append("ACTIVE")
                        else:
                            mode_parts.append("IDLE")
                        if command.estop:
                            mode_parts.append("ESTOP!")
                        if command.position_control:
                            mode_parts.append("POS-CTRL")
                        if command.rl_inference:
                            mode_parts.append("RL-INFER")
                        mode_str = " | ".join(mode_parts)
                        print(
                            f"[{mode_str}] "
                            f"vx={command.vx:+.3f} vy={command.vy:+.3f} yaw={command.yaw_rate:+.3f}"
                        )
                        next_status_s = now_s + 1.0

                    sleep_s = period_s - (time.monotonic() - loop_start_s)
                    if sleep_s > 0.0:
                        time.sleep(sleep_s)

        except (DeviceLostError, FileNotFoundError) as exc:
            print(f"[reconnect] Joystick lost: {exc} — waiting for device to return ...", file=sys.stderr)
            # Write safe neutral command so deploy doesn't hold stale active state.
            if last_command is not None:
                last_command.vx = 0.0
                last_command.vy = 0.0
                last_command.yaw_rate = 0.0
                last_command.active = False
                last_command.position_control = False
                last_command.rl_inference = False
                try:
                    atomic_write(args.output, format_command_file(last_command))
                except OSError:
                    pass
            time.sleep(2)
            device = None  # force re-scan for js0
            continue

    if last_command is not None:
        last_command.vx = 0.0
        last_command.vy = 0.0
        last_command.yaw_rate = 0.0
        last_command.active = False
        last_command.position_control = False
        last_command.rl_inference = False
        try:
            atomic_write(args.output, format_command_file(last_command))
        except OSError:
            pass
    print("Joystick bridge stopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
