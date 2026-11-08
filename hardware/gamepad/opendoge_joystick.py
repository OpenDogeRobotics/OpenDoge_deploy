#!/usr/bin/env python3
"""Linux joystick helpers for OpenDoge command-file control."""

from __future__ import annotations

import array
import dataclasses
import errno
import fcntl
import glob
import os
import struct
from typing import Dict, Iterable, List, Optional


JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80
JSIOCGNAME_128 = 0x80806A13
EVENT_STRUCT = struct.Struct("IhBB")


class DeviceLostError(OSError):
    """Raised when /dev/input/js0 disappears (xboxdrv died or dongle unplugged)."""
    def __init__(self, device: str):
        self.device = device
        super().__init__(errno.ENODEV, f"joystick device lost: {device}")


@dataclasses.dataclass(frozen=True)
class JoystickEvent:
    timestamp_ms: int
    value: int
    event_type: int
    number: int

    @property
    def is_init(self) -> bool:
        return bool(self.event_type & JS_EVENT_INIT)

    @property
    def base_type(self) -> int:
        return self.event_type & ~JS_EVENT_INIT


class LinuxJoystick:
    """Non-blocking reader for /dev/input/js* devices."""

    def __init__(self, device: Optional[str] = None):
        self.device = device or first_joystick_device()
        self.fd: Optional[int] = None

    def open(self) -> None:
        if not self.device:
            raise FileNotFoundError("no /dev/input/js* joystick device found")
        self.fd = os.open(self.device, os.O_RDONLY | os.O_NONBLOCK)

    def close(self) -> None:
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def __enter__(self) -> "LinuxJoystick":
        self.open()
        return self

    def __exit__(self, *_args) -> None:
        self.close()

    def name(self) -> str:
        if self.fd is None:
            return ""
        buf = array.array("B", [0] * 128)
        try:
            fcntl.ioctl(self.fd, JSIOCGNAME_128, buf, True)
        except OSError:
            return ""
        raw = bytes(buf).split(b"\0", 1)[0]
        return raw.decode("utf-8", errors="replace")

    def read_events(self) -> List[JoystickEvent]:
        if self.fd is None:
            raise RuntimeError("joystick is not open")

        events: List[JoystickEvent] = []
        while True:
            try:
                data = os.read(self.fd, EVENT_STRUCT.size)
            except BlockingIOError:
                break
            except OSError as exc:
                if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                    break
                # ENODEV (19): xboxdrv died or dongle unplugged — surface as
                # DeviceLost so the caller can wait and re-open js0.
                if exc.errno == errno.ENODEV:
                    raise DeviceLostError(self.device) from exc
                raise

            if len(data) != EVENT_STRUCT.size:
                break
            events.append(JoystickEvent(*EVENT_STRUCT.unpack(data)))
        return events


def first_joystick_device() -> Optional[str]:
    devices = sorted(glob.glob("/dev/input/js*"))
    return devices[0] if devices else None


def apply_deadzone(value: float, deadzone: float) -> float:
    value = max(-1.0, min(1.0, value))
    if abs(value) < deadzone:
        return 0.0
    sign = 1.0 if value > 0.0 else -1.0
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)


def normalize_axis(raw_value: int) -> float:
    if raw_value < 0:
        return max(-1.0, raw_value / 32768.0)
    return min(1.0, raw_value / 32767.0)


@dataclasses.dataclass
class RobotCommand:
    vx: float = 0.0
    vy: float = 0.0
    yaw_rate: float = 0.0
    active: bool = False
    estop: bool = False
    position_control: bool = False
    rl_inference: bool = False
    clear_fault: bool = False
    low_gain_mode: bool = False


class XboxCommandMapper:
    """Maps common Xbox joystick axes/buttons to OpenDoge command fields."""

    # Standard Xbox 360 joystick button numbering
    BUTTON_A = 0     # A (south)
    BUTTON_B = 1     # B (east)
    BUTTON_X = 2     # X (west)
    BUTTON_Y = 3     # Y (north)
    BUTTON_LB = 4    # left bumper
    BUTTON_RB = 5    # right bumper
    BUTTON_BACK = 6  # back / view
    BUTTON_START = 7 # start / menu

    def __init__(
        self,
        max_vx: float,
        max_vy: float,
        max_yaw_rate: float,
        deadzone: float,
        axis_vy: int = 0,
        axis_vx: int = 1,
        axis_yaw: int = 2,  # right stick X (xboxdrv default layout)
        require_rb: bool = False,
        btn_a: int | None = None,
        btn_b: int | None = None,
        btn_x: int | None = None,
        btn_y: int | None = None,
        btn_back: int | None = None,
        btn_start: int | None = None,
        btn_rb: int | None = None,
        btn_estop: int | None = None,
    ):
        self.max_vx = max_vx
        self.max_vy = max_vy
        self.max_yaw_rate = max_yaw_rate
        self.deadzone = deadzone
        self.axis_vy = axis_vy
        self.axis_vx = axis_vx
        self.axis_yaw = axis_yaw
        self.require_rb = require_rb
        self.btn_a = btn_a if btn_a is not None else self.BUTTON_A
        self.btn_b = btn_b if btn_b is not None else self.BUTTON_B
        self.btn_x = btn_x if btn_x is not None else self.BUTTON_X
        self.btn_y = btn_y if btn_y is not None else self.BUTTON_Y
        self.btn_back = btn_back if btn_back is not None else self.BUTTON_BACK
        self.btn_start = btn_start if btn_start is not None else self.BUTTON_START
        self.btn_rb = btn_rb if btn_rb is not None else self.BUTTON_RB
        # Optional latching emergency-stop button. None disables it.
        self.btn_estop = btn_estop
        self.axes: Dict[int, float] = {}
        self.buttons: Dict[int, bool] = {}
        self.active_requested = False
        self.estop = False
        self.position_control = False
        self.rl_inference = False
        self.clear_fault = False
        self.low_gain_mode = False

    def update(self, events: Iterable[JoystickEvent]) -> None:
        for event in events:
            if event.base_type == JS_EVENT_AXIS:
                self.axes[event.number] = normalize_axis(event.value)
            elif event.base_type == JS_EVENT_BUTTON:
                pressed = event.value != 0
                self.buttons[event.number] = pressed
                if pressed and not event.is_init:
                    self._handle_button_down(event.number)

    def command(self) -> RobotCommand:
        # Raw joystick axes: X+ = right, Y+ = down.
        # Robot/URDF body frame (REP-103): X+ = forward, Y+ = left,
        # yaw+ = counter-clockwise. Negate all three so "stick right" maps to
        # "move right" and "stick up" maps to "move forward".
        vx_axis = apply_deadzone(-self.axes.get(self.axis_vx, 0.0), self.deadzone)
        vy_axis = apply_deadzone(-self.axes.get(self.axis_vy, 0.0), self.deadzone)
        yaw_axis = apply_deadzone(-self.axes.get(self.axis_yaw, 0.0), self.deadzone)

        active = self.active_requested and not self.estop
        if self.require_rb and not self.buttons.get(self.btn_rb, False):
            active = False

        # Low-gain mode suppresses policy activation and velocity commands
        if self.low_gain_mode:
            active = False
            vx_axis = 0.0
            vy_axis = 0.0
            yaw_axis = 0.0
        elif not active:
            vx_axis = 0.0
            vy_axis = 0.0
            yaw_axis = 0.0

        # clear_fault is a one-shot pulse: true for one cycle, then auto-reset
        clear_fault = self.clear_fault
        self.clear_fault = False

        # When not active or estop, mode flags are forced to false for consistency
        return RobotCommand(
            vx=vx_axis * self.max_vx,
            vy=vy_axis * self.max_vy,
            yaw_rate=yaw_axis * self.max_yaw_rate,
            active=active,
            estop=self.estop,
            position_control=self.position_control if active else False,
            rl_inference=self.rl_inference if active else False,
            clear_fault=clear_fault,
            low_gain_mode=self.low_gain_mode,
        )

    def _handle_button_down(self, button: int) -> None:
        # Latching emergency stop: press once -> estop, zero commands.
        # Press A afterwards to clear estop and recover.
        if self.btn_estop is not None and button == self.btn_estop:
            self.estop = True
            self.active_requested = False
            self.position_control = False
            self.rl_inference = False
            self.low_gain_mode = False
            return
        # A: Start robot and enter position control mode (also clears estop)
        if button == self.btn_a:
            self.estop = False
            self.clear_fault = True  # one-shot: recover from DampingFault
            self.low_gain_mode = False
            self.active_requested = True
            self.position_control = True
            self.rl_inference = False
        # B: Disable robot (motors enter protection mode)
        elif button == self.btn_b:
            self.active_requested = False
            self.position_control = False
            self.rl_inference = False
            self.low_gain_mode = False
        # X: Toggle RL inference (only available in position control mode)
        elif button == self.btn_x:
            if self.position_control and self.active_requested and not self.estop:
                self.rl_inference = True
                self.position_control = False
        # Y: Exit RL inference back to position control mode
        elif button == self.btn_y:
            if self.rl_inference and self.active_requested and not self.estop:
                self.rl_inference = False
                self.position_control = True
        elif button == self.btn_start:
            if not self.estop:
                self.active_requested = not self.active_requested
        elif button == self.btn_back:
            if not self.estop:
                self.low_gain_mode = not self.low_gain_mode
                if self.low_gain_mode:
                    self.active_requested = False


def format_command_file(command: RobotCommand) -> str:
    return (
        f"vx={command.vx:.6f}\n"
        f"vy={command.vy:.6f}\n"
        f"yaw_rate={command.yaw_rate:.6f}\n"
        f"active={'true' if command.active else 'false'}\n"
        f"estop={'true' if command.estop else 'false'}\n"
        f"position_control={'true' if command.position_control else 'false'}\n"
        f"rl_inference={'true' if command.rl_inference else 'false'}\n"
        f"clear_fault={'true' if command.clear_fault else 'false'}\n"
        f"low_gain_mode={'true' if command.low_gain_mode else 'false'}\n"
    )


def atomic_write(path: str, text: str) -> None:
    # Latest-value IPC file: the deploy polls it at 200 Hz. Durability across
    # a crash is irrelevant, and fsync on ext4 (/tmp is not tmpfs here) adds
    # several ms per write at 100 Hz, which shows up as gamepad latency.
    # flush() + os.replace() is enough for a reader to see a complete file.
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    tmp_path = f"{path}.tmp.{os.getpid()}"
    with open(tmp_path, "w", encoding="utf-8") as file:
        file.write(text)
        file.flush()
    os.replace(tmp_path, path)
