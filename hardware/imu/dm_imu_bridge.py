#!/usr/bin/env python3
"""Bridge DM-IMU-L1 data to OpenDoge imu.state."""

from __future__ import annotations

import argparse
import math
import os
import socket
import struct
import sys
import termios
import time
import tty
from dataclasses import dataclass
from typing import Iterable, Optional, Tuple


ACCEL_CAN_MIN = -235.2
ACCEL_CAN_MAX = 235.2
GYRO_CAN_MIN = -34.88
GYRO_CAN_MAX = 34.88
QUAT_CAN_MIN = -1.0
QUAT_CAN_MAX = 1.0

CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_FRAME_STRUCT = struct.Struct("=IB3x8s")


@dataclass
class ImuState:
    gyro: Optional[Tuple[float, float, float]] = None
    quat: Optional[Tuple[float, float, float, float]] = None
    accel: Optional[Tuple[float, float, float]] = None
    last_update_s: float = 0.0
    # Per-component receipt times/sequences. A single global sequence can be
    # advanced by an accel frame while the gyro/quat are frozen; the deploy
    # needs to see each path separately.
    gyro_time_s: float = 0.0
    quat_time_s: float = 0.0
    gyro_seq: int = 0
    quat_seq: int = 0

    def ready(self) -> bool:
        return self.gyro is not None and self.quat is not None


def uint_to_float(value: int, low: float, high: float, bits: int) -> float:
    return float(value) * (high - low) / float((1 << bits) - 1) + low


def normalize_quat(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    norm = math.sqrt(sum(v * v for v in q))
    if norm <= 1.0e-9:
        return 1.0, 0.0, 0.0, 0.0
    return tuple(v / norm for v in q)  # type: ignore[return-value]


def quat_rotate_inverse(q: Tuple[float, float, float, float], v: Tuple[float, float, float]) -> Tuple[float, float, float]:
    w, x, y, z = normalize_quat(q)
    qv = (x, y, z)
    dot = qv[0] * v[0] + qv[1] * v[1] + qv[2] * v[2]
    cross = (
        qv[1] * v[2] - qv[2] * v[1],
        qv[2] * v[0] - qv[0] * v[2],
        qv[0] * v[1] - qv[1] * v[0],
    )
    scale = 2.0 * w * w - 1.0
    return (
        v[0] * scale - 2.0 * w * cross[0] + 2.0 * dot * qv[0],
        v[1] * scale - 2.0 * w * cross[1] + 2.0 * dot * qv[1],
        v[2] * scale - 2.0 * w * cross[2] + 2.0 * dot * qv[2],
    )


def parse_axis_map(axis_map: str, axis_signs: str):
    axis_map = axis_map.strip().lower()
    if sorted(axis_map) != ["x", "y", "z"] or len(axis_map) != 3:
        raise ValueError("--axis-map must be a permutation of xyz")
    signs = tuple(float(x) for x in axis_signs.replace(",", " ").split())
    if len(signs) != 3 or any(abs(s) != 1.0 for s in signs):
        raise ValueError("--axis-signs must contain three values, each 1 or -1")
    indices = {"x": 0, "y": 1, "z": 2}
    order = tuple(indices[c] for c in axis_map)
    return order, signs


def remap_vec(v: Tuple[float, float, float], order, signs) -> Tuple[float, float, float]:
    return tuple(signs[i] * v[order[i]] for i in range(3))  # type: ignore[return-value]


def remap_quat(q: Tuple[float, float, float, float], order, signs) -> Tuple[float, float, float, float]:
    w, x, y, z = q
    vec = remap_vec((x, y, z), order, signs)
    return normalize_quat((w, vec[0], vec[1], vec[2]))


def atomic_write(path: str, text: str) -> None:
    # Latest-value IPC file: the deploy polls it at 200 Hz. fsync on ext4
    # (/tmp is not tmpfs here) is unnecessary for this and very costly at 1 kHz.
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    tmp_path = f"{path}.tmp.{os.getpid()}"
    with open(tmp_path, "w", encoding="utf-8") as file:
        file.write(text)
        file.flush()
    os.replace(tmp_path, path)


def format_imu_state(
    gyro: Tuple[float, float, float],
    gravity: Tuple[float, float, float],
    stamp_s: float = 0.0,
    gyro_time_s: float = 0.0,
    quat_time_s: float = 0.0,
    gyro_seq: int = 0,
    quat_seq: int = 0,
) -> str:
    return (
        f"wx={gyro[0]:.9f}\n"
        f"wy={gyro[1]:.9f}\n"
        f"wz={gyro[2]:.9f}\n"
        f"gx={gravity[0]:.9f}\n"
        f"gy={gravity[1]:.9f}\n"
        f"gz={gravity[2]:.9f}\n"
        # Monotonic receipt timestamps (CLOCK_MONOTONIC; shared with the
        # deploy's steady_clock on Linux). t is the latest frame; gyro_t/quat_t
        # are per-component. seq lets the deploy detect a frozen component
        # even when the bridge rewrites its cached sample every 5 ms.
        f"t={stamp_s:.9f}\n"
        f"gyro_t={gyro_time_s:.9f}\n"
        f"gyro_seq={gyro_seq}\n"
        f"quat_t={quat_time_s:.9f}\n"
        f"quat_seq={quat_seq}\n"
    )


class DeviceLostError(RuntimeError):
    """USB serial device disappeared or was re-enumerated to a new tty."""


class SerialActiveReader:
    def __init__(self, device: str, baud: int, check_crc: bool):
        self.device = device
        self.baud = baud
        self.check_crc = check_crc
        self.fd: Optional[int] = None
        self.buffer = bytearray()
        self.real_path: Optional[str] = None

    def __enter__(self) -> "SerialActiveReader":
        self.fd = os.open(self.device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        configure_serial(self.fd, self.baud)
        self.real_path = os.path.realpath(self.device)
        return self

    def __exit__(self, *_args) -> None:
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def read_updates(self) -> Iterable[Tuple[int, Tuple[float, ...]]]:
        if self.fd is None:
            return []
        # A /dev/serial/by-id symlink is retargeted on USB re-enumeration
        # (ttyACM0 -> ttyACM1). The old fd then reads nothing forever, which
        # showed up as "IMU update timeout" while the service stayed alive.
        # Detect the retarget and let main() reopen the device.
        try:
            current = os.path.realpath(self.device)
        except OSError as exc:
            raise DeviceLostError(str(exc)) from exc
        if self.real_path and current != self.real_path:
            raise DeviceLostError(
                f"{self.device} -> {current} (was {self.real_path})")
        try:
            chunk = os.read(self.fd, 4096)
        except BlockingIOError:
            chunk = b""
        except OSError as exc:
            raise DeviceLostError(str(exc)) from exc
        if chunk:
            self.buffer.extend(chunk)
        return parse_serial_frames(self.buffer, self.check_crc)

    def write(self, data: bytes) -> None:
        if self.fd is None:
            raise RuntimeError("serial device is not open")
        try:
            os.write(self.fd, data)
        except OSError as exc:
            raise DeviceLostError(str(exc)) from exc


def configure_serial(fd: int, baud: int) -> None:
    attrs = termios.tcgetattr(fd)
    tty.setraw(fd)
    attrs = termios.tcgetattr(fd)
    speed = getattr(termios, f"B{baud}", None)
    if speed is None:
        raise ValueError(f"unsupported baud rate: {baud}")
    attrs[4] = speed
    attrs[5] = speed
    attrs[2] |= termios.CLOCAL | termios.CREAD
    attrs[2] &= ~termios.CSTOPB
    attrs[2] &= ~termios.PARENB
    attrs[2] &= ~termios.CSIZE
    attrs[2] |= termios.CS8
    attrs[3] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def parse_serial_frames(buffer: bytearray, check_crc: bool = False):
    updates = []
    while True:
        start = buffer.find(b"\x55\xAA")
        if start < 0:
            del buffer[:]
            break
        if start > 0:
            del buffer[:start]
        if len(buffer) < 5:
            break

        frame_type = buffer[3]
        payload_len = 16 if frame_type == 0x04 else 12 if frame_type in (0x01, 0x02, 0x03) else None
        if payload_len is None:
            del buffer[0]
            continue
        frame_len = 2 + 1 + 1 + payload_len + 2 + 1
        if len(buffer) < frame_len:
            break
        if buffer[frame_len - 1] != 0x0A:
            del buffer[0]
            continue
        if check_crc:
            expected = crc16_ccitt(bytes(buffer[: 4 + payload_len]))
            crc_lo = buffer[4 + payload_len]
            crc_hi = buffer[5 + payload_len]
            received_le = crc_lo | (crc_hi << 8)
            received_be = (crc_lo << 8) | crc_hi
            if expected not in (received_le, received_be):
                del buffer[0]
                continue
        payload = bytes(buffer[4 : 4 + payload_len])
        values = struct.unpack("<" + "f" * (payload_len // 4), payload)
        updates.append((frame_type, values))
        del buffer[:frame_len]
    return updates


def send_usb_setup(reader: SerialActiveReader, save: bool) -> None:
    # DM-IMU-L1 USB quick commands from the user manual.
    commands = [
        b"\xAA\x06\x01\x0D",  # enter setup mode
        b"\xAA\x0A\x00\x0D",  # output interface: USB
        b"\xAA\x01\x04\x0D",  # disable accel output
        b"\xAA\x01\x15\x0D",  # enable gyro output
        b"\xAA\x01\x06\x0D",  # disable euler output
        b"\xAA\x01\x17\x0D",  # enable quaternion output
    ]
    if save:
        commands.append(b"\xAA\x03\x01\x0D")  # save parameters
    commands.append(b"\xAA\x06\x00\x0D")  # enter normal mode

    for command in commands:
        reader.write(command)
        time.sleep(0.05)


class CanReader:
    def __init__(self, interface: str, expected_id: Optional[int]):
        self.interface = interface
        self.expected_id = expected_id
        self.sock: Optional[socket.socket] = None

    def __enter__(self) -> "CanReader":
        self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.setblocking(False)
        self.sock.bind((self.interface,))
        return self

    def __exit__(self, *_args) -> None:
        if self.sock is not None:
            self.sock.close()
            self.sock = None

    def read_updates(self):
        if self.sock is None:
            return []
        updates = []
        while True:
            try:
                frame = self.sock.recv(CAN_FRAME_STRUCT.size)
            except BlockingIOError:
                break
            if len(frame) != CAN_FRAME_STRUCT.size:
                continue
            can_id, can_dlc, data = CAN_FRAME_STRUCT.unpack(frame)
            if can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG):
                continue
            std_id = can_id & 0x7FF
            if self.expected_id is not None and std_id != self.expected_id:
                continue
            updates.extend(parse_can_data(data[:can_dlc]))
        return updates


def parse_can_data(data: bytes):
    if len(data) < 8:
        return []
    kind = data[0]
    if kind == 0x02:
        gyro = (
            uint_to_float(data[3] << 8 | data[2], GYRO_CAN_MIN, GYRO_CAN_MAX, 16),
            uint_to_float(data[5] << 8 | data[4], GYRO_CAN_MIN, GYRO_CAN_MAX, 16),
            uint_to_float(data[7] << 8 | data[6], GYRO_CAN_MIN, GYRO_CAN_MAX, 16),
        )
        return [(kind, gyro)]
    if kind == 0x04:
        w = data[1] << 6 | ((data[2] & 0xFC) >> 2)
        x = (data[2] & 0x03) << 12 | (data[3] << 4) | ((data[4] & 0xF0) >> 4)
        y = (data[4] & 0x0F) << 10 | (data[5] << 2) | ((data[6] & 0xC0) >> 6)
        z = (data[6] & 0x3F) << 8 | data[7]
        quat = (
            uint_to_float(w, QUAT_CAN_MIN, QUAT_CAN_MAX, 14),
            uint_to_float(x, QUAT_CAN_MIN, QUAT_CAN_MAX, 14),
            uint_to_float(y, QUAT_CAN_MIN, QUAT_CAN_MAX, 14),
            uint_to_float(z, QUAT_CAN_MIN, QUAT_CAN_MAX, 14),
        )
        return [(kind, quat)]
    return []


def apply_update(state: ImuState, frame_type: int, values: Tuple[float, ...], order, signs) -> None:
    now_s = time.monotonic()
    if frame_type == 0x01 and len(values) >= 3:
        state.accel = remap_vec((values[0], values[1], values[2]), order, signs)
    elif frame_type == 0x02 and len(values) >= 3:
        state.gyro = remap_vec((values[0], values[1], values[2]), order, signs)
        state.gyro_time_s = now_s
        state.gyro_seq += 1
    elif frame_type == 0x04 and len(values) >= 4:
        state.quat = remap_quat((values[0], values[1], values[2], values[3]), order, signs)
        state.quat_time_s = now_s
        state.quat_seq += 1
    state.last_update_s = now_s


def parse_args():
    parser = argparse.ArgumentParser(description="Bridge DM-IMU-L1 USB virtual serial or CAN output to OpenDoge imu.state.")
    parser.add_argument("--source", choices=["serial", "can"], default="serial")
    parser.add_argument("--device", help="Serial device for --source serial, e.g. /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=921600, help="Serial baud rate")
    parser.add_argument("--can", default="can4", help="SocketCAN interface for --source can")
    parser.add_argument("--can-id", type=lambda x: int(x, 0), default=None, help="Expected IMU CAN response StdId")
    parser.add_argument("--output", default="/tmp/opendoge_imu.state")
    parser.add_argument("--axis-map", default="xyz", help="Map robot xyz from IMU axes, e.g. xzy")
    parser.add_argument("--axis-signs", default="1,1,1", help="Signs for mapped axes, e.g. 1,-1,1")
    parser.add_argument("--timeout-sec", type=float, default=0.1, help="Warn if updates are older than this")
    parser.add_argument(
        "--configure-usb",
        action="store_true",
        help="Send USB quick commands: set USB output, enable gyro/quaternion, disable accel/euler",
    )
    parser.add_argument("--save-config", action="store_true", help="Save settings when used with --configure-usb")
    parser.add_argument("--check-crc", action="store_true", help="Validate USB serial CRC16 before accepting frames")
    parser.add_argument(
        "--gyro-lowpass-hz", type=float, default=0.0,
        help="Low-pass filter cutoff for wx/wy/wz (0=off). 40-60 Hz suppresses "
             "real-world foot-impact vibration; adds ~1/(2*pi*f) phase lag.")
    parser.add_argument("--quiet", action="store_true")
    return parser.parse_args()


def run(reader, output: str, order, signs, timeout_s: float, quiet: bool,
        lowpass_hz: float = 0.0, write_min_interval_s: float = 0.002) -> int:
    state = ImuState()
    next_status_s = 0.0
    last_write_s = 0.0
    last_filter_s = 0.0
    written_gyro_seq = 0
    written_quat_seq = 0
    filtered_gyro = None
    while True:
        for frame_type, values in reader.read_updates():
            apply_update(state, frame_type, values, order, signs)
        now_s = time.monotonic()
        gyro_updated = state.gyro_seq != written_gyro_seq
        quat_updated = state.quat_seq != written_quat_seq
        # Write as soon as a new gyro/quat sample is available, capped at
        # `write_min_interval_s`. The DM-IMU-L1 outputs gyro and quat at
        # ~1 kHz here; writing every frame (~2 kHz atomic_write+rename) makes
        # the bridge itself a scheduling/disk load and was observed to cause
        # 40-130 ms IMU age spikes. The deploy inference is 100 Hz, so a 2 ms
        # cap (500 Hz, age <= 2 ms) still removes most of the old 5 ms gate
        # without the file churn.
        if (state.ready() and (gyro_updated or quat_updated) and
                (write_min_interval_s <= 0.0 or
                 now_s - last_write_s >= write_min_interval_s)):
            gravity = quat_rotate_inverse(state.quat, (0.0, 0.0, -1.0))  # type: ignore[arg-type]
            gyro = state.gyro
            if lowpass_hz > 0.0 and gyro is not None:
                # First-order IIR per written sample; use the measured write
                # interval so the cutoff does not depend on the write cap.
                # Default off so the training contract (raw rad/s gyro) is
                # unchanged; 40-60 Hz is a useful sim2real filter for
                # foot-impact vibration.
                if gyro_updated:
                    dt = now_s - last_filter_s if last_filter_s > 0.0 else 0.005
                    dt = min(max(dt, 0.001), 0.02)
                    alpha = 1.0 - math.exp(-2.0 * math.pi * lowpass_hz * dt)
                    if filtered_gyro is None:
                        filtered_gyro = list(gyro)
                    else:
                        for i in range(3):
                            filtered_gyro[i] += alpha * (gyro[i] - filtered_gyro[i])
                    last_filter_s = now_s
                if filtered_gyro is not None:
                    gyro = tuple(filtered_gyro)
            atomic_write(output, format_imu_state(
                gyro, gravity,  # type: ignore[arg-type]
                stamp_s=state.last_update_s,
                gyro_time_s=state.gyro_time_s,
                quat_time_s=state.quat_time_s,
                gyro_seq=state.gyro_seq,
                quat_seq=state.quat_seq,
            ))
            last_write_s = now_s
            written_gyro_seq = state.gyro_seq
            written_quat_seq = state.quat_seq
        if not quiet and now_s >= next_status_s:
            age = now_s - state.last_update_s if state.last_update_s else float("inf")
            print(f"ready={int(state.ready())} age_ms={age * 1000.0:.1f} output={output}")
            next_status_s = now_s + 1.0
            if age > timeout_s and state.last_update_s:
                print("Warning: IMU update timeout", file=sys.stderr)
        time.sleep(0.001)


def main() -> int:
    args = parse_args()
    order, signs = parse_axis_map(args.axis_map, args.axis_signs)
    try:
        if args.source == "serial":
            if not args.device:
                print("--device is required for --source serial", file=sys.stderr)
                return 1
            factory = lambda: SerialActiveReader(  # noqa: E731
                args.device, args.baud, args.check_crc)
            configure = args.configure_usb
            label = args.device
        else:
            factory = lambda: CanReader(args.can, args.can_id)  # noqa: E731
            configure = False
            label = args.can

        # USB re-enumeration (ttyACM0 -> ttyACM1) invalidates the open fd.
        # The old loop kept the dead fd and printed "IMU update timeout"
        # forever; reopen and re-send --configure-usb instead.
        while True:
            try:
                with factory() as reader:
                    if configure:
                        send_usb_setup(reader, args.save_config)
                    return run(reader, args.output, order, signs,
                               args.timeout_sec, args.quiet,
                               args.gyro_lowpass_hz)
            except (DeviceLostError, FileNotFoundError, OSError) as exc:
                print(f"[reconnect] IMU lost ({label}): {exc}; "
                      f"waiting for device ...", file=sys.stderr)
                time.sleep(1.0)
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
