#!/usr/bin/env python3
"""record_highrate.py — 只读 100/200 Hz 高频记录器（旁路 CAN + 文件 IPC）。

为什么需要它
============
`opendoge_deploy` 的 `/tmp/opendoge_status.json` 只有 **1 Hz**，看不到 3–10 Hz 的
机体/步态震荡，也看不到一次动作周期内 target 的快速变化。本工具在 **不修改 deploy、
不占用实时线程** 的前提下：

  - 另开 4 个 `AF_CAN` raw socket（`CAN_RAW`），**只 recv、从不 send**；
  - 解码 deploy 发出的 type-1 运控帧（target q/dq/kp/kd/tau）和电机回的 type-2
    状态帧（q/dq/tau/temp/fault/mode），type-17 补扫回包也一并记录；
  - 同时读 `/tmp/opendoge_imu.state`（带内 `t/seq`）和
    `/tmp/opendoge_command.state`，以及 1 Hz 的 deploy `state`；
  - 按 `--rate-hz`（默认 100 Hz）写出 **宽表 CSV**，每行 12 个关节 + IMU + 命令，
    字段与 `monitor_status.py` 同源（q/target/dq/tau/tau_j/temp/fault/age/flags）。

CAN 读取依赖 SocketCAN 的本地回环：deploy 的发送帧默认会送达同一接口上的其他
raw socket。若终端提示 `NO_TARGET`/`TGT_STALE`，说明没有收到 type-1 帧（例如
deploy 关闭了 loopback，或本工具运行在另一个 netns），此时 q/dq/tau 仍然有效。

用法
====
    # 默认写 logs/highrate/highrate_<时间>.csv，最多保留 5 个
    sudo python3 hardware/motor/record_highrate.py
    sudo python3 hardware/motor/record_highrate.py --rate-hz 200
    sudo python3 hardware/motor/record_highrate.py --keep 10
    sudo python3 hardware/motor/record_highrate.py --no-csv          # 只看终端
    sudo python3 hardware/motor/record_highrate.py --csv /tmp/hi.csv  # 追加，不轮转
    sudo python3 hardware/motor/record_highrate.py --duration 60

只读保证：脚本没有任何 CAN `send()` 调用。
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import select
import socket
import struct
import sys
import time
from collections import Counter
from pathlib import Path
from typing import Dict, Optional, Tuple

_SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = _SCRIPT_DIR.parent.parent
DEFAULT_CONFIG = REPO_ROOT / "deploy" / "configs" / "opendoge_deploy.conf"
DEFAULT_LOG_DIR = REPO_ROOT / "logs" / "highrate"
DEFAULT_IMU = Path("/tmp/opendoge_imu.state")
DEFAULT_COMMAND = Path("/tmp/opendoge_command.state")
DEFAULT_STATUS = Path("/tmp/opendoge_status.json")

CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_EFF_MASK = 0x1FFFFFFF

COMM_CONTROL = 0x01
COMM_STATUS = 0x02
COMM_READ_PARAM = 0x11

# Must match hardware/motor/el05_socketcan.cpp / el05_motor_menu.py.
P_MIN, P_MAX = -12.57, 12.57
V_MIN, V_MAX = -50.0, 50.0
T_MIN, T_MAX = -6.0, 6.0
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0
IDX_MECH_POS = 0x7019
IDX_MECH_VEL = 0x701B

# Fixed deployment mapping (see CLAUDE.md): motor id -> bus -> joint name.
JOINTS: Tuple[Tuple[str, str, int], ...] = (
    ("FL_hip_joint", "can0", 1),
    ("FL_thigh_joint", "can0", 2),
    ("FL_calf_joint", "can0", 3),
    ("FR_hip_joint", "can1", 4),
    ("FR_thigh_joint", "can1", 5),
    ("FR_calf_joint", "can1", 6),
    ("RL_hip_joint", "can2", 7),
    ("RL_thigh_joint", "can2", 8),
    ("RL_calf_joint", "can2", 9),
    ("RR_hip_joint", "can3", 10),
    ("RR_thigh_joint", "can3", 11),
    ("RR_calf_joint", "can3", 12),
)
JOINT_NAMES = [name for name, _bus, _mid in JOINTS]
JOINT_BUS = {name: bus for name, bus, _mid in JOINTS}

# Same states as monitor_status.py: only these compare q against target.
ACTIVE_STATES = {
    "entering_position", "active_pc", "active_rl",
    "standing_up", "low_gain_test",
}

MANAGED_LOG_RE = re.compile(r"^highrate_\d{8}_\d{6}(_\d{3})?\.csv$")

PER_JOINT_COLUMNS = (
    "q", "target", "dq", "tau", "tau_j", "temp", "fault",
    "status_age_ms", "target_age_ms",
)

CSV_HEADER = [
    "t", "wall", "state", "fault_reason", "flags",
    "cmd_vx", "cmd_vy", "cmd_yaw",
    "cmd_active", "cmd_estop", "cmd_pc", "cmd_rl", "cmd_low_gain",
    "imu_wx", "imu_wy", "imu_wz", "imu_gx", "imu_gy", "imu_gz",
    "imu_age_ms", "imu_gyro_seq", "imu_quat_seq",
]
for _name in JOINT_NAMES:
    CSV_HEADER.extend(f"{_name}_{field}" for field in PER_JOINT_COLUMNS)


# ──────────────────────────────────────────────────────────────────────
# Protocol helpers
# ──────────────────────────────────────────────────────────────────────
def uint_to_float(value: int, low: float, high: float) -> float:
    return float(value) * (high - low) / 65535.0 + low


def float_le(data: bytes, offset: int) -> float:
    return struct.unpack_from("<f", data, offset)[0]


def build_ext_id(comm_type: int, data2: int, target_id: int) -> int:
    return ((comm_type & 0x1F) << 24) | ((data2 & 0xFFFF) << 8) | (target_id & 0xFF)


def parse_ext_id(can_id: int) -> Tuple[int, int, int]:
    raw = can_id & CAN_EFF_MASK
    return (raw >> 24) & 0x1F, (raw >> 8) & 0xFFFF, raw & 0xFF


def decode_frame(can_id: int, data: bytes) -> Optional[dict]:
    """Decode one EL05 EFF frame.

    ID layouts differ by comm type (see buildExtId / el05_motor_menu.py):
      - control (0x01): motor id in bits 7:0, tau placeholder in bits 23:8;
      - status  (0x02) / read reply (0x11): motor id in data2 bits 7:0.
    """
    comm, data2, low = parse_ext_id(can_id)
    if len(data) < 8:
        return None

    if comm == COMM_STATUS:
        motor_id = data2 & 0xFF
        return {
            "kind": "status",
            "motor": motor_id,
            "fault": (data2 >> 8) & 0x3F,
            "mode": (data2 >> 14) & 0x03,
            "q": uint_to_float((data[0] << 8) | data[1], P_MIN, P_MAX),
            "dq": uint_to_float((data[2] << 8) | data[3], V_MIN, V_MAX),
            "tau": uint_to_float((data[4] << 8) | data[5], T_MIN, T_MAX),
            "temp": ((data[6] << 8) | data[7]) * 0.1,
        }

    if comm == COMM_CONTROL:
        return {
            "kind": "control",
            "motor": low,
            "tau_ff": uint_to_float(data2, T_MIN, T_MAX),
            "q": uint_to_float((data[0] << 8) | data[1], P_MIN, P_MAX),
            "dq": uint_to_float((data[2] << 8) | data[3], V_MIN, V_MAX),
            "kp": uint_to_float((data[4] << 8) | data[5], KP_MIN, KP_MAX),
            "kd": uint_to_float((data[6] << 8) | data[7], KD_MIN, KD_MAX),
        }

    if comm == COMM_READ_PARAM:
        index = data[0] | (data[1] << 8)
        if index not in (IDX_MECH_POS, IDX_MECH_VEL):
            return None
        return {
            "kind": "read",
            "motor": data2 & 0xFF,
            "index": index,
            "value": float_le(data, 4),
        }
    return None


# ──────────────────────────────────────────────────────────────────────
# Calibration / unit conversion (same as types.hpp logicalPosition)
# ──────────────────────────────────────────────────────────────────────
def load_calibration(path: Path) -> Dict[str, dict]:
    calib = {
        name: {
            "direction": 1.0, "offset": 0.0, "reduction": 1.0,
            "lower": -math.inf, "upper": math.inf,
        }
        for name in JOINT_NAMES
    }
    try:
        text = path.read_text(encoding="utf-8")
    except OSError as exc:
        print(f"⚠️  读不到配置 {path}: {exc}（使用 direction=1/offset=0/reduction=1）",
              file=sys.stderr)
        return calib
    for raw in text.splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line or "=" not in line:
            continue
        key, value = line.split("=", 1)
        parts = key.strip().split(".")
        if len(parts) != 3 or parts[0] != "joint" or parts[1] not in calib:
            continue
        field = parts[2]
        if field not in ("direction", "offset", "reduction", "lower", "upper"):
            continue
        try:
            calib[parts[1]][field] = float(value.strip())
        except ValueError:
            pass
    for cfg in calib.values():
        if cfg["reduction"] <= 0.0:
            cfg["reduction"] = 1.0
        if cfg["direction"] == 0.0:
            cfg["direction"] = 1.0
    return calib


def to_logical_position(motor_position: float, calib: dict) -> float:
    return calib["direction"] * (motor_position / calib["reduction"] - calib["offset"])


def to_logical_velocity(motor_velocity: float, calib: dict) -> float:
    return calib["direction"] * motor_velocity / calib["reduction"]


def to_joint_torque(motor_torque: float, calib: dict) -> float:
    return motor_torque * calib["reduction"]


# ──────────────────────────────────────────────────────────────────────
# Managed CSV (same retention semantics as monitor_status.py)
# ──────────────────────────────────────────────────────────────────────
def create_managed_csv(log_dir: Path):
    try:
        log_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        suffixes = [""] + [f"_{i:03d}" for i in range(2, 1000)]
        for suffix in suffixes:
            path = log_dir / f"highrate_{stamp}{suffix}.csv"
            try:
                csv_file = path.open("x", encoding="utf-8", newline="")
            except FileExistsError:
                continue
            writer = csv.writer(csv_file)
            writer.writerow(CSV_HEADER)
            csv_file.flush()
            return csv_file, writer, path
        print(f"⚠️  无法创建 CSV（同一秒冲突过多）: {log_dir}", file=sys.stderr)
    except OSError as exc:
        print(f"⚠️  无法创建日志目录 {log_dir}: {exc}（继续只打印终端）", file=sys.stderr)
    return None, None, None


def prune_managed_logs(log_dir: Path, keep: int, current: Path) -> None:
    """Keep the newest `keep` managed logs (including current), delete oldest."""
    keep = max(int(keep), 1)
    try:
        entries = [
            p for p in log_dir.iterdir()
            if p.is_file() and not p.is_symlink()
            and p != current and MANAGED_LOG_RE.match(p.name)
        ]
    except OSError:
        return
    entries.sort(key=lambda p: p.name)
    while len(entries) + 1 > keep:
        victim = entries.pop(0)
        try:
            victim.unlink()
        except FileNotFoundError:
            pass
        except OSError as exc:
            print(f"⚠️  删除旧日志失败 {victim}: {exc}", file=sys.stderr)


# ──────────────────────────────────────────────────────────────────────
# File IPC readers
# ──────────────────────────────────────────────────────────────────────
def read_kv_file(path: Path, previous: Dict[str, str]) -> Dict[str, str]:
    try:
        text = path.read_text(encoding="utf-8")
    except OSError:
        return previous
    values = dict(previous)
    for raw in text.splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line or "=" not in line:
            continue
        key, value = line.split("=", 1)
        values[key.strip()] = value.strip()
    return values


def _float(values: Dict[str, str], key: str, fallback: float = 0.0) -> float:
    try:
        return float(values.get(key, fallback))
    except (TypeError, ValueError):
        return fallback


def _bool(values: Dict[str, str], key: str) -> bool:
    return str(values.get(key, "")).strip().lower() in ("1", "true", "yes", "on", "active")


def read_deploy_state(path: Path, previous: Dict[str, str],
                      max_age_s: float = 2.0) -> Dict[str, str]:
    try:
        mtime = path.stat().st_mtime
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return previous
    return {
        "state": str(data.get("state", "")),
        "fault_reason": str(data.get("fault_reason", "")),
        "stale": (time.time() - mtime) > max_age_s,
    }


# ──────────────────────────────────────────────────────────────────────
# Flags (mirror monitor_status.py where it makes sense)
# ──────────────────────────────────────────────────────────────────────
def build_flags(now: float, latest: Dict[str, dict], calib: Dict[str, dict],
                state: str, imu_age_ms: Optional[float], open_buses: set,
                grace_s: float, args) -> list:
    flags = []
    if imu_age_ms is not None and imu_age_ms >= 0.0 and imu_age_ms > args.imu_stale_ms:
        flags.append("IMU_STALE")

    active = state in ACTIVE_STATES
    for name, _bus, _mid in JOINTS:
        item = latest[name]
        if item["fault"]:
            flags.append(f"FAULT:{name}")

        if item["status_t"] is not None:
            age_ms = (now - item["status_t"]) * 1000.0
            if age_ms > args.status_stale_ms:
                flags.append(f"STALE_STATUS:{name}")
            if age_ms > args.max_gap_ms:
                flags.append(f"GAP:{name}")
        elif item["pos_t"] is None and JOINT_BUS[name] in open_buses:
            flags.append(f"NOFDBK:{name}")

        if item["target_t"] is not None:
            target_age_ms = (now - item["target_t"]) * 1000.0
            if target_age_ms > args.target_stale_ms:
                flags.append(f"TGT_STALE:{name}")
        elif now >= grace_s:
            flags.append(f"NO_TARGET:{name}")

        if item["dq"] is not None and abs(item["dq"]) > args.dq_warn:
            flags.append(f"DQ:{name}")
        if active and item["q"] is not None:
            cfg = calib[name]
            if item["target"] is not None and abs(item["q"] - item["target"]) > args.tracking_warn:
                flags.append(f"TRACK:{name}")
            if item["q"] < cfg["lower"] - args.out_margin or item["q"] > cfg["upper"] + args.out_margin:
                flags.append(f"OUT:{name}")
    return flags


# ──────────────────────────────────────────────────────────────────────
# Recorder
# ──────────────────────────────────────────────────────────────────────
def open_can(interface: str) -> socket.socket:
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.setblocking(False)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
    except OSError:
        pass
    sock.bind((interface,))
    return sock


def blank_joint() -> dict:
    return {
        "q": None, "dq": None, "target": None, "tau": None, "temp": None,
        "fault": 0, "mode": 0,
        "status_t": None, "pos_t": None, "vel_t": None,
        "target_t": None,
    }


def frame_value(value: Optional[float], precision: int = 6) -> str:
    return "" if value is None else f"{value:.{precision}f}"


def age_value(timestamp: Optional[float], now: float) -> str:
    if timestamp is None:
        return ""
    return str(int(round(max(0.0, (now - timestamp) * 1000.0))))


def run(args) -> int:
    calib = load_calibration(args.config)
    open_buses = set(args.interfaces)

    socks: Dict[str, socket.socket] = {}
    for interface in args.interfaces:
        try:
            socks[interface] = open_can(interface)
        except OSError as exc:
            print(f"✗ 打开 {interface} 失败: {exc}", file=sys.stderr)
            for sock in socks.values():
                sock.close()
            print("  CAN raw socket 通常需要 root/CAP_NET_RAW；可尝试 sudo 运行。",
                  file=sys.stderr)
            return 1

    motor_map: Dict[Tuple[str, int], str] = {}
    for name, bus, motor_id in JOINTS:
        if bus in socks:
            motor_map[(bus, motor_id)] = name
    missing_buses = [b for b in ("can0", "can1", "can2", "can3") if b not in socks]
    if missing_buses:
        print(f"⚠️  未打开总线: {' '.join(missing_buses)}（对应关节留空）", file=sys.stderr)

    latest = {name: blank_joint() for name in JOINT_NAMES}
    command_values: Dict[str, str] = {}
    imu_values: Dict[str, str] = {}
    deploy_state: Dict[str, str] = {"state": "", "fault_reason": "", "stale": True}

    csv_file = writer = managed_path = None
    managed = False
    data_rows = 0
    pruned = False
    if args.no_csv:
        pass
    elif args.csv:
        path = Path(args.csv)
        existed = path.exists() and path.stat().st_size > 0
        path.parent.mkdir(parents=True, exist_ok=True)
        csv_file = path.open("a", encoding="utf-8", newline="")
        writer = csv.writer(csv_file)
        if not existed:
            writer.writerow(CSV_HEADER)
            csv_file.flush()
    else:
        csv_file, writer, managed_path = create_managed_csv(Path(args.log_dir))
        managed = csv_file is not None

    period = 1.0 / max(args.rate_hz, 1.0)
    t0 = time.monotonic()
    next_sample = t0 + (args.once_wait_ms / 1000.0 if args.once else 0.0)
    next_summary = t0
    last_flush = t0
    next_status_read = 0.0

    counters = {"control": 0, "status": 0, "read": 0, "error": 0}
    window = {"control": 0, "status": 0, "read": 0, "error": 0}
    warned_no_target = False

    print("OpenDoge high-rate recorder（只读，不发送任何 CAN 帧）")
    print(f"  CAN: {' '.join(args.interfaces)}  rate={args.rate_hz:g} Hz  "
          f"duration={'∞' if args.duration <= 0 else f'{args.duration:g}s'}")
    print(f"  配置: {args.config}")
    if managed_path is not None:
        print(f"  日志: {managed_path}  (keep={args.keep})")
    elif args.csv:
        print(f"  日志: {args.csv}（追加）")
    else:
        print("  日志: 关闭（--no-csv）")

    def handle_frame(bus: str, can_id: int, data: bytes, now: float) -> None:
        decoded = decode_frame(can_id, data)
        if decoded is None:
            return
        kind = decoded["kind"]
        counters[kind] += 1
        window[kind] += 1
        name = motor_map.get((bus, decoded["motor"]))
        if name is None:
            return
        item = latest[name]
        cfg = calib[name]
        if kind == "status":
            item["q"] = to_logical_position(decoded["q"], cfg)
            item["dq"] = to_logical_velocity(decoded["dq"], cfg)
            item["tau"] = decoded["tau"]
            item["temp"] = decoded["temp"]
            item["fault"] = decoded["fault"]
            item["mode"] = decoded["mode"]
            item["status_t"] = now
            item["pos_t"] = now
            item["vel_t"] = now
        elif kind == "control":
            item["target"] = to_logical_position(decoded["q"], cfg)
            item["target_t"] = now
        elif kind == "read":
            if decoded["index"] == IDX_MECH_POS:
                item["q"] = to_logical_position(decoded["value"], cfg)
                item["pos_t"] = now
            else:
                item["dq"] = to_logical_velocity(decoded["value"], cfg)
                item["vel_t"] = now

    def drain(interface: str, sock: socket.socket) -> None:
        while True:
            try:
                packet = sock.recv(16)
            except BlockingIOError:
                return
            except OSError as exc:
                print(f"⚠️  {interface} recv: {exc}", file=sys.stderr)
                return
            if len(packet) < 16:
                continue
            can_id, dlc, payload = struct.unpack("=IB3x8s", packet)
            if can_id & CAN_ERR_FLAG:
                counters["error"] += 1
                window["error"] += 1
                continue
            if can_id & CAN_RTR_FLAG:
                continue
            if not (can_id & CAN_EFF_FLAG):
                continue
            if dlc > 8:
                dlc = 8
            handle_frame(interface, can_id & CAN_EFF_MASK, payload[:dlc], time.monotonic())

    def make_row(now: float) -> list:
        nonlocal next_status_read, deploy_state
        if now >= next_status_read:
            deploy_state = read_deploy_state(Path(args.status), deploy_state,
                                             args.status_max_age)
            next_status_read = now + 0.5

        imu_t = _float(imu_values, "t", 0.0)
        imu_age_ms = max(0.0, (now - imu_t) * 1000.0) if imu_t > 0.0 else None
        state_for_flags = "" if deploy_state.get("stale") else deploy_state.get("state", "")
        flags = build_flags(now, latest, calib, state_for_flags,
                            imu_age_ms, open_buses, t0 + 0.2, args)
        if deploy_state.get("stale"):
            flags.append("STATUS_STALE")

        row = [
            f"{now - t0:.4f}",
            time.strftime("%H:%M:%S") + f".{int((time.time() % 1.0) * 1000):03d}",
            deploy_state.get("state", ""),
            deploy_state.get("fault_reason", ""),
            "|".join(flags),
            f"{_float(command_values, 'vx'):.4f}",
            f"{_float(command_values, 'vy'):.4f}",
            f"{_float(command_values, 'yaw_rate'):.4f}",
            int(_bool(command_values, "active")),
            int(_bool(command_values, "estop")),
            int(_bool(command_values, "position_control")),
            int(_bool(command_values, "rl_inference")),
            int(_bool(command_values, "low_gain_mode")),
            f"{_float(imu_values, 'wx'):.6f}",
            f"{_float(imu_values, 'wy'):.6f}",
            f"{_float(imu_values, 'wz'):.6f}",
            f"{_float(imu_values, 'gx'):.6f}",
            f"{_float(imu_values, 'gy'):.6f}",
            f"{_float(imu_values, 'gz'):.6f}",
            "" if imu_age_ms is None else f"{imu_age_ms:.1f}",
            imu_values.get("gyro_seq", ""),
            imu_values.get("quat_seq", ""),
        ]

        for name in JOINT_NAMES:
            item = latest[name]
            cfg = calib[name]
            tau = item["tau"]
            row.extend([
                frame_value(item["q"]),
                frame_value(item["target"]),
                frame_value(item["dq"]),
                frame_value(tau, 5),
                frame_value(None if tau is None else to_joint_torque(tau, cfg), 5),
                frame_value(item["temp"], 2),
                str(item["fault"]),
                age_value(item["status_t"], now),
                age_value(item["target_t"], now),
            ])
        return row

    try:
        while True:
            now = time.monotonic()
            # Drain often enough that status/target ages stay accurate even
            # when the CSV rate is 100 Hz (CAN frames arrive every ~5 ms).
            timeout = max(0.0, min(next_sample - now, 0.002))
            ready, _, _ = select.select(list(socks.values()), [], [], timeout)
            for interface, sock in socks.items():
                if sock in ready:
                    drain(interface, sock)

            now = time.monotonic()
            if now >= next_sample:
                command_values = read_kv_file(Path(args.command), command_values)
                imu_values = read_kv_file(Path(args.imu), imu_values)
                row = make_row(now)

                if (row[4] and not warned_no_target and "NO_TARGET:" in row[4]
                        and counters["control"] == 0 and now - t0 >= 0.5):
                    warned_no_target = True
                    print("⚠️  没收到 type-1 控制帧：target 列为空。"
                          "检查 deploy 是否在发命令 / CAN loopback 是否开启。",
                          file=sys.stderr)

                if writer is not None:
                    writer.writerow(row)
                    data_rows += 1
                    if not pruned and managed and managed_path is not None:
                        prune_managed_logs(Path(args.log_dir), args.keep, managed_path)
                        pruned = True
                if now - last_flush >= 1.0 and csv_file is not None:
                    csv_file.flush()
                    last_flush = now

                if args.once:
                    break
                next_sample = max(next_sample + period, now)
                if args.duration > 0.0 and now - t0 >= args.duration:
                    break

            now = time.monotonic()
            if now >= next_summary:
                imu_t = _float(imu_values, "t", 0.0)
                imu_age_ms = max(0.0, (now - imu_t) * 1000.0) if imu_t > 0.0 else None
                state_for_flags = ("" if deploy_state.get("stale")
                                   else deploy_state.get("state", ""))
                flags = build_flags(now, latest, calib, state_for_flags,
                                    imu_age_ms, open_buses, t0 + 0.2, args)
                if deploy_state.get("stale"):
                    flags.append("STATUS_STALE")
                flag_counts = Counter(flag.split(":", 1)[0] for flag in flags)
                received = sum(
                    1 for name in JOINT_NAMES
                    if latest[name]["status_t"] is not None or latest[name]["pos_t"] is not None
                )
                flag_text = " ".join(f"{k}:{v}" for k, v in sorted(flag_counts.items()))
                imu_text = "n/a" if imu_age_ms is None else f"{imu_age_ms:.1f}ms"
                print(
                    f"t={now - t0:7.2f}s state={deploy_state.get('state', '?'):12s} "
                    f"t1/s={window['control']:4d} t2/s={window['status']:4d} "
                    f"t17/s={window['read']:3d} err/s={window['error']:2d} "
                    f"joint={received}/12 imu_age={imu_text} "
                    + (f"flags: {flag_text}" if flag_text else "flags: ok")
                )
                for key in window:
                    window[key] = 0
                next_summary = now + 1.0

    except KeyboardInterrupt:
        print("\nCtrl-C，停止记录。")
    finally:
        for sock in socks.values():
            sock.close()
        if csv_file is not None:
            csv_file.flush()
            csv_file.close()
        if managed and managed_path is not None and data_rows == 0:
            try:
                managed_path.unlink()
                print(f"未收到任何样本，已删除空日志 {managed_path}", file=sys.stderr)
            except OSError:
                pass

    total = sum(counters.values())
    print(f"记录结束：{data_rows} 行，CAN 帧 control={counters['control']} "
          f"status={counters['status']} read={counters['read']} error={counters['error']}"
          f"（共 {total}）")
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="OpenDoge 只读高频记录器（旁路 CAN + IMU/command/status 文件）")
    parser.add_argument("--rate-hz", type=float, default=100.0,
                        help="CSV 写行频率 (Hz)，默认 100；200 可用于看 10–20 Hz 振动")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="运行时长 (s)，0 = 直到 Ctrl-C")
    parser.add_argument("--once", action="store_true",
                        help="等待 --once-wait-ms 后只写一行，便于快速验证")
    parser.add_argument("--once-wait-ms", type=float, default=200.0,
                        help="--once 模式下等待 CAN 帧的时间 (ms)")
    parser.add_argument("--interfaces", nargs="+",
                        default=["can0", "can1", "can2", "can3"],
                        help="要监听的 CAN 接口")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG,
                        help="deploy 配置（读 direction/offset/reduction/lower/upper）")
    parser.add_argument("--imu", type=Path, default=DEFAULT_IMU)
    parser.add_argument("--command", type=Path, default=DEFAULT_COMMAND)
    parser.add_argument("--status", type=Path, default=DEFAULT_STATUS)
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--keep", type=int, default=5,
                        help="托管日志保留数量（默认 5，和 monitor_status 一致）")
    parser.add_argument("--csv", type=str, default="",
                        help="显式 CSV 路径：追加写入、不轮转（兼容脚本）")
    parser.add_argument("--no-csv", action="store_true", help="不写 CSV，只打印终端")
    parser.add_argument("--dq-warn", type=float, default=15.0,
                        help="|dq| 警告阈值 (rad/s)，默认 15")
    parser.add_argument("--tracking-warn", type=float, default=0.5,
                        help="|q-target| 警告阈值 (rad)，默认 0.5")
    parser.add_argument("--status-stale-ms", type=float, default=30.0,
                        help="type-2 status 当前 age 超此值标记 STALE_STATUS (ms)")
    parser.add_argument("--max-gap-ms", type=float, default=12.0,
                        help="type-2 status 当前 age 超此值标记 GAP (ms)")
    parser.add_argument("--target-stale-ms", type=float, default=30.0,
                        help="type-1 target 当前 age 超此值标记 TGT_STALE (ms)")
    parser.add_argument("--imu-stale-ms", type=float, default=100.0,
                        help="IMU 带内 t 的 age 超此值标记 IMU_STALE (ms)")
    parser.add_argument("--status-max-age", type=float, default=2.0,
                        help="deploy status JSON 的 mtime 超过此值标记 STATUS_STALE(s)")
    parser.add_argument("--out-margin", type=float, default=0.05,
                        help="超出 joint lower/upper 的容差 (rad)")
    return parser.parse_args()


def main() -> int:
    return run(parse_args())


if __name__ == "__main__":
    sys.exit(main())
