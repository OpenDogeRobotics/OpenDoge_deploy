#!/usr/bin/env python3
"""
OpenDoge EL05/RobStride 电机交互调测菜单 (SocketCAN)。

协议: RobStride 私有 29-bit 扩展 CAN 帧 (非 CANopen)
硬件: 上位机 → CANable/candleLight (gs_usb) → SocketCAN(can0-3) → EL05 电机
手册: docs/EL05使用说明书2600428.pdf

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
快速使用
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  前置条件: CAN 接口已启动 (sudo ip link set up canX), 电机已上电

  启动菜单:
    python3 hardware/motor/el05_motor_menu.py --channel can3
    python3 hardware/motor/el05_motor_menu.py --channel can3 --master-id 0xfd

  只读操作 (安全, 不发控制指令):
    0: 发现电机    — 扫描总线, 列出所有电机及其位置/速度/模式
    2: 读取位置    — 查询指定电机的 mechPos/mechVel
    3: 监听状态    — 被动接收电机主动上报的反馈帧
    10: 读取故障   — 查询 faultSta 寄存器, 显示故障位含义

  控制操作 (需确认, ⚠️ 谨慎):
    4: 使能  5: 停止  6: 清故障  7: 运控模式
    8: 小幅运动测试  9: 机械置零  11: 修改CAN ID

  新电机接入流程:
    1. 选项 0 发现 — 确认电机 ID (出厂=127)
    2. 选项 11 — 将 ID 从 127 改为标准 ID (如 12)
    3. 选项 0 再次发现 — 确认修改成功
    4. 选项 2 — 验证位置读数正常
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

from __future__ import annotations

import argparse
import math
import select
import socket
import struct
import sys
import time
from dataclasses import dataclass
from typing import Iterable, Optional


CAN_EFF_FLAG = 0x80000000
CAN_EFF_MASK = 0x1FFFFFFF

COMM_GET_DEVICE_ID = 0x00
COMM_CONTROL = 0x01
COMM_STATUS = 0x02
COMM_ENABLE = 0x03
COMM_STOP = 0x04
COMM_SET_ZERO = 0x06
COMM_SET_CAN_ID = 0x07
COMM_READ_PARAM = 0x11
COMM_WRITE_PARAM = 0x12
COMM_FAULT_FEEDBACK = 0x15
COMM_SAVE_PARAM = 0x16

IDX_RUN_MODE = 0x7005
IDX_MECH_POS = 0x7019
IDX_MECH_VEL = 0x701B
IDX_FAULT_STA = 0x3022
RUN_MODE_MOTION = 0

P_MIN = -12.57
P_MAX = 12.57
V_MIN = -50.0
V_MAX = 50.0
T_MIN = -6.0
T_MAX = 6.0
KP_MIN = 0.0
KP_MAX = 500.0
KD_MIN = 0.0
KD_MAX = 5.0

RUN_MODE_NAMES = {0: "运控", 1: "位置", 2: "速度", 3: "电流"}

DEFAULT_MOTORS = [
    ("FL_hip_joint", 1),
    ("FL_thigh_joint", 2),
    ("FL_calf_joint", 3),
    ("FR_hip_joint", 4),
    ("FR_thigh_joint", 5),
    ("FR_calf_joint", 6),
    ("RL_hip_joint", 7),
    ("RL_thigh_joint", 8),
    ("RL_calf_joint", 9),
    ("RR_hip_joint", 10),
    ("RR_thigh_joint", 11),
    ("RR_calf_joint", 12),
]

# 每个 CAN 通道对应的电机 ID (hip/thigh/calf 顺序)
CHANNEL_MOTOR_IDS = {
    "can0": [1, 2, 3],
    "can1": [4, 5, 6],
    "can2": [7, 8, 9],
    "can3": [10, 11, 12],
}

# 0x3022 faultSta bit definitions (manual §3.3.7). NOT the same layout as the
# type-2 status frame's 6-bit fault field (CAN-ID bit21:16), where bit0=欠压,
# bit1=三相电流, bit2=过温, …; keep the two mappings separate.
FAULT_NAMES = {
    0: "过温",
    1: "驱动芯片",
    2: "欠压",
    3: "过压",
    4: "B相过流",
    5: "C相过流",
    7: "编码器未标定",
    8: "硬件识别",
    9: "位置初始化",
    14: "堵转过载",
    16: "A相过流",
}


@dataclass
class MotorStatus:
    motor_id: int
    position: float
    velocity: float
    torque: float
    temperature: float
    fault: int
    mode: int


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def float_to_uint(value: float, low: float, high: float, bits: int = 16) -> int:
    value = clamp(value, low, high)
    span = high - low
    return int((value - low) * ((1 << bits) - 1) / span)


def uint_to_float(value: int, low: float, high: float, bits: int = 16) -> float:
    span = high - low
    return float(value) * span / ((1 << bits) - 1) + low


def build_ext_id(comm_type: int, data2: int, target_id: int) -> int:
    return ((comm_type & 0x1F) << 24) | ((data2 & 0xFFFF) << 8) | (target_id & 0xFF)


def parse_ext_id(can_id: int) -> tuple[int, int, int]:
    raw = can_id & CAN_EFF_MASK
    return (raw >> 24) & 0x1F, (raw >> 8) & 0xFFFF, raw & 0xFF


def pack_frame(can_id: int, data: bytes | bytearray | list[int]) -> bytes:
    payload = bytes(data[:8]).ljust(8, b"\x00")
    return struct.pack("=IB3x8s", can_id | CAN_EFF_FLAG, len(payload), payload)


def unpack_frame(packet: bytes) -> tuple[int, bytes]:
    can_id, can_dlc, payload = struct.unpack("=IB3x8s", packet)
    return can_id, payload[:can_dlc]


def parse_status(can_id: int, data: bytes) -> Optional[MotorStatus]:
    comm_type, data2, _target = parse_ext_id(can_id)
    if comm_type != COMM_STATUS or len(data) < 8:
        return None

    motor_id = data2 & 0xFF
    fault = (data2 >> 8) & 0x3F
    mode = (data2 >> 14) & 0x03

    pos_u = (data[0] << 8) | data[1]
    vel_u = (data[2] << 8) | data[3]
    trq_u = (data[4] << 8) | data[5]
    tmp_u = (data[6] << 8) | data[7]

    return MotorStatus(
        motor_id=motor_id,
        position=uint_to_float(pos_u, P_MIN, P_MAX),
        velocity=uint_to_float(vel_u, V_MIN, V_MAX),
        torque=uint_to_float(trq_u, T_MIN, T_MAX),
        temperature=tmp_u * 0.1,
        fault=fault,
        mode=mode,
    )


class El05Bus:
    def __init__(self, channel: str, master_id: int):
        self.channel = channel
        self.master_id = master_id & 0xFF
        self.sock: Optional[socket.socket] = None

    def open(self) -> None:
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.setblocking(False)
        self.sock.bind((self.channel,))

    def close(self) -> None:
        if self.sock is not None:
            self.sock.close()
            self.sock = None

    def send(self, comm_type: int, motor_id: int, data: Iterable[int] = (), data2: Optional[int] = None) -> None:
        if self.sock is None:
            raise RuntimeError("CAN 套接字未打开")
        if data2 is None:
            data2 = self.master_id
        can_id = build_ext_id(comm_type, data2, motor_id)
        self.sock.send(pack_frame(can_id, list(data)))

    def recv(self, timeout: float = 0.0) -> Optional[tuple[int, bytes]]:
        if self.sock is None:
            raise RuntimeError("CAN 套接字未打开")
        ready, _, _ = select.select([self.sock], [], [], timeout)
        if not ready:
            return None
        packet = self.sock.recv(16)
        if len(packet) < 16:
            return None
        return unpack_frame(packet)

    def drain(self) -> None:
        while self.recv(0.0) is not None:
            pass

    def discover(self, timeout: float = 1.0) -> list[int]:
        """发现总线上的所有电机 (COMM_GET_DEVICE_ID 0x00, 纯只读)。"""
        data = [0] * 8
        self.send(COMM_GET_DEVICE_ID, 0xFF, data)
        found: set[int] = set()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            frame = self.recv(max(0.01, deadline - time.monotonic()))
            if frame is None:
                continue
            can_id, _payload = frame
            comm_type, data2, _target = parse_ext_id(can_id)
            if comm_type == COMM_GET_DEVICE_ID:
                motor_id = data2 & 0xFF
                found.add(motor_id)
        # 广播可能只收到部分回复：多个电机同时应答时，USB2CAN 串口路径
        # 可能只转发其中一帧。不能因为找到一个就认为总线只有这一个；
        # 对未发现的 ID 逐个补扫。
        for mid in range(1, 128):
            if mid in found:
                continue
            self.send(COMM_GET_DEVICE_ID, mid, [0] * 8)
            for _ in range(3):
                frame = self.recv(0.03)
                if frame is None:
                    break
                can_id, _payload = frame
                comm_type, data2, _target = parse_ext_id(can_id)
                if comm_type == COMM_GET_DEVICE_ID:
                    found.add(data2 & 0xFF)
                    if (data2 & 0xFF) == mid:
                        break
        return sorted(found)

    def set_can_id(self, motor_id: int, new_id: int) -> Optional[MotorStatus]:
        """修改电机 CAN ID (通信类型 7)，立即生效。"""
        data = [0] * 8
        self.send(COMM_SET_CAN_ID, motor_id, data, data2=(new_id << 8) | self.master_id)
        return self.wait_status(motor_id, timeout=0.5)

    def wait_status(self, motor_id: int, timeout: float = 0.5) -> Optional[MotorStatus]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            frame = self.recv(max(0.0, deadline - time.monotonic()))
            if frame is None:
                continue
            status = parse_status(*frame)
            if status is not None and status.motor_id == motor_id:
                return status
        return None

    def read_param_float(self, motor_id: int, index: int, timeout: float = 0.5) -> Optional[float]:
        data = [index & 0xFF, (index >> 8) & 0xFF, 0, 0, 0, 0, 0, 0]
        self.send(COMM_READ_PARAM, motor_id, data)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            frame = self.recv(max(0.0, deadline - time.monotonic()))
            if frame is None:
                continue
            can_id, payload = frame
            comm_type, data2, _target = parse_ext_id(can_id)
            resp_motor_id = data2 & 0xFF
            if comm_type != COMM_READ_PARAM or resp_motor_id != motor_id or len(payload) < 8:
                continue
            resp_index = payload[0] | (payload[1] << 8)
            if resp_index != index:
                continue
            return struct.unpack("<f", payload[4:8])[0]
        return None

    def read_param_uint32(self, motor_id: int, index: int, timeout: float = 0.5) -> Optional[int]:
        data = [index & 0xFF, (index >> 8) & 0xFF, 0, 0, 0, 0, 0, 0]
        self.send(COMM_READ_PARAM, motor_id, data)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            frame = self.recv(max(0.0, deadline - time.monotonic()))
            if frame is None:
                continue
            can_id, payload = frame
            comm_type, data2, _target = parse_ext_id(can_id)
            resp_motor_id = data2 & 0xFF
            if comm_type != COMM_READ_PARAM or resp_motor_id != motor_id or len(payload) < 8:
                continue
            resp_index = payload[0] | (payload[1] << 8)
            if resp_index != index:
                continue
            return int.from_bytes(payload[4:8], "little", signed=False)
        return None

    def enable(self, motor_id: int) -> Optional[MotorStatus]:
        self.send(COMM_ENABLE, motor_id, [0] * 8)
        return self.wait_status(motor_id)

    def stop(self, motor_id: int, clear_fault: bool = False) -> Optional[MotorStatus]:
        data = [0] * 8
        data[0] = 1 if clear_fault else 0
        self.send(COMM_STOP, motor_id, data)
        return self.wait_status(motor_id, timeout=0.3)

    def set_zero(self, motor_id: int) -> Optional[MotorStatus]:
        data = [0] * 8
        data[0] = 1
        self.send(COMM_SET_ZERO, motor_id, data)
        return self.wait_status(motor_id, timeout=0.5)

    def write_run_mode_motion(self, motor_id: int) -> Optional[MotorStatus]:
        data = [
            IDX_RUN_MODE & 0xFF,
            (IDX_RUN_MODE >> 8) & 0xFF,
            0, 0,
            RUN_MODE_MOTION,
            0, 0, 0,
        ]
        self.send(COMM_WRITE_PARAM, motor_id, data)
        return self.wait_status(motor_id, timeout=0.5)

    def control_motion(self, motor_id: int, q: float, dq: float, tau: float, kp: float, kd: float) -> None:
        tau_u = float_to_uint(tau, T_MIN, T_MAX)
        q_u = float_to_uint(q, P_MIN, P_MAX)
        dq_u = float_to_uint(dq, V_MIN, V_MAX)
        kp_u = float_to_uint(kp, KP_MIN, KP_MAX)
        kd_u = float_to_uint(kd, KD_MIN, KD_MAX)
        data = [
            (q_u >> 8) & 0xFF, q_u & 0xFF,
            (dq_u >> 8) & 0xFF, dq_u & 0xFF,
            (kp_u >> 8) & 0xFF, kp_u & 0xFF,
            (kd_u >> 8) & 0xFF, kd_u & 0xFF,
        ]
        self.send(COMM_CONTROL, motor_id, data, data2=tau_u)


def motor_label(motors: list[tuple[str, int]], motor_id: int) -> str:
    for name, mid in motors:
        if mid == motor_id:
            return name
    return f"电机_{motor_id}"


def fault_description(fault: int) -> str:
    if fault == 0:
        return "无"
    parts = []
    for bit, name in FAULT_NAMES.items():
        if fault & (1 << bit):
            parts.append(name)
    return ", ".join(parts) if parts else f"0x{fault:02X}"


def print_status(status: Optional[MotorStatus], name: str = "") -> None:
    if status is None:
        print("  无响应")
        return
    prefix = f"{name} " if name else ""
    mode_name = RUN_MODE_NAMES.get(status.mode, str(status.mode))
    print(
        f"  {prefix}ID={status.motor_id:02d}  "
        f"位置={status.position:+.4f} rad  "
        f"速度={status.velocity:+.3f} rad/s  "
        f"力矩={status.torque:+.3f} Nm  "
        f"温度={status.temperature:.1f}°C  "
        f"模式={mode_name}  故障={fault_description(status.fault)}"
    )


def parse_ids(text: str, motors: list[tuple[str, int]]) -> Optional[list[int]]:
    text = text.strip()
    if not text:
        return None
    if text.lower() in {"q", "quit", "back", "返回"}:
        return []
    if text.lower() in {"a", "all", "全部"}:
        return [mid for _name, mid in motors]

    by_name = {name: mid for name, mid in motors}
    result: list[int] = []
    for token in text.replace(",", " ").replace("，", " ").split():
        if token in by_name:
            result.append(by_name[token])
            continue
        try:
            value = int(token, 0)
        except ValueError:
            print(f"  无法识别: {token}")
            return None
        if value < 1 or value > 127:
            print(f"  ID 超出范围 (1-127): {value}")
            return None
        result.append(value)
    return result


def prompt_ids(motors: list[tuple[str, int]]) -> list[int]:
    print("\n电机列表:")
    for idx, (name, motor_id) in enumerate(motors):
        print(f"  {idx:02d}: ID={motor_id:02d}  {name}")
    print("  输入示例: 1 | 1,2,3 | FL_hip_joint | a(全部) | q(返回)")
    while True:
        selected = parse_ids(input("选择电机: "), motors)
        if selected is not None:
            return selected


def confirm(prompt: str) -> Optional[bool]:
    """返回 True=确认, False=取消, None=退出(q)"""
    ans = input(f"{prompt} [y=确认 / n=取消 / q=退出]: ").strip().lower()
    if ans in {"q", "quit", "exit", "退出"}:
        return None
    return ans == "y"


def read_float(prompt: str, default: float, low: float, high: float) -> Optional[float]:
    """读取浮点数, 空输入用默认值, 输入 q 返回 None 表示退出"""
    text = input(f"{prompt} [{default}] (q=退出): ").strip()
    if not text:
        return default
    if text.lower() in {"q", "quit", "exit", "退出"}:
        return None
    try:
        value = float(text)
    except ValueError:
        print(f"  无效输入, 使用默认值 {default}")
        return default
    return clamp(value, low, high)


def prompt_ids_with_discover(bus: El05Bus, motors: list[tuple[str, int]],
                             channel: str = "") -> Optional[list[int]]:
    """先扫描总线, 列出在线电机供选择。返回所选 ID 列表, 空列表=退出。

    ⚠️ 数值输入一律按 **电机 ID** 解释；列表序号用 `#N`（0-based）。
    旧版本把数值先当索引，can0 在线 [1,2,3] 时输入 "1" 会误选 ID 2 (FL_thigh)。
    """
    print("正在扫描总线 (只读)...")
    bus.drain()
    online = bus.discover(timeout=1.5)
    if not online:
        print("  未发现任何电机")
        return None

    # 根据通道推断目标 ID 范围
    channel_targets = {
        "can0": (1, 2, 3), "can1": (4, 5, 6),
        "can2": (7, 8, 9), "can3": (10, 11, 12),
    }
    suggested = channel_targets.get(channel, (1, 2, 3))

    # 名称按全部 12 个标准 ID 映射，避免 --ids 过滤时把 2/3 号误标为“非标准”
    known = {mid: name for name, mid in DEFAULT_MOTORS}
    name_to_id = {name: mid for name, mid in DEFAULT_MOTORS}
    print(f"\n当前总线在线电机:")
    for idx, mid in enumerate(online):
        name = known.get(mid, "")
        id_hint = ""
        if mid not in known:
            # 给非标准ID建议一个目标ID
            unused = [t for t in suggested if t not in online]
            hint_id = unused[0] if unused else suggested[0]
            id_hint = f" → 建议改为 {hint_id}"
        pos = bus.read_param_float(mid, IDX_MECH_POS, timeout=0.3)
        pos_str = f"  位置={pos:+.4f} rad" if pos is not None else ""
        if name:
            label = f"  [#{idx}] ID={mid:3d} {name}"
        else:
            label = f"  [#{idx}] ID={mid:3d} (非标准,{id_hint})"
        print(f"{label}{pos_str}")

    print(f"\n  输入 q 返回上级")
    sel = input("选择电机 (输入 ID 或名称，列表序号用 #N，如 #0): ").strip()
    if sel.lower() in {"q", "quit", "exit", "返回", ""}:
        return []

    # 名称
    if sel in name_to_id:
        mid = name_to_id[sel]
        if mid not in online:
            print(f"  ID {mid} ({sel}) 不在线（在线: {online}）")
            return None
        return [mid]

    # 列表序号: #N（0-based）
    if sel.startswith("#"):
        try:
            idx = int(sel[1:], 0)
        except ValueError:
            print(f"  无效序号: {sel}")
            return None
        if 0 <= idx < len(online):
            return [online[idx]]
        print(f"  序号超出范围 (0-{len(online) - 1}): {sel}")
        return None

    # 数值: 一律按电机 ID 解释
    try:
        mid = int(sel, 0)
    except ValueError:
        print(f"  无效选择: {sel}")
        return None
    if not (1 <= mid <= 127):
        print(f"  ID 超出范围 (1-127): {mid}")
        return None
    if mid not in online:
        print(f"  ID {mid} 不在线（在线: {online}）")
        return None
    return [mid]


# ============================================================
# 菜单命令
# ============================================================

def cmd_discover(bus: El05Bus, motors: list[tuple[str, int]]) -> None:
    """只读: 扫描总线发现所有电机 (COMM_GET_DEVICE_ID, 不控制电机)。"""
    print("正在扫描 CAN 总线 (只读, 不控制电机)...")
    bus.drain()
    found = bus.discover(timeout=1.5)
    if not found:
        print("  未发现任何电机")
        return
    print(f"  发现 {len(found)} 个电机: {found}")
    known = {mid: name for name, mid in motors}
    for mid in found:
        name = known.get(mid, "")
        extra = ""
        if mid not in known:
            extra = " ⚠️ 非标准ID (出厂默认, 需改为 1-12)" if mid == 127 else " ⚠️ 非标准ID"
        label = f"  ID={mid:3d}  {name}" if name else f"  ID={mid:3d}"
        print(f"{label}{extra}")
        pos = bus.read_param_float(mid, IDX_MECH_POS, timeout=0.3)
        if pos is not None:
            vel = bus.read_param_float(mid, IDX_MECH_VEL, timeout=0.3)
            mode = bus.read_param_float(mid, IDX_RUN_MODE, timeout=0.3)
            v_str = f"速度={vel:+.3f} rad/s" if vel is not None else ""
            m_str = f"模式={RUN_MODE_NAMES.get(int(mode), str(mode))}" if mode is not None else ""
            print(f"       位置={pos:+.4f} rad  {v_str}  {m_str}")


def cmd_read_position(bus: El05Bus, motors: list[tuple[str, int]]) -> None:
    """只读: 读取指定电机的机械位置和速度。"""
    ids = prompt_ids(motors)
    if not ids:
        return
    for motor_id in ids:
        bus.drain()
        pos = bus.read_param_float(motor_id, IDX_MECH_POS)
        vel = bus.read_param_float(motor_id, IDX_MECH_VEL)
        name = motor_label(motors, motor_id)
        if pos is None:
            print(f"  {name} ID={motor_id:02d}  无响应")
        else:
            vel_text = f"速度={vel:+.3f} rad/s" if vel is not None else "速度=n/a"
            print(f"  {name} ID={motor_id:02d}  位置={pos:+.4f} rad  {vel_text}")


def cmd_read_fault_status(bus: El05Bus, motors: list[tuple[str, int]]) -> None:
    """只读: 读取故障状态寄存器 (0x3022)。"""
    ids = prompt_ids(motors)
    if not ids:
        return
    for motor_id in ids:
        bus.drain()
        fault_sta = bus.read_param_uint32(motor_id, IDX_FAULT_STA)
        name = motor_label(motors, motor_id)
        if fault_sta is None:
            print(f"  {name} ID={motor_id:02d}  无响应")
        else:
            bits = [b for b in range(32) if fault_sta & (1 << b)]
            desc = ", ".join(FAULT_NAMES.get(b, f"bit{b}") for b in bits) if bits else "无"
            print(f"  {name} ID={motor_id:02d}  faultSta=0x{fault_sta:08X} [{desc}]")


def cmd_monitor(bus: El05Bus, motors: list[tuple[str, int]]) -> None:
    """只读: 被动监听电机主动上报的状态帧。"""
    ids = prompt_ids(motors)
    if not ids:
        return
    print("被动监听状态帧 (仅接收, 不发送任何指令)。按 Enter 停止。")
    count = 0
    selected = set(ids)
    while True:
        ready, _, _ = select.select([sys.stdin], [], [], 0.0)
        if ready:
            sys.stdin.readline()
            break
        frame = bus.recv(timeout=0.2)
        if frame is None:
            continue
        status = parse_status(*frame)
        if status is not None and status.motor_id in selected:
            print(f"[{count:04d}]", end="")
            print_status(status, motor_label(motors, status.motor_id))
            count += 1


def cmd_simple(bus: El05Bus, motors: list[tuple[str, int]], action: str) -> None:
    """控制操作: 使能/停止/清故障/置零/设模式。"""
    labels = {
        "enable": "使能",
        "stop": "停止",
        "clear": "清除故障",
        "zero": "设置机械零位",
        "mode_motion": "设为运控模式",
    }
    warnings = {
        "zero": "⚠️ 设置机械零位 (需运控模式, 标零后自动保存到 Flash)",
        "clear": "⚠️ 将清除所选电机的故障状态。",
        "mode_motion": "⚠️ 将把电机切换到运控模式。",
    }

    ids = prompt_ids(motors)
    if not ids:
        return

    action_label = labels.get(action, action)
    if action in warnings:
        ans = confirm(warnings[action])
        if ans is None or not ans:
            print("  已取消")
            return

    for motor_id in ids:
        bus.drain()
        if action == "enable":
            status = bus.enable(motor_id)
        elif action == "stop":
            status = bus.stop(motor_id, clear_fault=False)
        elif action == "clear":
            status = bus.stop(motor_id, clear_fault=True)
        elif action == "zero":
            # Section 4.2.5: SET_ZERO requires CSP or motion-control mode.
            # DO NOT stop() first — that exits motion mode and blocks zeroing.
            status = bus.set_zero(motor_id)
            if status is not None:
                # COMM_SAVE_PARAM (0x16) to persist zero across power cycles
                bus.send(COMM_SAVE_PARAM, motor_id, [1, 2, 3, 4, 5, 6, 7, 8])
                time.sleep(0.03)
        elif action == "mode_motion":
            bus.stop(motor_id)
            time.sleep(0.05)
            status = bus.write_run_mode_motion(motor_id)
        else:
            raise ValueError(action)
        print(f"  [{action_label}] ", end="")
        print_status(status, motor_label(motors, motor_id))


def cmd_jog(bus: El05Bus, motors: list[tuple[str, int]], channel: str = "") -> None:
    """控制操作: 单电机小幅正弦运动测试 (每步可 q 退出)。"""
    print("\n════════ 小幅运动测试 ════════")

    result = prompt_ids_with_discover(bus, motors, channel)
    if result is None or not result:
        print("  已取消")
        return
    motor_id = result[0]

    amp = read_float("振幅 (rad)", 0.05, 0.0, 0.30)
    if amp is None: print("  已取消"); return
    freq = read_float("频率 (Hz)", 0.25, 0.01, 2.0)
    if freq is None: print("  已取消"); return
    duration = read_float("持续时间 (秒)", 4.0, 0.1, 20.0)
    if duration is None: print("  已取消"); return
    kp = read_float("Kp", 5.0, KP_MIN, KP_MAX)
    if kp is None: print("  已取消"); return
    kd = read_float("Kd", 0.2, KD_MIN, KD_MAX)
    if kd is None: print("  已取消"); return

    ans = confirm("⚠️ 将使能电机并发送运动控制帧!")
    if ans is None or not ans:
        print("  已取消")
        return

    bus.drain()
    bus.stop(motor_id)
    time.sleep(0.05)
    bus.write_run_mode_motion(motor_id)
    time.sleep(0.05)
    status = bus.enable(motor_id)
    print_status(status, motor_label(motors, motor_id))
    if status is None:
        print("  中止: 未收到电机状态")
        return

    center = status.position
    hz = 100.0
    period = 1.0 / hz
    t0 = time.monotonic()
    next_t = t0
    try:
        while time.monotonic() - t0 < duration:
            t = time.monotonic() - t0
            target = center + amp * math.sin(2.0 * math.pi * freq * t)
            bus.control_motion(motor_id, target, 0.0, 0.0, kp, kd)
            if int(t * 10) != int((t - period) * 10):
                status = bus.wait_status(motor_id, timeout=0.0)
                if status is not None:
                    print_status(status, motor_label(motors, motor_id))
            next_t += period
            time.sleep(max(0.0, next_t - time.monotonic()))
    except KeyboardInterrupt:
        print("\n  用户中断")
    finally:
        bus.control_motion(motor_id, center, 0.0, 0.0, kp, kd)
        time.sleep(0.05)
        bus.stop(motor_id)
        print("  已停止")


def cmd_set_can_id(bus: El05Bus, motors: list[tuple[str, int]], channel: str = "") -> None:
    """修改电机 CAN ID — 先扫描总线, 选择在线电机, 改到目标 ID。"""
    print("\n════════ 修改电机 CAN ID ════════")
    print("步骤 1/3: 扫描总线发现在线电机...")

    # 用改进的选择器
    result = prompt_ids_with_discover(bus, motors, channel)
    if result is None:
        return
    if not result:
        print("  已取消")
        return
    old_id = result[0]

    # 确认当前电机
    name = motor_label(motors, old_id)
    print(f"\n步骤 2/3: 当前电机 ID={old_id} ({name})")
    pos = bus.read_param_float(old_id, IDX_MECH_POS, timeout=0.3)
    if pos is not None:
        print(f"  位置: {pos:+.4f} rad")

    # 建议新 ID (基于通道)
    channel_targets = {"can0": (1,2,3), "can1": (4,5,6), "can2": (7,8,9), "can3": (10,11,12)}
    suggested = channel_targets.get(channel, (1, 2, 3))
    # 重新扫描总线, 获取当前实际在线电机 (排除当前电机自身)
    bus.drain()
    online_now = bus.discover(timeout=1.0)
    already_used = [mid for mid in suggested if mid in online_now and mid != old_id]
    available = [t for t in suggested if t not in already_used]
    if available:
        default_new = available[0]
    else:
        # 所有建议 ID 都已被占用, 回退到第一个建议 ID
        print(f"  ⚠️ 通道 {channel} 建议 ID {suggested} 均已在线, 默认使用 {suggested[0]}")
        default_new = suggested[0]

    print(f"  通道 {channel} 建议 ID: {suggested}")
    new_id_str = input(f"  新 CAN ID [{default_new}] (q=退出): ").strip()
    if new_id_str.lower() in {"q", "quit", "exit", "退出", ""}:
        if new_id_str == "":
            new_id = default_new
        else:
            print("  已取消")
            return
    else:
        try:
            new_id = int(new_id_str)
        except ValueError:
            print("  无效 ID, 已取消")
            return
        if new_id < 1 or new_id > 127:
            print("  ID 范围 1-127, 已取消")
            return

    # 最终确认
    print(f"\n步骤 3/3: 确认修改")
    print(f"  通道: {channel}")
    print(f"  当前 ID: {old_id}  ({name})")
    print(f"  新 ID:   {new_id}  ({motor_label(motors, new_id)})")
    print(f"  ⚠️ 立即生效! 旧 ID={old_id} 将不再响应!")
    ans = input(f"  输入 'yes' 确认执行 (其他键取消): ").strip()
    if ans != "yes":
        print("  已取消")
        return

    bus.stop(old_id)
    time.sleep(0.05)
    status = bus.set_can_id(old_id, new_id)

    # 验证
    time.sleep(0.1)
    bus.drain()
    verify_pos = bus.read_param_float(new_id, IDX_MECH_POS, timeout=0.3)
    if verify_pos is not None:
        print(f"\n  ✅ ID 修改成功: {old_id} → {new_id}")
        print(f"  新 ID={new_id} 位置={verify_pos:+.4f} rad")
    else:
        print(f"\n  ⚠️ 无响应 — 请用选项 0 (发现) 扫描确认新 ID={new_id}")


def print_menu(channel: str, master_id: int) -> None:
    print(
        f"\n╔══════════════════════════════════════════════════════╗\n"
        f"║  OpenDoge EL05 电机调测菜单                         ║\n"
        f"║  通道: {channel:<6s}  主机ID: 0x{master_id:02X}                     ║\n"
        f"╠══════════════════════════════════════════════════════╣\n"
        f"║  —— 只读操作 (安全, 不控制电机) ——                   ║\n"
        f"║  0: 发现电机     扫描总线, 识别所有电机              ║\n"
        f"║  1: 电机列表     显示电机映射表                     ║\n"
        f"║  2: 读取位置     读取机械位置和速度                 ║\n"
        f"║  3: 监听状态     被动接收电机上报的状态帧           ║\n"
        f"║ 10: 读取故障     读取故障状态寄存器                 ║\n"
        f"╠══════════════════════════════════════════════════════╣\n"
        f"║  —— 控制操作 (需确认, ⚠️ 危险) ——                    ║\n"
        f"║  4: 使能电机     发送使能命令                       ║\n"
        f"║  5: 停止电机     发送停止命令                       ║\n"
        f"║  6: 清除故障     停止并清除故障标志                 ║\n"
        f"║  7: 运控模式     切换为运控模式 (run_mode=0)        ║\n"
        f"║  8: 小幅运动     单电机正弦运动测试                 ║\n"
        f"║  9: 机械置零     设置当前角度为零位                 ║\n"
        f"║ 11: 修改CAN ID   修改电机 CAN 地址 (立即生效)       ║\n"
        f"╠══════════════════════════════════════════════════════╣\n"
        f"║  q: 退出                                            ║\n"
        f"╚══════════════════════════════════════════════════════╝"
    )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--channel", default="can0", help="SocketCAN 接口, 如 can0")
    parser.add_argument("--master-id", default="0xfd", help="主机 CAN ID, 默认 0xfd")
    parser.add_argument(
        "--ids",
        default=None,
        help="电机 ID 列表 (逗号分隔), 默认根据通道自动选择 (can0→1,2,3 can1→4,5,6 ...)",
    )
    return parser


def _build_motor_list(ids: list[int]) -> list[tuple[str, int]]:
    """根据 ID 列表构建 (名字, ID) 映射, 从 DEFAULT_MOTORS 按 ID 匹配。"""
    name_by_id = {mid: name for name, mid in DEFAULT_MOTORS}
    return [(name_by_id.get(mid, f"电机_{mid}"), mid) for mid in ids]


def main() -> int:
    args = build_arg_parser().parse_args()
    master_id = int(str(args.master_id), 0)
    ids: list[int]
    if args.ids is not None:
        ids = [int(x, 0) for x in args.ids.replace(",", " ").split()]
    else:
        ch = args.channel.lower()
        ids = CHANNEL_MOTOR_IDS.get(ch, [1, 2, 3])
        print(f"自动选择通道 {ch} 的电机 ID: {ids}")
    motors = _build_motor_list(ids)

    bus = El05Bus(args.channel, master_id)
    try:
        bus.open()
    except OSError as exc:
        print(f"无法打开 {args.channel}: {exc}")
        print("请检查: sudo ip link set can0 type can bitrate 1000000 && sudo ip link set up can0")
        return 1

    try:
        while True:
            print_menu(args.channel, master_id)
            choice = input("请选择: ").strip().lower()
            if choice == "0":
                cmd_discover(bus, motors)
            elif choice == "1":
                print("\n电机映射表:")
                for name, motor_id in motors:
                    print(f"  ID={motor_id:02d}  {name}")
            elif choice == "2":
                cmd_read_position(bus, motors)
            elif choice == "3":
                cmd_monitor(bus, motors)
            elif choice == "4":
                cmd_simple(bus, motors, "enable")
            elif choice == "5":
                cmd_simple(bus, motors, "stop")
            elif choice == "6":
                cmd_simple(bus, motors, "clear")
            elif choice == "7":
                cmd_simple(bus, motors, "mode_motion")
            elif choice == "8":
                cmd_jog(bus, motors, args.channel)
            elif choice == "9":
                cmd_simple(bus, motors, "zero")
            elif choice == "10":
                cmd_read_fault_status(bus, motors)
            elif choice == "11":
                cmd_set_can_id(bus, motors, args.channel)
            elif choice in {"q", "quit", "exit", "退出"}:
                print("再见!")
                break
            else:
                print("  无效选项, 请输入 0-11 或 q")
    finally:
        bus.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
