#!/usr/bin/env python3
"""
EL05 只读电机扫描工具 — 不发送任何控制指令 (不使能、不置零、不写参数)。

使用 COMM_GET_DEVICE_ID (0x00) 发现电机 + COMM_READ_PARAM (0x11) 读取参数。
纯只读, 不会驱动电机转动, 可安全使用。

协议: EL05 / RobStride 29-bit 扩展 CAN 帧
手册: docs/EL05使用说明书2600428.pdf

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
快速使用
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  前置条件:
    sudo modprobe gs_usb can can_raw
    sudo ip link set can0 type can bitrate 1000000; sudo ip link set up can0
    # ... can1/can2/can3 同理

  扫描全部四通道标准 ID (1-12):
    python3 hardware/motor/scan_motors_readonly.py

  扫描全部 ID 含出厂默认 (1-127):
    python3 hardware/motor/scan_motors_readonly.py --all-ids

  只扫指定通道/电机:
    python3 hardware/motor/scan_motors_readonly.py --channel can3
    python3 hardware/motor/scan_motors_readonly.py --channel can3 --ids 10,11,12

  扫描后被动监听 3 秒 (捕获实时状态帧):
    python3 hardware/motor/scan_motors_readonly.py --listen 3

  常见问题:
    - 无响应: 检查电机 24V 供电、CAN 终端电阻 (120Ω)、USB Hub 外部供电
    - 出厂 ID=127: 新电机默认 ID, 需用 EDULITE-TOOL 改为 1-12
    - ERROR-PASSIVE: CAN 总线无终端电阻或无设备
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

from __future__ import annotations

import argparse
import socket
import struct
import select
import subprocess
import sys
import time
from typing import Optional

# ============================================================
# EL05 / RobStride 协议常量
# ============================================================
CAN_EFF_FLAG = 0x80000000
CAN_EFF_MASK = 0x1FFFFFFF

COMM_GET_DEVICE_ID = 0x00
COMM_STATUS = 0x02
COMM_READ_PARAM = 0x11

IDX_RUN_MODE = 0x7005
IDX_MECH_POS = 0x7019
IDX_MECH_VEL = 0x701B
IDX_FAULT_STA = 0x3022

P_MIN, P_MAX = -12.57, 12.57
V_MIN, V_MAX = -50.0, 50.0
T_MIN, T_MAX = -6.0, 6.0

MASTER_ID = 0xFD
FACTORY_DEFAULT_ID = 127  # 新电机出厂 CAN ID

# OpenDoge 标准电机映射
MOTOR_NAMES = {
    1: "FL_hip", 2: "FL_thigh", 3: "FL_calf",
    4: "FR_hip", 5: "FR_thigh", 6: "FR_calf",
    7: "RL_hip", 8: "RL_thigh", 9: "RL_calf",
    10: "RR_hip", 11: "RR_thigh", 12: "RR_calf",
}

CHANNEL_LEG = {"can0": "FL(左前)", "can1": "FR(右前)", "can2": "RL(左后)", "can3": "RR(右后)"}

RUN_MODE_NAMES = {0: "运控", 1: "位置", 2: "速度", 3: "电流"}


# ============================================================
# CAN 帧编解码 (与 el05_socketcan.cpp 一致)
# ============================================================
def build_ext_id(comm_type: int, data2: int, target_id: int) -> int:
    return ((comm_type & 0x1F) << 24) | ((data2 & 0xFFFF) << 8) | (target_id & 0xFF)


def parse_ext_id(can_id: int) -> tuple[int, int, int]:
    raw = can_id & CAN_EFF_MASK
    return (raw >> 24) & 0x1F, (raw >> 8) & 0xFFFF, raw & 0xFF


def pack_frame(can_id: int, data: list[int]) -> bytes:
    payload = bytes(data[:8]).ljust(8, b"\x00")
    return struct.pack("=IB3x8s", can_id | CAN_EFF_FLAG, len(payload), payload)


def unpack_frame(packet: bytes) -> tuple[int, bytes]:
    can_id, can_dlc, payload = struct.unpack("=IB3x8s", packet)
    return can_id & CAN_EFF_MASK, payload[:can_dlc]


# ============================================================
# CAN 总线操作
# ============================================================
def check_channel_state(channel: str) -> Optional[str]:
    """检查 CAN 通道状态, 返回状态字符串或 None"""
    try:
        result = subprocess.run(
            ["ip", "-details", "link", "show", channel],
            capture_output=True, text=True
        )
        output = result.stdout
        if "state DOWN" in output:
            return "DOWN"
        if "ERROR-PASSIVE" in output:
            return "ERROR-PASSIVE"
        # vcan (USB2CAN 桥接) 没有真实 bitrate/错误状态，非 DOWN 即视为可用
        if "vcan" in output:
            return "OK"
        if "ERROR-ACTIVE" in output or "state UP" in output:
            return "OK"
    except Exception:
        pass
    return None


def open_channel(channel: str) -> socket.socket:
    """打开 CAN raw socket"""
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind((channel,))
    sock.setblocking(False)
    return sock


def drain(sock: socket.socket) -> None:
    """清空接收缓冲区"""
    while True:
        try:
            sock.recv(16)
        except BlockingIOError:
            break
        except Exception:
            break


def send(sock: socket.socket, comm_type: int, motor_id: int, data: list[int]) -> None:
    """发送 CAN 扩展帧"""
    can_id = build_ext_id(comm_type, MASTER_ID, motor_id)
    sock.send(pack_frame(can_id, data))


def recv(sock: socket.socket, timeout: float = 0.0) -> Optional[tuple[int, bytes]]:
    """非阻塞接收 CAN 帧"""
    ready, _, _ = select.select([sock], [], [], timeout)
    if not ready:
        return None
    try:
        packet = sock.recv(16)
        if len(packet) >= 16:
            return unpack_frame(packet)
    except Exception:
        pass
    return None


# ============================================================
# 电机发现与查询
# ============================================================
def discover_motors(sock: socket.socket, timeout: float = 1.0) -> set[int]:
    """
    用 COMM_GET_DEVICE_ID (0x00) 广播发现总线上的所有电机。
    这是纯只读操作, 不修改电机状态。
    """
    found = set()

    # 广播
    send(sock, COMM_GET_DEVICE_ID, 0xFF, [0] * 8)

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        frame = recv(sock, max(0.01, deadline - time.monotonic()))
        if frame is None:
            continue
        can_id, _payload = frame
        comm_type, data2, _ = parse_ext_id(can_id)
        if comm_type == COMM_GET_DEVICE_ID:
            motor_id = data2 & 0xFF
            found.add(motor_id)

    return found


def read_param_float(sock: socket.socket, motor_id: int, index: int,
                     timeout: float = 0.3) -> Optional[float]:
    """读取 float 类型参数 (COMM_READ_PARAM 0x11)"""
    data = [index & 0xFF, (index >> 8) & 0xFF, 0, 0, 0, 0, 0, 0]
    send(sock, COMM_READ_PARAM, motor_id, data)

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        frame = recv(sock, max(0.0, deadline - time.monotonic()))
        if frame is None:
            break
        can_id, payload = frame
        comm_type, data2, _ = parse_ext_id(can_id)
        resp_motor_id = data2 & 0xFF
        if comm_type != COMM_READ_PARAM or resp_motor_id != motor_id or len(payload) < 8:
            continue
        resp_index = payload[0] | (payload[1] << 8)
        if resp_index != index:
            continue
        return struct.unpack("<f", payload[4:8])[0]
    return None


def read_param_uint32(sock: socket.socket, motor_id: int, index: int,
                      timeout: float = 0.3) -> Optional[int]:
    """读取 uint32 类型参数"""
    data = [index & 0xFF, (index >> 8) & 0xFF, 0, 0, 0, 0, 0, 0]
    send(sock, COMM_READ_PARAM, motor_id, data)

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        frame = recv(sock, max(0.0, deadline - time.monotonic()))
        if frame is None:
            break
        can_id, payload = frame
        comm_type, data2, _ = parse_ext_id(can_id)
        resp_motor_id = data2 & 0xFF
        if comm_type != COMM_READ_PARAM or resp_motor_id != motor_id or len(payload) < 8:
            continue
        resp_index = payload[0] | (payload[1] << 8)
        if resp_index != index:
            continue
        return int.from_bytes(payload[4:8], "little", signed=False)
    return None


def _read_param_retry(fn, *args, retries: int = 2, **kwargs):
    """USB2CAN 串口路径会丢帧；参数读失败时重试。"""
    for _ in range(max(1, retries)):
        value = fn(*args, **kwargs)
        if value is not None:
            return value
    return None


def read_motor_status(sock: socket.socket, motor_id: int, timeout: float = 0.3) -> dict:
    """读取电机完整状态 (只读)"""
    result = {
        "motor_id": motor_id,
        "run_mode": None,
        "position": None,
        "velocity": None,
        "fault_sta": None,
    }

    result["run_mode"] = _read_param_retry(
        read_param_float, sock, motor_id, IDX_RUN_MODE, timeout)
    result["position"] = _read_param_retry(
        read_param_float, sock, motor_id, IDX_MECH_POS, timeout)
    result["velocity"] = _read_param_retry(
        read_param_float, sock, motor_id, IDX_MECH_VEL, timeout)
    result["fault_sta"] = _read_param_retry(
        read_param_uint32, sock, motor_id, IDX_FAULT_STA, timeout)

    return result


def listen_status_frames(sock: socket.socket, target_ids: set[int],
                         duration: float) -> list[dict]:
    """被动监听 COMM_STATUS (0x02) 反馈帧"""
    results = []
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        frame = recv(sock, max(0.0, deadline - time.monotonic()))
        if frame is None:
            break
        can_id, payload = frame
        comm_type, data2, _ = parse_ext_id(can_id)
        if comm_type != COMM_STATUS or len(payload) < 8:
            continue
        motor_id = data2 & 0xFF
        if motor_id not in target_ids:
            continue
        fault = (data2 >> 8) & 0x3F
        mode = (data2 >> 14) & 0x03
        pos_u = (payload[0] << 8) | payload[1]
        vel_u = (payload[2] << 8) | payload[3]
        trq_u = (payload[4] << 8) | payload[5]
        tmp_u = (payload[6] << 8) | payload[7]
        pos = pos_u * (P_MAX - P_MIN) / 65535.0 + P_MIN
        vel = vel_u * (V_MAX - V_MIN) / 65535.0 + V_MIN
        trq = trq_u * (T_MAX - T_MIN) / 65535.0 + T_MIN
        results.append({
            "motor_id": motor_id,
            "position": pos,
            "velocity": vel,
            "torque": trq,
            "temperature": tmp_u * 0.1,
            "fault": fault,
            "mode": mode,
        })
    return results


# ============================================================
# 输出
# ============================================================
def motor_label(motor_id: int) -> str:
    if motor_id in MOTOR_NAMES:
        return MOTOR_NAMES[motor_id]
    if motor_id == FACTORY_DEFAULT_ID:
        return f"ID_{motor_id}(出厂)"
    return f"ID_{motor_id}"


def print_motor(channel: str, motor_id: int, status: dict) -> None:
    """打印单个电机状态"""
    name = motor_label(motor_id)
    pos = status.get("position")
    vel = status.get("velocity")
    mode = status.get("run_mode")
    fault = status.get("fault_sta")

    pos_str = f"{pos:+.4f} rad" if pos is not None else "N/A"
    vel_str = f"{vel:+.3f} rad/s" if vel is not None else "N/A"
    mode_str = RUN_MODE_NAMES.get(int(mode), str(mode)) if mode is not None else "?"
    fault_icon = "⚡" if (fault and fault != 0) else "✅"

    extra = ""
    if motor_id == FACTORY_DEFAULT_ID:
        extra = " ⚠️ 需要改为1-12"
    elif motor_id not in MOTOR_NAMES:
        extra = " ⚠️ 非标准ID"

    print(f"  [{channel}] {name:>14s}  pos={pos_str}  vel={vel_str}  "
          f"mode={mode_str}  fault={fault_icon}{extra}")


# ============================================================
# 主入口
# ============================================================
def main() -> int:
    parser = argparse.ArgumentParser(
        description="EL05 只读电机扫描 — COMM_GET_DEVICE_ID + COMM_READ_PARAM, 不发送控制指令"
    )
    parser.add_argument("--channel", "-c", default=None,
                        help="仅扫描指定通道 (如 can3)")
    parser.add_argument("--all-ids", action="store_true",
                        help="扫描 ID 1-127 (含出厂默认), 默认仅扫描 1-12")
    parser.add_argument("--ids", default=None,
                        help="自定义电机 ID 列表, 逗号分隔")
    parser.add_argument("--timeout", type=float, default=0.3,
                        help="单个参数读取超时 (秒, 默认 0.3)")
    parser.add_argument("--discovery-passes", type=int, default=3,
                        help="逐 ID 补扫轮数 (默认 3；丢帧时多轮补扫)")
    parser.add_argument("--listen", type=float, default=0,
                        help="扫描后额外被动监听 N 秒")
    parser.add_argument("--skip-bad", action="store_true", default=True,
                        help="跳过 ERROR-PASSIVE/DOWN 通道 (默认)")
    args = parser.parse_args()

    channels = [args.channel] if args.channel else ["can3", "can2", "can1", "can0"]

    if args.ids:
        target_ids = [int(x.strip()) for x in args.ids.split(",") if x.strip()]
    elif args.all_ids:
        target_ids = list(range(1, 128))
    else:
        target_ids = list(range(1, 13))

    print("=" * 65)
    print("OpenDoge EL05 只读电机扫描")
    print(f"通道: {channels}  目标 ID: {min(target_ids)}-{max(target_ids)}  超时: {args.timeout}s")
    print("=" * 65)

    all_found = {}
    any_found = False

    for channel in channels:
        state = check_channel_state(channel)
        leg = CHANNEL_LEG.get(channel, channel)

        if state is None:
            print(f"\n[{channel}] {leg}: 不可用")
            continue
        if state in ("DOWN", "ERROR-PASSIVE"):
            print(f"\n[{channel}] {leg}: {state} — 跳过")
            continue

        sock = open_channel(channel)
        try:
            drain(sock)

            # 第一步: 广播发现
            print(f"\n[{channel}] {leg} ({state}) — 广播发现...")
            discovered = discover_motors(sock, timeout=1.5)

            # 第二步: 对广播未发现的 ID 逐个补扫，并重复多轮。
            # 多个电机同时应答时，USB2CAN 串口路径会丢帧，不能因为某一轮
            # 没回复就认为电机不在线；只对仍然缺失的 ID 继续补扫。
            for attempt in range(max(1, args.discovery_passes)):
                missing = [mid for mid in target_ids if mid not in discovered]
                if not missing:
                    break
                if attempt == 0:
                    if not discovered:
                        print(f"  广播无响应, 逐 ID 扫描 {min(target_ids)}-{max(target_ids)}...")
                    else:
                        print(f"  广播只发现 {sorted(discovered)}, 补扫其余 ID ...")
                else:
                    print(f"  第 {attempt + 1} 轮补扫: {sorted(missing)} ...")
                for mid in missing:
                    send(sock, COMM_GET_DEVICE_ID, mid, [0] * 8)
                    frame = recv(sock, 0.1)
                    if frame:
                        can_id, _ = frame
                        comm_type, data2, _ = parse_ext_id(can_id)
                        if comm_type == COMM_GET_DEVICE_ID:
                            discovered.add(data2 & 0xFF)

            if not discovered:
                print(f"  无电机响应")
                continue

            print(f"  发现 {len(discovered)} 个电机: {sorted(discovered)}")

            # 第三步: 读取每个电机的详细状态
            channel_found = {}
            for mid in sorted(discovered):
                drain(sock)
                status = read_motor_status(sock, mid, args.timeout)
                if status["position"] is not None or status["run_mode"] is not None:
                    channel_found[mid] = status
                    print_motor(channel, mid, status)
                else:
                    print(f"  [{channel}] {motor_label(mid):>14s}  发现但无法读取参数")

            # 第四步: 可选被动监听
            if args.listen > 0 and channel_found:
                print(f"\n  ... 被动监听 {args.listen}s ...")
                statuses = listen_status_frames(sock, set(channel_found.keys()), args.listen)
                for s in statuses:
                    name = motor_label(s["motor_id"])
                    print(f"  📡 {name:>14s}  pos={s['position']:+.4f}  "
                          f"vel={s['velocity']:+.3f}  tau={s['torque']:+.3f}  "
                          f"temp={s['temperature']:.1f}°C  "
                          f"mode={RUN_MODE_NAMES.get(s['mode'], str(s['mode']))}")

            all_found[channel] = channel_found
            any_found = True

        finally:
            sock.close()

    # 汇总
    print("\n" + "=" * 65)
    total = sum(len(v) for v in all_found.values())
    print(f"扫描完成: {total} 个电机在线")

    if total > 0:
        # 检查非标准 ID
        nonstandard = []
        for ch, motors in all_found.items():
            for mid in motors:
                if mid not in MOTOR_NAMES:
                    nonstandard.append((ch, mid))
        if nonstandard:
            print(f"\n⚠️  {len(nonstandard)} 个电机使用非标准 ID (出厂默认):")
            for ch, mid in nonstandard:
                print(f"  [{ch}] ID={mid} — 需用 EDULITE-TOOL (Windows) 改为标准 ID")
                if ch in CHANNEL_LEG:
                    # 提示该通道对应的标准 ID
                    ch_ids = {
                        "can0": (1, 2, 3), "can1": (4, 5, 6),
                        "can2": (7, 8, 9), "can3": (10, 11, 12),
                    }
                    print(f"       → 应改为 {ch_ids.get(ch, ())} 之一")
    else:
        print("\n💡 所有通道无电机响应. 请检查:")
        print("   1. 电机 24V 供电是否接通")
        print("   2. CAN 总线终端电阻 (120Ω)")
        print("   3. USB Hub 是否带外部供电")
        print("   4. 使用 --all-ids 扫描完整范围 (1-127)")

    return 0 if any_found else 1


if __name__ == "__main__":
    raise SystemExit(main())
