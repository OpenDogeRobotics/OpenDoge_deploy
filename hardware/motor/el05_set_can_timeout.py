#!/usr/bin/env python3
"""el05_set_can_timeout.py — 读写 EL05 的 CAN 超时保护参数 (0x7028 canTimeout)。

现状：暂时不做使用，如果执行bash scripts/start_robot.sh没有成功失能，
可以再执行一次然后ctrl+C效果是一样的

背景：deploy 正常退出时会发 COMM_STOP 失能电机；但如果某个电机的 CAN
链路正在丢包（例如 RL_calf/9 号），3 次 COMM_STOP 可能全部丢失，电机就会
一直保持最后的阻尼指令。把 canTimeout 设为非 0 后，电机在一段时间内收不到
任何 CAN 指令就会自动进入 reset（不再输出力矩），相当于电机侧的“失联保护”。

手册 §3.3.6 / 可读写参数表：
  0x7028 canTimeout, uint32, 默认 0（关闭）
  单位 50 µs：20000 = 1 s，10000 = 0.5 s，4000 = 0.2 s

用法（先停止 deploy，保持电机供电）：
    # 只读，看当前值
    sudo python3 hardware/motor/el05_set_can_timeout.py

    # 只给 9 号电机写 1 s，并保存到 Flash，然后读回验证
    sudo python3 hardware/motor/el05_set_can_timeout.py --ids 9 --value 20000 --apply

    # 恢复默认（关闭）
    sudo python3 hardware/motor/el05_set_can_timeout.py --ids 9 --value 0 --apply

注意：
  - 这是**写电机 Flash** 的参数，--apply 才会写入；默认只读。
  - 写入后如果行为没变，给电机断电重启一次再试。
  - 先只改 9 号验证，确认正常后再考虑 12 个电机统一设置。
"""

from __future__ import annotations

import argparse
import time

from el05_motor_menu import (
    COMM_SAVE_PARAM,
    COMM_WRITE_PARAM,
    DEFAULT_MOTORS,
    El05Bus,
)

IDX_CAN_TIMEOUT = 0x7028
CAN_TIMEOUT_UNIT_S = 50.0e-6

DEFAULT_CHANNELS = {
    "can0": [1, 2, 3],
    "can1": [4, 5, 6],
    "can2": [7, 8, 9],
    "can3": [10, 11, 12],
}


def parse_ids(text: str) -> list[int]:
    ids = []
    for part in text.replace(",", " ").split():
        ids.append(int(part, 0))
    return ids


def write_uint32(bus: El05Bus, motor_id: int, index: int, value: int) -> None:
    data = [
        index & 0xFF,
        (index >> 8) & 0xFF,
        0,
        0,
        value & 0xFF,
        (value >> 8) & 0xFF,
        (value >> 16) & 0xFF,
        (value >> 24) & 0xFF,
    ]
    bus.send(COMM_WRITE_PARAM, motor_id, data)


def save_params(bus: El05Bus, motor_id: int) -> None:
    bus.send(COMM_SAVE_PARAM, motor_id, [1, 2, 3, 4, 5, 6, 7, 8])


def describe(value: int | None) -> str:
    if value is None:
        return "无响应"
    if value == 0:
        return "0（关闭）"
    return f"{value}（≈{value * CAN_TIMEOUT_UNIT_S:.3f} s）"


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Read/write EL05 0x7028 canTimeout (0=off, 20000=1s).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--channel", default=None,
                        help="单条 CAN 通道，如 can2；省略则扫描 can0-can3")
    parser.add_argument("--ids", default=None,
                        help="电机 ID 列表，如 9 或 7,8,9；省略则用通道默认表")
    parser.add_argument("--value", type=lambda x: int(x, 0), default=20000,
                        help="要写入的值（50 µs 单位，20000=1s；0=关闭），默认 20000")
    parser.add_argument("--master-id", type=lambda x: int(x, 0), default=0xFD)
    parser.add_argument("--apply", action="store_true",
                        help="真正写入并保存到 Flash；不加时只读")
    parser.add_argument("--timeout", type=float, default=0.5)
    args = parser.parse_args()

    name_by_id = {mid: name for name, mid in DEFAULT_MOTORS}

    if args.channel:
        channels = {args.channel: parse_ids(args.ids) if args.ids else DEFAULT_CHANNELS.get(args.channel, [])}
    else:
        channels = {ch: (parse_ids(args.ids) if args.ids else ids)
                    for ch, ids in DEFAULT_CHANNELS.items()}

    if not any(channels.values()):
        print("没有要处理的电机（检查 --channel/--ids）")
        return 1

    print(f"canTimeout 目标值: {describe(args.value)}")
    print(f"模式: {'写入并保存' if args.apply else '只读'}")
    print()

    rc = 0
    for channel, ids in channels.items():
        if not ids:
            continue
        bus = El05Bus(channel, args.master_id)
        try:
            bus.open()
        except Exception as exc:  # socket errors / missing interface
            print(f"[{channel}] 打开失败: {exc}")
            rc = 1
            continue
        try:
            for motor_id in ids:
                label = name_by_id.get(motor_id, "?")
                bus.drain()
                before = bus.read_param_uint32(motor_id, IDX_CAN_TIMEOUT,
                                               timeout=args.timeout)
                print(f"[{channel}] {label} ID={motor_id:2d}  当前: {describe(before)}")
                if before is None:
                    rc = 1
                    continue
                if not args.apply:
                    continue
                write_uint32(bus, motor_id, IDX_CAN_TIMEOUT, args.value)
                bus.wait_status(motor_id, timeout=args.timeout)
                time.sleep(0.05)
                save_params(bus, motor_id)
                bus.wait_status(motor_id, timeout=args.timeout)
                time.sleep(0.1)
                after = bus.read_param_uint32(motor_id, IDX_CAN_TIMEOUT,
                                              timeout=args.timeout)
                ok = after == args.value
                print(f"[{channel}] {label} ID={motor_id:2d}  写入后: {describe(after)}"
                      f"  {'✅' if ok else '⚠️ 读回不一致'}")
                if not ok:
                    rc = 1
        finally:
            bus.close()

    if not args.apply:
        print("\n（只读模式；加 --apply 才会写入并保存）")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
