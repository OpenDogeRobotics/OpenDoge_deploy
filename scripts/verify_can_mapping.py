#!/usr/bin/env python3
"""上电前只读校验：can0-can3 与 EL05 电机 ID 的映射。

预期映射（与 deploy/include/opendoge_deploy/types.hpp 一致）：
    can0 -> 1/2/3   (FL 左前)
    can1 -> 4/5/6   (FR 右前)
    can2 -> 7/8/9   (RL 左后)
    can3 -> 10/11/12 (RR 右后)

只发送 COMM_GET_DEVICE_ID / COMM_READ_PARAM，不使能、不置零、不控制电机。
当前硬件为赤乌（CHIWU）4×CAN 原生 SocketCAN（也可用于任何 can0-can3 适配器）。

用法:
    sudo python3 scripts/verify_can_mapping.py
    sudo python3 scripts/verify_can_mapping.py --allow-missing
    sudo python3 scripts/verify_can_mapping.py --channels can0 can1
    python3 scripts/verify_can_mapping.py --interfaces-only   # 不读电机，只查接口
    python3 scripts/verify_can_mapping.py --interfaces-only --expect-driver gs_usb

退出码: 0=全部匹配, 1=映射/通信失败, 2=环境问题（未运行 deploy、权限等）。
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "hardware" / "motor"))

import scan_motors_readonly as scan  # noqa: E402

EXPECTED: dict[str, set[int]] = {
    "can0": {1, 2, 3},
    "can1": {4, 5, 6},
    "can2": {7, 8, 9},
    "can3": {10, 11, 12},
}

LEG = {"can0": "FL", "can1": "FR", "can2": "RL", "can3": "RR"}


def describe_fault(fault: int) -> str:
    if not fault:
        return "0x00000000"
    bits = [b for b in range(32) if fault & (1 << b)]
    return "0x{:08X} (设定位: {})".format(fault, ",".join(str(b) for b in bits))


def discover(channel: str, timeout: float) -> tuple[set[int], str | None]:
    """返回 (发现的 ID 集合, 错误信息)。"""
    state = scan.check_channel_state(channel)
    if state is None:
        return set(), "接口不存在"
    if state in ("DOWN", "ERROR-PASSIVE"):
        return set(), f"接口状态 {state}"

    try:
        sock = scan.open_channel(channel)
    except OSError as exc:
        return set(), f"打开失败: {exc}"

    try:
        scan.drain(sock)
        found = scan.discover_motors(sock, timeout=timeout)
        # 广播可能只收到部分回复；对未发现的预期 ID 逐个补扫，并多轮重试。
        for _attempt in range(3):
            missing = sorted(EXPECTED[channel] - found)
            if not missing:
                break
            for motor_id in missing:
                scan.send(sock, scan.COMM_GET_DEVICE_ID, motor_id, [0] * 8)
                frame = scan.recv(sock, 0.1)
                if frame is not None:
                    can_id, _ = frame
                    comm, data2, _ = scan.parse_ext_id(can_id)
                    if comm == scan.COMM_GET_DEVICE_ID:
                        found.add(data2 & 0xFF)
        return found, None
    finally:
        sock.close()


def describe_interface(channel: str) -> tuple[bool, str, str]:
    """Return (ok, detail, driver) for one native SocketCAN interface."""
    sysfs = Path("/sys/class/net") / channel
    if not sysfs.exists():
        return False, "接口不存在", ""
    try:
        out = subprocess.run(["ip", "-details", "link", "show", channel],
                             capture_output=True, text=True, timeout=2.0)
        text = (out.stdout or "") + (out.stderr or "")
    except Exception as exc:  # noqa: BLE001
        return False, f"ip 失败: {exc}", ""
    m = re.search(r"<([^>]*)>", text)
    flags = set(m.group(1).split(",")) if m else set()
    up = "UP" in flags
    bitrate = "bitrate 1000000" in text
    fd = ("dbitrate" in text) or ("CAN_CTRLMODE_FD" in text)
    drv_link = sysfs / "device" / "driver"
    try:
        driver = os.path.basename(os.path.realpath(str(drv_link))) if drv_link.exists() else ""
    except OSError:
        driver = ""
    problems = []
    if not up:
        problems.append("DOWN")
    if not bitrate:
        problems.append("bitrate!=1000000")
    if fd:
        problems.append("CAN FD enabled")
    if "vcan" in text:
        problems.append("vcan (old DingLab bridge?)")
    return (not problems), ",".join(problems) if problems else "OK", driver


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--channels", nargs="+", default=list(EXPECTED),
                        choices=list(EXPECTED), help="只校验指定通道")
    parser.add_argument("--timeout", type=float, default=1.2,
                        help="广播发现等待时间 (秒)")
    parser.add_argument("--allow-missing", action="store_true",
                        help="允许预期电机暂时缺失（仍拒绝意外 ID 和空总线）")
    parser.add_argument("--interfaces-only", action="store_true",
                        help="只检查 can0-can3 接口/驱动/bitrate，不读电机（无需 root）")
    parser.add_argument("--expect-driver", default=None,
                        help="配合 --interfaces-only：驱动名必须匹配（如 gs_usb）")
    args = parser.parse_args()

    if args.interfaces_only:
        failed = False
        for channel in args.channels:
            ok, detail, driver = describe_interface(channel)
            print(f"[{channel}] {detail}  driver={driver or 'unknown'}")
            if not ok:
                failed = True
            if args.expect_driver and driver != args.expect_driver:
                print(f"  ❌ driver={driver or 'unknown'} != expected {args.expect_driver}")
                failed = True
        return 1 if failed else 0

    if os.geteuid() != 0:
        print("⚠️  需要 root 或 CAP_NET_RAW: sudo python3 scripts/verify_can_mapping.py",
              file=sys.stderr)
        return 2

    if subprocess.run(["pgrep", "-f", "opendoge_deploy"],
                      capture_output=True).returncode == 0:
        print("⚠️  opendoge_deploy 正在运行，请先停止再校验（避免并发控制）",
              file=sys.stderr)
        return 2

    print("=" * 68)
    print("OpenDoge CAN 映射只读校验（不会使能或转动电机）")
    print("=" * 68)

    failed = False
    for channel in args.channels:
        expected = EXPECTED[channel]
        found, error = discover(channel, args.timeout)

        print(f"\n[{channel}] {LEG[channel]}  期望 ID: {sorted(expected)}")
        if error:
            print(f"  ❌ {error}")
            failed = True
            continue
        if not found:
            print("  ❌ 总线上没有电机响应")
            failed = True
            continue

        print(f"  发现: {sorted(found)}")
        sock = scan.open_channel(channel)
        try:
            for motor_id in sorted(found):
                pos = None
                fault = None
                for _ in range(2):
                    pos = scan.read_param_float(sock, motor_id, scan.IDX_MECH_POS, 0.5)
                    if pos is not None:
                        break
                for _ in range(2):
                    fault = scan.read_param_uint32(sock, motor_id, scan.IDX_FAULT_STA, 0.5)
                    if fault is not None:
                        break
                name = scan.MOTOR_NAMES.get(motor_id, f"ID{motor_id}")
                pos_text = f"{pos:+.4f} rad" if pos is not None else "n/a"
                print(f"    ID={motor_id:3d} {name:>10s}  pos={pos_text:>14s}  "
                      f"fault={describe_fault(fault or 0)}")
        finally:
            sock.close()

        unexpected = found - expected
        missing = expected - found
        if unexpected:
            print(f"  ❌ 出现不属于 {channel} 的电机 ID: {sorted(unexpected)}（接口可能接反）")
            failed = True
        if missing:
            if args.allow_missing:
                print(f"  ⚠️  预期电机缺失: {sorted(missing)}（--allow-missing 已放行）")
            else:
                print(f"  ❌ 预期电机缺失: {sorted(missing)}")
                failed = True
        if not unexpected and not missing:
            print("  ✅ 映射正确")

    print("\n" + "=" * 68)
    print("校验通过" if not failed else "校验失败：不要使能电机，先修正接口顺序/接线")
    print("=" * 68)
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
