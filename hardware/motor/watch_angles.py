#!/usr/bin/env python3
"""watch_angles.py — 实时查看 12 个 EL05 电机的原始/逻辑角度。

类似 `watch`，但读的是电机：
  - 只发送 COMM_READ_PARAM (0x11)，**不使能、不运动、不改参数**；
  - 默认逐个电机顺序读（菜单同款最稳路径），失败自动重试；
  - 也可 `--mode batch` 批量读（快，但 USB-CAN 路径可能丢回包）；
  - 可按 deploy 配置把 mechPos 换算成 logical (URDF) 角度，
    用于手掰关节验证 direction / offset / reduction。

关于 n/a:
  EL05 是绝对编码器，上电后 mechPos 立即可读，**不需要转一圈**。
  显示 n/a 只表示这一轮没有收到该电机的 type-17 回包，常见原因：
    - 电机没上电；
    - 对应 canX 没 UP / 驱动没加载；
    - 电机 ID 不是 1-12，或该 ID 不在对应总线上；
    - deploy 正在以高频率占用总线；
    - USB-CAN 适配器丢回包（本脚本默认会重试）。

本脚本只使用原生 SocketCAN canX（CHIWU/CANable/gs_usb），
不需要旧的 vcan bridge。

用法:
    sudo python3 hardware/motor/watch_angles.py                 # 顺序读 + 重试
    sudo python3 hardware/motor/watch_angles.py --once
    sudo python3 hardware/motor/watch_angles.py --probe         # 多轮补扫 + 打印原始回包
    sudo python3 hardware/motor/watch_angles.py --mode batch    # 批量读
    sudo python3 hardware/motor/watch_angles.py --ids 1,2,3
    sudo python3 hardware/motor/watch_angles.py --csv /tmp/angles.csv --rate-hz 10
"""

from __future__ import annotations

import argparse
import json
import os
import struct
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

_SCRIPT_DIR = Path(__file__).resolve().parent
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

from el05_motor_menu import (  # noqa: E402
    CHANNEL_MOTOR_IDS,
    COMM_READ_PARAM,
    DEFAULT_MOTORS,
    IDX_FAULT_STA,
    IDX_MECH_POS,
    IDX_MECH_VEL,
    El05Bus,
    parse_ext_id,
)

REPO_ROOT = _SCRIPT_DIR.parent.parent
DEFAULT_CONFIG = REPO_ROOT / "deploy" / "configs" / "opendoge_deploy.conf"

JOINT_NAMES = [name for name, _mid in DEFAULT_MOTORS]
MOTOR_CHANNEL = {}
for _ch, _ids in CHANNEL_MOTOR_IDS.items():
    for _mid in _ids:
        MOTOR_CHANNEL[_mid] = _ch


def load_calibration(path: Path) -> Dict[str, Dict[str, float]]:
    """Parse joint.<name>.{direction,offset,reduction,lower,upper}."""
    calib = {
        name: {
            "direction": 1.0,
            "offset": 0.0,
            "reduction": 1.0,
            "lower": -12.57,
            "upper": 12.57,
        }
        for name in JOINT_NAMES
    }
    if not path.exists():
        print(f"⚠️  配置不存在，logical 角按 direction=+1/offset=0/reduction=1 计算: {path}")
        return calib
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line or "=" not in line:
            continue
        key, value = line.split("=", 1)
        parts = key.strip().split(".")
        if len(parts) != 3 or parts[0] != "joint":
            continue
        name, field = parts[1], parts[2]
        if name in calib and field in calib[name]:
            try:
                calib[name][field] = float(value.strip())
            except ValueError:
                pass
    return calib


def logical_position(motor_position: float, cal: Dict[str, float]) -> float:
    reduction = cal["reduction"] if cal["reduction"] != 0.0 else 1.0
    return cal["direction"] * (motor_position / reduction - cal["offset"])


def send_read(bus: El05Bus, motor_id: int, index: int) -> None:
    data = [index & 0xFF, (index >> 8) & 0xFF, 0, 0, 0, 0, 0, 0]
    bus.send(COMM_READ_PARAM, motor_id, data)


def decode_reply(frame: Tuple[int, bytes]) -> Optional[Tuple[int, int, float]]:
    """Return (motor_id, index, value) or None."""
    can_id, payload = frame
    comm_type, data2, _ = parse_ext_id(can_id)
    if comm_type != COMM_READ_PARAM or len(payload) < 8:
        return None
    motor_id = data2 & 0xFF
    index = payload[0] | (payload[1] << 8)
    if index == IDX_FAULT_STA:
        value = float(int.from_bytes(payload[4:8], "little", signed=False))
    else:
        value = struct.unpack("<f", payload[4:8])[0]
    return motor_id, index, value


def collect(
    buses: Dict[str, El05Bus],
    wanted: Dict[int, set[int]],
    timeout: float,
) -> Dict[Tuple[int, int], float]:
    """Collect replies for `timeout` seconds. Key = (motor_id, index)."""
    results: Dict[Tuple[int, int], float] = {}
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        got_any = False
        for bus in buses.values():
            frame = bus.recv(0.0)
            if frame is None:
                continue
            got_any = True
            decoded = decode_reply(frame)
            if decoded is None:
                continue
            motor_id, index, value = decoded
            if motor_id not in wanted or index not in wanted[motor_id]:
                continue
            results[(motor_id, index)] = value
        if not got_any:
            time.sleep(0.0005)
    return results


def read_pos_with_retries(
    buses: Dict[str, El05Bus],
    motor_ids: List[int],
    timeout: float,
    retries: int,
    log=None,
) -> Dict[Tuple[int, int], float]:
    """One motor at a time, retry until mechPos arrives or retries exhausted."""
    results: Dict[Tuple[int, int], float] = {}
    for motor_id in motor_ids:
        bus = buses.get(MOTOR_CHANNEL.get(motor_id, ""))
        if bus is None:
            continue
        for attempt in range(max(retries, 1)):
            bus.drain()
            pos = bus.read_param_float(motor_id, IDX_MECH_POS, timeout)
            if pos is not None:
                results[(motor_id, IDX_MECH_POS)] = pos
                break
            if log is not None:
                log(f"    retry {attempt + 1}/{retries}: ID={motor_id} mechPos 无回包")
    return results


def read_all_sequential(
    buses: Dict[str, El05Bus],
    motor_ids: List[int],
    timeout: float,
    retries: int,
    read_fault: bool,
    log=None,
) -> Dict[Tuple[int, int], float]:
    """Sequential position read with retries, then vel/fault for responders."""
    results = read_pos_with_retries(buses, motor_ids, timeout, retries, log)
    for motor_id in motor_ids:
        if (motor_id, IDX_MECH_POS) not in results:
            continue
        bus = buses.get(MOTOR_CHANNEL.get(motor_id, ""))
        if bus is None:
            continue
        vel = bus.read_param_float(motor_id, IDX_MECH_VEL, min(timeout, 0.1))
        if vel is not None:
            results[(motor_id, IDX_MECH_VEL)] = vel
        if read_fault:
            fault = bus.read_param_uint32(motor_id, IDX_FAULT_STA, min(timeout, 0.1))
            if fault is not None:
                results[(motor_id, IDX_FAULT_STA)] = float(fault)
    return results


def read_all_batch(
    buses: Dict[str, El05Bus],
    motor_ids: List[int],
    indices: set[int],
    timeout: float,
    retries: int,
    read_fault: bool,
) -> Dict[Tuple[int, int], float]:
    """Burst all reads, then retry only the motors missing mechPos."""
    results: Dict[Tuple[int, int], float] = {}
    pending = list(motor_ids)
    wanted = {mid: set(indices) for mid in pending}
    for attempt in range(max(retries, 1)):
        for motor_id in pending:
            bus = buses.get(MOTOR_CHANNEL.get(motor_id, ""))
            if bus is None:
                continue
            for index in indices:
                send_read(bus, motor_id, index)
            time.sleep(0.001)  # small gap; USB-CAN adapters dislike back-to-back bursts
        results.update(collect(buses, wanted, timeout))
        pending = [mid for mid in pending if (mid, IDX_MECH_POS) not in results]
        if not pending:
            break
        wanted = {mid: {IDX_MECH_POS} for mid in pending}
    # vel/fault for responders that were not filled by the first pass
    for motor_id in motor_ids:
        if (motor_id, IDX_MECH_POS) not in results:
            continue
        bus = buses.get(MOTOR_CHANNEL.get(motor_id, ""))
        if bus is None:
            continue
        if (motor_id, IDX_MECH_VEL) not in results:
            vel = bus.read_param_float(motor_id, IDX_MECH_VEL, min(timeout, 0.1))
            if vel is not None:
                results[(motor_id, IDX_MECH_VEL)] = vel
        if read_fault and (motor_id, IDX_FAULT_STA) not in results:
            fault = bus.read_param_uint32(motor_id, IDX_FAULT_STA, min(timeout, 0.1))
            if fault is not None:
                results[(motor_id, IDX_FAULT_STA)] = float(fault)
    return results


def probe(
    buses: Dict[str, El05Bus],
    motor_ids: List[int],
    timeout: float,
    retries: int,
    passes: int,
) -> None:
    """Multi-pass, per-motor retry probe; prints raw replies + per-motor summary."""
    print(f"\n[probe] {passes} 轮补扫，每电机最多重试 {retries} 次，"
          f"单次等待 {timeout:.2f}s ...")
    best: Dict[int, float] = {}
    reply_count: Dict[int, int] = {mid: 0 for mid in motor_ids}

    for p in range(max(passes, 1)):
        print(f"\n--- pass {p + 1}/{passes} ---")
        for motor_id in motor_ids:
            bus = buses.get(MOTOR_CHANNEL.get(motor_id, ""))
            if bus is None:
                continue
            for attempt in range(max(retries, 1)):
                bus.drain()
                send_read(bus, motor_id, IDX_MECH_POS)
                deadline = time.monotonic() + timeout
                got = None
                while time.monotonic() < deadline:
                    frame = bus.recv(0.0)
                    if frame is None:
                        time.sleep(0.001)
                        continue
                    can_id, payload = frame
                    comm_type, data2, target = parse_ext_id(can_id)
                    mid = data2 & 0xFF
                    if mid == motor_id:
                        reply_count[motor_id] += 1
                    index = (payload[0] | (payload[1] << 8)) if len(payload) >= 2 else -1
                    decoded = decode_reply(frame)
                    val_s = f"{decoded[2]:+.4f}" if decoded is not None else "n/a"
                    print(f"  [{MOTOR_CHANNEL.get(motor_id, '?')}] comm=0x{comm_type:02X} "
                          f"motor=0x{mid:02X} target=0x{target:02X} "
                          f"index=0x{index:04X} value={val_s} payload={payload.hex()}")
                    if decoded is not None and decoded[0] == motor_id and decoded[1] == IDX_MECH_POS:
                        got = decoded[2]
                        break
                if got is not None:
                    best[motor_id] = got
                    break
                print(f"    retry {attempt + 1}/{retries}: ID={motor_id} 无有效 mechPos")
            if got is not None:
                name = next((n for n, mid in DEFAULT_MOTORS if mid == motor_id), str(motor_id))
                print(f"  ✅ ID={motor_id:3d} {name:<16s} mechPos={got:+.4f}")

    print("\n[probe] 汇总:")
    for motor_id in motor_ids:
        name = next((n for n, mid in DEFAULT_MOTORS if mid == motor_id), str(motor_id))
        if motor_id in best:
            print(f"  ✅ ID={motor_id:3d} {name:<16s} mechPos={best[motor_id]:+.4f}  "
                  f"回包 {reply_count[motor_id]} 帧")
        else:
            print(f"  ❌ ID={motor_id:3d} {name:<16s} 无有效回包（收到 {reply_count[motor_id]} 帧）")
    if not best:
        print("  ⚠️  一个有效回包都没有。检查：电机是否上电、桥接是否运行、"
              "canX 是否 UP、电机 ID 是否为 1-12、deploy 是否在占用总线。")


def check_environment(buses: Dict[str, El05Bus]) -> None:
    print("环境检查:")
    for channel in CHANNEL_MOTOR_IDS:
        try:
            out = subprocess.run(
                ["ip", "-details", "link", "show", channel],
                capture_output=True, text=True, timeout=2.0)
            text = (out.stdout or "") + (out.stderr or "")
            state = "UP" if "state UP" in text else "DOWN/缺失"
            kind = "vcan" if "vcan" in text else "can"
            drv = ""
            try:
                drv = os.path.basename(os.path.realpath(
                    f"/sys/class/net/{channel}/device/driver"))
            except OSError:
                pass
            print(f"  {channel:5s} {kind:4s} {state} driver={drv or 'unknown'}")
        except Exception as exc:  # noqa: BLE001
            print(f"  {channel:5s} 检查失败: {exc}")
    print("  注意: 本脚本只用原生 SocketCAN canX（CHIWU/CANable/gs_usb）。")
    print(f"  已打开 CAN 接口: {', '.join(buses) if buses else '无'}\n")


def deploy_running() -> bool:
    try:
        return subprocess.run(
            ["pgrep", "-f", "opendoge_deploy"],
            capture_output=True, timeout=2.0,
        ).returncode == 0
    except Exception:
        return False


def watch_status(path: Path, once: bool, rate_hz: float) -> int:
    """Read the deploy's JSON snapshot instead of polling CAN.

    The deploy writes /tmp/opendoge_status.json at 1 Hz with per-joint
    q/dq/tau/temp/fault. This costs no CAN bandwidth, so it is safe to use
    while opendoge_deploy is running.
    """
    period = 1.0 / max(rate_hz, 0.2)
    start_s = time.monotonic()
    while True:
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            print(f"⚠️  读取 {path} 失败: {exc}")
            if once:
                return 1
            time.sleep(period)
            continue

        print(f"\n t={time.monotonic() - start_s:7.2f}s  state={data.get('state')}  "
              f"can_rx={data.get('can_rx')}  can_err={data.get('can_err')}  "
              f"fault={data.get('fault_reason', '')}")
        print(f"  {'joint':<16s} {'q':>9s} {'dq':>9s} {'tau':>9s} {'temp':>6s} "
              f"{'fault':>6s} {'recv':>5s}")
        for j in data.get("joints", []):
            print(f"  {j.get('n', ''):<16s} {j.get('q', 0.0):+9.4f} "
                  f"{j.get('dq', 0.0):+9.3f} {j.get('tau', 0.0):+9.3f} "
                  f"{j.get('temp', 0.0):6.1f} {j.get('fault', 0):6d} "
                  f"{str(j.get('recv')):>5s}")
        if once:
            return 0
        time.sleep(period)


def format_row(
    name: str,
    raw: Optional[float],
    logical: Optional[float],
    delta: Optional[float],
    vel: Optional[float],
    fault: Optional[float],
) -> str:
    raw_s = f"{raw:+.4f}" if raw is not None else "  n/a "
    log_s = f"{logical:+.4f}" if logical is not None else "  n/a "
    del_s = f"{delta:+.4f}" if delta is not None else "  n/a "
    vel_s = f"{vel:+.3f}" if vel is not None else "  n/a "
    fault_s = f"{int(fault):08X}" if fault is not None else "  n/a   "
    return f"  {name:<16s} {raw_s:>10s} {log_s:>12s} {del_s:>10s} {vel_s:>10s} {fault_s:>10s}"


def print_header() -> None:
    print(
        f"  {'joint':<16s} {'mechPos':>10s} {'logical':>12s} "
        f"{'Δlogical':>10s} {'mechVel':>10s} {'faultSta':>10s}"
    )
    print("  " + "-" * 74)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG,
                        help=f"deploy 配置 (默认 {DEFAULT_CONFIG})")
    parser.add_argument("--rate-hz", type=float, default=5.0,
                        help="轮询频率 (默认 5 Hz；deploy 运行时建议 2)")
    parser.add_argument("--timeout", type=float, default=0.3,
                        help="单次等待回包的时间 (秒, 默认 0.3)")
    parser.add_argument("--retries", type=int, default=3,
                        help="每个电机无回包时的重试次数 (默认 3)")
    parser.add_argument("--passes", type=int, default=2,
                        help="--probe 的补扫轮数 (默认 2)")
    parser.add_argument("--mode", choices=["sequential", "batch"], default="sequential",
                        help="sequential=逐电机(稳)；batch=批量(快但可能丢包)")
    # Backwards-compatible aliases (older docs/commands used these).
    parser.add_argument("--sequential", action="store_true",
                        help="兼容旧写法，等价于 --mode sequential（默认）")
    parser.add_argument("--batch", action="store_true",
                        help="兼容旧写法，等价于 --mode batch")
    parser.add_argument("--master-id", default="0xfd", help="主机 CAN ID (默认 0xfd)")
    parser.add_argument("--ids", default=None,
                        help="只读指定 ID，逗号分隔，例如 1,2,3")
    parser.add_argument("--once", action="store_true", help="只打印一次后退出")
    parser.add_argument("--probe", action="store_true",
                        help="多轮补扫并打印所有原始回包（排障）")
    parser.add_argument("--csv", type=Path, default=None, help="追加写入 CSV")
    parser.add_argument("--no-fault", action="store_true", help="不读 faultSta")
    parser.add_argument("--no-env-check", action="store_true", help="跳过环境检查")
    parser.add_argument("--from-status", type=Path, default=None,
                        help="读取 deploy 的 JSON 快照（不占 CAN），"
                             "如 /tmp/opendoge_status.json")
    args = parser.parse_args()
    if args.batch:
        args.mode = "batch"
    elif args.sequential:
        args.mode = "sequential"

    # Reading the deploy's JSON snapshot costs no CAN bandwidth and does not
    # need raw-socket privileges.
    if args.from_status is not None:
        return watch_status(args.from_status, args.once, args.rate_hz)

    if os.geteuid() != 0:
        print("⚠️  需要 root 或 CAP_NET_RAW: sudo python3 hardware/motor/watch_angles.py",
              file=sys.stderr)
        return 2

    calib = load_calibration(args.config)
    master_id = int(str(args.master_id), 0)

    if args.ids:
        wanted_ids = {int(x) for x in args.ids.replace(" ", "").split(",") if x}
    else:
        wanted_ids = {mid for _name, mid in DEFAULT_MOTORS}

    buses: Dict[str, El05Bus] = {}
    csv_file = None
    try:
        for channel in CHANNEL_MOTOR_IDS:
            if not any(MOTOR_CHANNEL.get(mid) == channel for mid in wanted_ids):
                continue
            bus = El05Bus(channel, master_id)
            try:
                bus.open()
            except OSError as exc:
                print(f"⚠️  跳过 {channel}: {exc}")
                continue
            bus.drain()
            buses[channel] = bus
        if not buses:
            print("❌ 没有任何可用的 CAN 接口", file=sys.stderr)
            return 1

        motor_ids = [mid for _name, mid in DEFAULT_MOTORS
                     if mid in wanted_ids and MOTOR_CHANNEL.get(mid) in buses]

        if not args.no_env_check:
            check_environment(buses)
            if deploy_running():
                print("⚠️  opendoge_deploy 正在运行：它和本脚本共用 can0-3，"
                      "会加剧丢包。建议改用 "
                      "`--from-status /tmp/opendoge_status.json`，或先停止 deploy。\n")

        indices = {IDX_MECH_POS, IDX_MECH_VEL}
        if not args.no_fault:
            indices.add(IDX_FAULT_STA)

        if args.probe:
            probe(buses, motor_ids, args.timeout, args.retries, args.passes)
            return 0

        baseline: Dict[str, float] = {}
        if args.csv is not None:
            args.csv.parent.mkdir(parents=True, exist_ok=True)
            csv_file = args.csv.open("a", encoding="utf-8")
            if csv_file.tell() == 0:
                header = ["t"]
                for name in JOINT_NAMES:
                    header += [f"{name}.raw", f"{name}.logical"]
                csv_file.write(",".join(header) + "\n")
            print(f"CSV: {args.csv}")

        period = 1.0 / max(args.rate_hz, 0.2)
        start_s = time.monotonic()
        print(f"只读轮询 {len(motor_ids)} 个电机 @ {args.rate_hz:.1f} Hz "
              f"(mode={args.mode}, retries={args.retries})；Ctrl-C 退出")
        print(f"配置: {args.config}")
        print_header()

        while True:
            cycle_start = time.monotonic()
            if args.mode == "batch":
                replied = read_all_batch(
                    buses, motor_ids, indices, args.timeout, args.retries, not args.no_fault)
            else:
                replied = read_all_sequential(
                    buses, motor_ids, args.timeout, args.retries, not args.no_fault)

            rows = []
            csv_row = [f"{time.monotonic() - start_s:.4f}"]
            for name, motor_id in DEFAULT_MOTORS:
                if motor_id not in wanted_ids:
                    rows.append(format_row(name, None, None, None, None, None))
                    csv_row += ["", ""]
                    continue
                raw = replied.get((motor_id, IDX_MECH_POS))
                vel = replied.get((motor_id, IDX_MECH_VEL))
                fault = replied.get((motor_id, IDX_FAULT_STA))
                logical = logical_position(raw, calib[name]) if raw is not None else None
                if logical is not None and name not in baseline:
                    baseline[name] = logical
                delta = (logical - baseline[name]) if logical is not None and name in baseline else None
                rows.append(format_row(name, raw, logical, delta, vel, fault))
                csv_row += [
                    "" if raw is None else f"{raw:.6f}",
                    "" if logical is None else f"{logical:.6f}",
                ]

            pos_count = sum(1 for mid in motor_ids if (mid, IDX_MECH_POS) in replied)
            print(f"\n t={time.monotonic() - start_s:7.2f}s  "
                  f"mechPos replies={pos_count}/{len(motor_ids)}")
            print_header()
            for row in rows:
                print(row)

            if csv_file is not None:
                csv_file.write(",".join(csv_row) + "\n")
                csv_file.flush()

            if args.once:
                break
            sleep_s = period - (time.monotonic() - cycle_start)
            if sleep_s > 0.0:
                time.sleep(sleep_s)

    except KeyboardInterrupt:
        print("\n停止中...")
    finally:
        if csv_file is not None:
            try:
                csv_file.close()
            except Exception:
                pass
        for bus in buses.values():
            bus.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
