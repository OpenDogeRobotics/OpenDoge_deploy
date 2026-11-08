#!/usr/bin/env python3
"""monitor_status.py — 只读监控 deploy 的 JSON 状态，检测关节异常。

数据来源是 opendoge_deploy 每秒写的 /tmp/opendoge_status.json：
  - 不占用 CAN、不需要 root；
  - 可以和 deploy 同时运行。

检查项：
  - q 超出配置的 [lower, upper]（OUT，只在主动控制状态检查）
  - |q - target| 过大（TRACK，只在主动控制状态检查）
  - |dq| 过大（DQ，默认 >15 rad/s）
  - 关节侧力矩 |tau * reduction| 接近 max_torque（TAU）
  - 电机 status fault 位非 0（FAULT，并打印欠压/过温/过流等位名）
  - 反馈过旧（STALE，默认 >80 ms；JSON 需含 age_ms）
  - 该电机没有反馈（NOFDBK）
  - state / can_rx / fault_reason 变化

注意：OUT/TRACK 只在 entering_position/active_pc/active_rl/standing_up/
low_gain_test 这些“真的在跟踪目标”的状态下检查。damping_fault/ready 时
JSON 里的 target 只是默认姿态占位值，拿它和实际 q 比会误报。

用法:
    python3 hardware/motor/monitor_status.py
    # 默认自动写 logs/monitor_status/monitor_status_<时间>.csv，最多保留 5 个
    python3 hardware/motor/monitor_status.py --no-csv          # 不写文件
    python3 hardware/motor/monitor_status.py --keep 10         # 保留 10 个
    python3 hardware/motor/monitor_status.py --log-dir /tmp/monitor
    # 显式路径：保持旧的 append 行为（不轮转、不自动清理）
    python3 hardware/motor/monitor_status.py --csv /tmp/opendoge_monitor.csv
    python3 hardware/motor/monitor_status.py --once

注意：JSON 是 1 Hz；`--rate-hz` 只是更频繁地重复同一个快照，CSV 会跳过内容未变化
的快照。快速瞬态（比如几十 ms 的过流）仍可能只留下最后一次 1 Hz 快照。
"""

from __future__ import annotations

import argparse
import csv
import json
import re
import sys
import time
from pathlib import Path
from typing import Dict, Optional

_SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = _SCRIPT_DIR.parent.parent
DEFAULT_STATUS = Path("/tmp/opendoge_status.json")
DEFAULT_CONFIG = REPO_ROOT / "deploy" / "configs" / "opendoge_deploy.conf"
DEFAULT_LOG_DIR = REPO_ROOT / "logs" / "monitor_status"
# Strict: timestamped managed logs only. Do not match arbitrary CSVs.
MANAGED_LOG_RE = re.compile(r"^monitor_status_\d{8}_\d{6}(_\d{3})?\.csv$")
CSV_HEADER = [
    "t", "state", "joint", "q", "target", "dq", "tau", "tau_j",
    "temp", "fault", "fault_sta", "age_ms", "recv", "flags",
]

JOINT_NAMES = [
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
]

# EL05 *status frame* fault field (CAN-ID bits 21:16, `kStatusFaultMask=0x3F`).
# This layout is NOT the same as the 0x3022 faultSta register used by
# el05_motor_menu.py::FAULT_NAMES. Manual §3.4:
#   bit5 未标定, bit4 堵转过载, bit3 磁编码故障,
#   bit2 过温, bit1 三相电流故障, bit0 欠压.
# So `fault=1` means UNDERVOLTAGE, not overcurrent (that would be 2).
STATUS_FAULT_NAMES = {
    0: "欠压",
    1: "三相电流故障",
    2: "过温",
    3: "磁编码故障",
    4: "堵转过载",
    5: "未标定",
}


# 0x3022 faultSta register layout (manual §3.3.7), different from the status
# frame field above. Keep in sync with el05_motor_menu.py::FAULT_NAMES.
FAULT_STA_NAMES = {
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


def fault_description(fault: int, fault_sta: int = 0) -> str:
    parts = []
    if fault:
        parts.append("status:" + ",".join(
            STATUS_FAULT_NAMES[b] for b in range(6) if fault & (1 << b)))
    if fault_sta:
        parts.append("faultSta:" + ",".join(
            FAULT_STA_NAMES.get(b, f"bit{b}")
            for b in range(32) if fault_sta & (1 << b)))
    return " | ".join(parts) if parts else "无"


def load_limits(path: Path) -> Dict[str, Dict[str, float]]:
    limits = {
        name: {"lower": -12.57, "upper": 12.57, "max_torque": 4.0, "reduction": 1.0}
        for name in JOINT_NAMES
    }
    if not path.exists():
        print(f"⚠️  配置不存在，使用默认限位: {path}")
        return limits
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].strip()
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        parts = key.strip().split(".")
        if len(parts) != 3 or parts[0] != "joint":
            continue
        name, field = parts[1], parts[2]
        if name in limits and field in limits[name]:
            try:
                limits[name][field] = float(value.strip())
            except ValueError:
                pass
    return limits


def load_status(path: Path) -> Optional[dict]:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None


# Only in these states is q actively driven toward `target`. In
# damping_fault/ready/wait_feedback the status `target` is just bookkeeping
# (the default pose), while the motors are actually damping at their measured
# position — comparing q to it would produce meaningless OUT/TRACK alarms.
ACTIVE_STATES = {
    "entering_position", "active_pc", "active_rl",
    "standing_up", "low_gain_test",
}


def check_joint(j: dict, lim: Dict[str, float], dq_warn: float,
                track_warn: float, state: str, stale_ms: float,
                status_stale_ms: float = 30.0, max_gap_ms: float = 12.0,
                min_rx_ratio: float = 0.95) -> list[str]:
    flags: list[str] = []
    q = float(j.get("q", 0.0))
    target = j.get("target")
    dq = float(j.get("dq", 0.0))
    tau_j = float(j.get("tau", 0.0)) * lim["reduction"]
    fault = int(j.get("fault", 0))
    fault_sta = int(j.get("fault_sta", 0))
    recv = bool(j.get("recv", True))
    age_ms = j.get("age_ms")
    if state in ACTIVE_STATES:
        if q < lim["lower"] - 0.05 or q > lim["upper"] + 0.05:
            flags.append("OUT")
        if target is not None and abs(q - float(target)) > track_warn:
            flags.append("TRACK")
    if abs(dq) > dq_warn:
        flags.append("DQ")
    if abs(tau_j) > 0.8 * max(lim["max_torque"], 1e-6):
        flags.append("TAU")
    if fault or fault_sta:
        flags.append("FAULT")
    if age_ms is not None and int(age_ms) >= 0 and int(age_ms) > stale_ms:
        flags.append("STALE")
    # New deploy fields (backward compatible: absent -> skip).
    status_age = j.get("status_age_ms")
    if status_age is not None and int(status_age) >= 0 and int(status_age) > status_stale_ms:
        flags.append("STALE_STATUS")
    # GAP is a *current* condition. The JSON max_gap_ms is a lifetime maximum
    # (includes the startup gap before the first type-2 frame), so using it
    # here would flag forever after a one-off startup gap.
    current_gap_ms = j.get("status_age_ms")
    if current_gap_ms is None:
        current_gap_ms = age_ms
    if current_gap_ms is not None and int(current_gap_ms) >= 0 and int(current_gap_ms) > max_gap_ms:
        flags.append("GAP")
    expected = j.get("expected")
    frames = j.get("status_frames")
    if expected and frames is not None and (float(frames) / float(expected)) < min_rx_ratio:
        flags.append("FB_RATIO")
    if not recv:
        flags.append("NOFDBK")
    return flags


def create_managed_csv(log_dir: Path):
    """Create a new timestamped CSV under log_dir.

    Returns (file, writer, path) or (None, None, None) if the directory/file
    cannot be created. Uses exclusive create so two invocations in the same
    second get distinct names (_002, _003, ...).
    """
    try:
        log_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        suffixes = [""] + [f"_{i:03d}" for i in range(2, 1000)]
        for suffix in suffixes:
            path = log_dir / f"monitor_status_{stamp}{suffix}.csv"
            try:
                csv_file = path.open("x", encoding="utf-8", newline="")
            except FileExistsError:
                continue
            writer = csv.writer(csv_file)
            writer.writerow(CSV_HEADER)
            csv_file.flush()
            return csv_file, writer, path
        print(f"⚠️  无法创建监控 CSV（同一秒冲突过多）: {log_dir}", file=sys.stderr)
    except OSError as exc:
        print(f"⚠️  无法创建监控 CSV 目录 {log_dir}: {exc}（继续只打印终端）", file=sys.stderr)
    return None, None, None


def prune_managed_logs(log_dir: Path, keep: int, current: Path) -> None:
    """Keep the newest `keep` managed logs (including current), delete oldest.

    Only files matching MANAGED_LOG_RE are considered; symlinks and the
    current file are never deleted. Ordered by filename (fixed-width
    timestamps) rather than mtime.
    """
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
            print(f"⚠️  删除旧监控日志失败 {victim}: {exc}", file=sys.stderr)


def fmt(value, width: int, precision: int = 4) -> str:
    if value is None:
        return f"{'n/a':>{width}s}"
    return f"{float(value):+{width}.{precision}f}"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--status", type=Path, default=DEFAULT_STATUS)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--rate-hz", type=float, default=2.0)
    parser.add_argument("--dq-warn", type=float, default=15.0, help="|dq| warning threshold (rad/s)")
    parser.add_argument("--tracking-warn", type=float, default=0.5, help="|q-target| warning threshold (rad)")
    parser.add_argument("--stale-ms", type=float, default=80.0, help="feedback age warning threshold (ms)")
    parser.add_argument("--status-stale-ms", type=float, default=30.0,
                        help="type-2 status frame age warning threshold (ms)")
    parser.add_argument("--max-gap-ms", type=float, default=12.0,
                        help="current type-2 status age warning threshold (ms)")
    parser.add_argument("--min-rx-ratio", type=float, default=0.95,
                        help="status_frames/expected feedback ratio warning threshold")
    parser.add_argument("--imu-stale-ms", type=float, default=100.0,
                        help="IMU age warning threshold (ms; requires new bridge t=/seq=)")
    parser.add_argument("--max-age", type=float, default=3.0,
                        help="status file age before warning that deploy is not running (s)")
    csv_group = parser.add_mutually_exclusive_group()
    csv_group.add_argument(
        "--csv", type=Path, default=None,
        help="显式 CSV 路径：保持旧的 append 行为，不轮转、不自动清理")
    csv_group.add_argument(
        "--no-csv", action="store_true",
        help="不生成 CSV（默认会在 logs/monitor_status 下自动生成）")
    parser.add_argument(
        "--log-dir", type=Path, default=DEFAULT_LOG_DIR,
        help=f"自动 CSV 的托管目录（默认 {DEFAULT_LOG_DIR}）")
    parser.add_argument(
        "--keep", type=int, default=5,
        help="托管 CSV 最多保留几个（默认 5，最小 1）")
    parser.add_argument("--once", action="store_true")
    args = parser.parse_args()

    limits = load_limits(args.config)
    period = 1.0 / max(args.rate_hz, 0.2)
    start_s = time.monotonic()
    last_state = None
    last_fault = None
    stale_file_warned = False
    csv_file = None
    writer = None
    managed = False
    managed_path: Optional[Path] = None
    data_rows = 0
    pruned = False
    last_status_sig = None
    if args.csv is not None:
        # Legacy explicit path: append, no rotation, no automatic cleanup.
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        csv_file = args.csv.open("a", encoding="utf-8", newline="")
        writer = csv.writer(csv_file)
        if csv_file.tell() == 0:
            writer.writerow(CSV_HEADER)
    elif not args.no_csv:
        managed = True
        csv_file, writer, managed_path = create_managed_csv(args.log_dir)

    try:
        while True:
            data = load_status(args.status)
            if data is None:
                print(f"\r⚠️  读不到 {args.status} ...", end="", flush=True)
                if args.once:
                    return 1
                time.sleep(period)
                continue

            try:
                st = args.status.stat()
                file_age_s = time.time() - st.st_mtime
                status_sig = (st.st_mtime_ns, st.st_size)
            except OSError:
                file_age_s = float("inf")
                status_sig = None
            # The source JSON is 1 Hz; skip CSV rows when the snapshot did not
            # change (--rate-hz > 1 would otherwise duplicate every row).
            joints = data.get("joints", [])
            write_csv = (
                writer is not None
                and bool(joints)
                and (status_sig is None or status_sig != last_status_sig)
            )
            file_is_stale = file_age_s > args.max_age
            if file_is_stale and not stale_file_warned:
                print(f"\n⏸  状态文件 {file_age_s:.1f}s 未更新：deploy 可能没在跑。"
                      f"下面是最后一次快照，不能代表电机当前在线状态。")
                stale_file_warned = True
            elif not file_is_stale:
                stale_file_warned = False

            state = data.get("state", "?")
            fault = data.get("fault_reason", "")
            imu = data.get("imu", {})
            if state != last_state or fault != last_fault:
                can_state = ",".join(
                    f"{b.get('name')}:{b.get('state')}"
                    for b in data.get("can_buses", [])
                    if b.get("state") in ("BUS-OFF", "ERROR-PASSIVE", "ERROR-WARNING"))
                print(f"\n[state] {state}  can_rx={data.get('can_rx')}  "
                      f"can_tx={data.get('can_tx')}  err={data.get('can_err')}  "
                      f"late_us={data.get('max_late_us')}  "
                      f"imu_age={data.get('imu_age_ms')}  "
                      + (f"can_state={can_state}  " if can_state else "")
                      + f"fault={fault!r}  file_age={file_age_s:.1f}s")
                last_state, last_fault = state, fault

            print(f"\n t={time.monotonic() - start_s:7.2f}s  state={state}  "
                  f"active={data.get('active_cmd')}  rl={data.get('rl_inference')}  "
                  f"can_rx={data.get('can_rx')}  file_age={file_age_s:.1f}s  "
                  f"imu_age={data.get('imu_age_ms')}  "
                  f"imu(wz={imu.get('wz', 0.0):+.3f}, "
                  f"gz={imu.get('gz', 0.0):+.3f})")
            print("  {:<16s} {:>9s} {:>9s} {:>9s} {:>9s} {:>9s} {:>6s} {:>10s} {:>8s} {:>5s} {}".format(
                "joint", "q", "target", "dq", "tau", "tau_j", "temp",
                "fault/fSta", "age_ms", "recv", "flags"))
            warnings = []
            for j in joints:
                name = j.get("n", "?")
                lim = limits.get(name, limits[JOINT_NAMES[0]])
                flags = check_joint(j, lim, args.dq_warn, args.tracking_warn,
                                    state, args.stale_ms,
                                    args.status_stale_ms, args.max_gap_ms,
                                    args.min_rx_ratio)
                tau_j = float(j.get("tau", 0.0)) * lim["reduction"]
                joint_fault = int(j.get("fault", 0))
                joint_fault_sta = int(j.get("fault_sta", 0))
                fault_cell = f"{joint_fault}"
                if joint_fault_sta:
                    fault_cell += f"/{joint_fault_sta:X}"
                age = j.get("age_ms")
                age_cell = "n/a" if age is None else str(int(age))
                print("  {:<16s} {:>9s} {:>9s} {:>9s} {:>9s} {:>9s} {:>6s} {:>10s} {:>8s} {:>5s} {}".format(
                    name,
                    fmt(j.get("q"), 9),
                    fmt(j.get("target"), 9),
                    fmt(j.get("dq"), 9, 3),
                    fmt(j.get("tau"), 9, 3),
                    fmt(tau_j, 9, 3),
                    f"{float(j.get('temp', 0.0)):6.1f}",
                    fault_cell,
                    age_cell,
                    "yes" if j.get("recv") else "NO",
                    ",".join(flags),
                ))
                if flags:
                    detail = (f" ({fault_description(joint_fault, joint_fault_sta)})"
                              if (joint_fault or joint_fault_sta) else "")
                    warnings.append(f"{name}: {'/'.join(flags)}{detail}")
                if write_csv:
                    writer.writerow([
                        f"{time.monotonic() - start_s:.3f}", state, name,
                        j.get("q"), j.get("target"), j.get("dq"), j.get("tau"),
                        f"{tau_j:.6f}", j.get("temp"), joint_fault,
                        joint_fault_sta, age, j.get("recv"), "|".join(flags),
                    ])
            if write_csv:
                last_status_sig = status_sig
                data_rows += 1
                if not pruned and managed and managed_path is not None:
                    prune_managed_logs(args.log_dir, args.keep, managed_path)
                    pruned = True
            imu_age = data.get("imu_age_ms")
            if imu_age is not None and int(imu_age) >= 0 and int(imu_age) > args.imu_stale_ms:
                print(f"  ⚠️  IMU age {int(imu_age)} ms > {args.imu_stale_ms} ms")
            if warnings:
                marker = "旧快照 " if file_is_stale else ""
                print(f"  ⚠️  {marker}[{state}] " + " | ".join(warnings))
            if writer is not None:
                csv_file.flush()
            if state == "damping_fault":
                print(f"  ❌ damping_fault: {fault}")
                if args.once:
                    return 2
            if args.once:
                return 0
            time.sleep(period)
    except KeyboardInterrupt:
        print("\n停止中...")
    finally:
        if csv_file is not None:
            csv_file.close()
        # A run that never wrote a data row (deploy down, --once with no
        # status, zero joints) must not consume a retention slot.
        if managed and managed_path is not None and data_rows == 0:
            try:
                managed_path.unlink()
            except OSError:
                pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
