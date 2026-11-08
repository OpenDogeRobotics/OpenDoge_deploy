#!/usr/bin/env python3
"""zero_all_motors.py — 在默认站立姿态下给 12 个 EL05 电机置零并写 Flash。

背景：机身拆装/重新标定后，需要把“默认站立姿态”设为机械零点。EL05 的
正确顺序是：运控模式 → COMM_SET_ZERO (0x06) → COMM_SAVE_PARAM (0x16)。
**不能在运控模式和 SET_ZERO 之间发 COMM_STOP (0x04)**：stop 会退出运控模式，
标零会被屏蔽（见 docs/上机问题与代码修改记录.md 修改 11/21）。

安全前提（脚本不能替代）：
  - 机器人已悬挂/可靠支撑，四条腿在默认站立位：
      hip=0.0, thigh=+0.8, calf=-1.6  (固件/URDF 约定)
  - 已用 verify_can_mapping.py 和逐电机 jog/hand-push 确认
    物理腿→CAN 映射和 direction（机身装反过尤其重要）。
  - zero_sta (0x7029) 应为 1 (-π..π)；不一致时先修正并断电重启。

用法:
    # 只读预检（默认）
    sudo python3 hardware/motor/zero_all_motors.py --dry-run

    # 真正置零 + 写 Flash（交互确认）
    sudo python3 hardware/motor/zero_all_motors.py --apply

    # 跳过交互（确认已支撑、姿态正确）
    sudo python3 hardware/motor/zero_all_motors.py --apply --yes

    # 只读检查 0x7029，不是 1 时写入并提示断电
    sudo python3 hardware/motor/zero_all_motors.py --fix-zero-sta

完成后必须：
  1. 给电机 24V 动力电断电重启（保持站立姿态不变）；
  2. sudo python3 hardware/motor/watch_angles.py --once
     期望 raw mechPos≈0，logical=(0,0.8,-1.6)；
  3. 再跑 low-gain / deploy。
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional

_SCRIPT_DIR = Path(__file__).resolve().parent
_REPO_ROOT = _SCRIPT_DIR.parent.parent
sys.path.insert(0, str(_SCRIPT_DIR))

from el05_motor_menu import (  # noqa: E402
    CHANNEL_MOTOR_IDS,
    COMM_SAVE_PARAM,
    COMM_WRITE_PARAM,
    DEFAULT_MOTORS,
    El05Bus,
    IDX_FAULT_STA,
    IDX_MECH_POS,
    IDX_RUN_MODE,
)

DEFAULT_CONFIG = _REPO_ROOT / "deploy" / "configs" / "opendoge_deploy.conf"
IDX_ZERO_STA = 0x7029
STOP_POS_TOL = 0.05  # rad, after SET_ZERO
STOP_POSE_TOL = 0.05  # rad, logical vs default after SET_ZERO

DEFAULT_ANGLES = {"hip": 0.0, "thigh": 0.8, "calf": -1.6}

NAME_BY_ID = {mid: name for name, mid in DEFAULT_MOTORS}


def load_calibration(path: Path) -> dict[str, dict[str, float]]:
    calib: dict[str, dict[str, float]] = {}
    if not path.exists():
        raise SystemExit(f"配置文件不存在: {path}")
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].strip()
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        parts = key.strip().split(".")
        if len(parts) != 3 or parts[0] != "joint":
            continue
        name, field = parts[1], parts[2]
        if field not in ("direction", "offset", "reduction", "lower", "upper"):
            continue
        try:
            calib.setdefault(name, {})[field] = float(value.strip())
        except ValueError:
            pass
    for name, values in calib.items():
        kind = name.split("_")[1] if "_" in name else ""
        if kind in DEFAULT_ANGLES:
            values["default"] = DEFAULT_ANGLES[kind]
    missing = [n for n in NAME_BY_ID.values() if n not in calib]
    if missing:
        raise SystemExit(f"配置缺少关节: {', '.join(sorted(missing))}")
    return calib


def logical_from_motor(motor_pos: float, cal: dict[str, float]) -> float:
    return cal["direction"] * (motor_pos / cal["reduction"] - cal["offset"])


def interface_problems(channel: str) -> str:
    try:
        out = subprocess.run(["ip", "-details", "link", "show", channel],
                             capture_output=True, text=True, timeout=2.0)
        text = (out.stdout or "") + (out.stderr or "")
    except Exception as exc:  # noqa: BLE001
        return f"ip 失败: {exc}"
    m = re.search(r"<([^>]*)>", text)
    flags = set(m.group(1).split(",")) if m else set()
    problems = []
    if "vcan" in text:
        problems.append("vcan（旧的 DingLab bridge 还在？）")
    if "UP" not in flags:
        problems.append("DOWN")
    if "bitrate 1000000" not in text:
        problems.append("bitrate!=1000000")
    if "dbitrate" in text or "CAN_CTRLMODE_FD" in text:
        problems.append("CAN FD enabled")
    return "; ".join(problems)


def deploy_running() -> bool:
    try:
        return subprocess.run(["pgrep", "-f", "opendoge_deploy"],
                              capture_output=True, timeout=2.0).returncode == 0
    except Exception:  # noqa: BLE001
        return False


def old_bridge_running() -> bool:
    try:
        return subprocess.run(["pgrep", "-f", "usb2can_vcan_bridge.py"],
                              capture_output=True, timeout=2.0).returncode == 0
    except Exception:  # noqa: BLE001
        return False


def read_zero_sta(bus: El05Bus, motor_id: int, timeout: float) -> Optional[int]:
    """Best-effort read of 0x7029. The type-17 reply encoding for uint8 is not
    documented; try both uint32 and float and only accept a clean 0/1."""
    values: list[tuple[str, float]] = []
    u = bus.read_param_uint32(motor_id, IDX_ZERO_STA, timeout)
    if u is not None:
        values.append(("u32", float(u)))
    f = bus.read_param_float(motor_id, IDX_ZERO_STA, timeout)
    if f is not None:
        values.append(("float", f))
    for kind, value in values:
        if abs(value - 1.0) < 1e-3:
            return 1
        if abs(value) < 1e-3:
            return 0
    return None


def write_zero_sta(bus: El05Bus, motor_id: int) -> bool:
    # Same uint8-in-data[4] convention as write_run_mode_motion.
    data = [IDX_ZERO_STA & 0xFF, (IDX_ZERO_STA >> 8) & 0xFF, 0, 0, 1, 0, 0, 0]
    bus.send(COMM_WRITE_PARAM, motor_id, data)
    status = bus.wait_status(motor_id, timeout=0.5)
    if status is None:
        return False
    bus.send(COMM_SAVE_PARAM, motor_id, [1, 2, 3, 4, 5, 6, 7, 8])
    bus.wait_status(motor_id, timeout=0.3)
    time.sleep(0.05)
    return read_zero_sta(bus, motor_id, 0.5) == 1


def parse_ids(text: str) -> list[int]:
    return [int(p, 0) for p in text.replace(",", " ").split()]


def build_targets(channels: list[str], ids: Optional[list[int]]) -> list[tuple[str, int]]:
    if ids is None:
        return [(ch, mid) for ch in channels for mid in CHANNEL_MOTOR_IDS[ch]]
    wanted = set(ids)
    targets = []
    for ch in channels:
        for mid in CHANNEL_MOTOR_IDS[ch]:
            if mid in wanted:
                targets.append((ch, mid))
    missing = wanted - {mid for _, mid in targets}
    if missing:
        raise SystemExit(f"这些 ID 不在所选通道里: {sorted(missing)}")
    return targets


def preflight(bus: El05Bus, mid: int, cal: dict[str, float], args) -> dict:
    row = {"id": mid, "name": NAME_BY_ID.get(mid, f"ID{mid}"), "errors": []}
    mech = bus.read_param_float(mid, IDX_MECH_POS, args.timeout)
    if mech is None:
        row["errors"].append("读不到 mechPos")
        return row
    fault = bus.read_param_uint32(mid, IDX_FAULT_STA, args.timeout)
    run_mode = bus.read_param_float(mid, IDX_RUN_MODE, args.timeout)
    zero_sta = read_zero_sta(bus, mid, args.timeout)
    logical = logical_from_motor(mech, cal)
    row.update({
        "mech": mech, "logical": logical, "fault": fault,
        "run_mode": run_mode, "zero_sta": zero_sta,
        "expected": cal.get("default"),
    })
    if fault is None:
        row["errors"].append("读不到 faultSta")
    elif fault != 0:
        row["errors"].append(f"faultSta=0x{fault:08X}（先清故障，不要自动清）")
    if zero_sta == 0:
        row["errors"].append("zero_sta=0（应为 1；见 --fix-zero-sta）")
    elif zero_sta is None:
        row["warnings"] = ["无法可靠读取 zero_sta(0x7029)，请用厂商工具确认=1"]
    expected = cal.get("default")
    if expected is not None and not args.skip_pose_check:
        if abs(logical - expected) > args.pose_tolerance:
            row["errors"].append(
                f"logical={logical:+.3f} 与站立默认 {expected:+.3f} 差 "
                f"{logical - expected:+.3f} > {args.pose_tolerance}；"
                "请确认已到站立位且 direction/offset 正确")
    if row["errors"]:
        row["result"] = "预检失败"
    else:
        row["result"] = "预检通过"
    return row


def apply_one(bus: El05Bus, mid: int, cal: dict[str, float], args) -> dict:
    row = {"id": mid, "name": NAME_BY_ID.get(mid, f"ID{mid}"), "errors": []}
    run_mode = bus.read_param_float(mid, IDX_RUN_MODE, args.timeout)
    if run_mode is not None and int(round(run_mode)) != 0:
        # Leaving PP/CSP: stop once, then write motion mode. NEVER stop again.
        bus.stop(mid)
        time.sleep(0.05)
        bus.write_run_mode_motion(mid)
        time.sleep(0.05)
    elif run_mode is None:
        bus.write_run_mode_motion(mid)
        time.sleep(0.05)

    bus.drain()
    status = bus.set_zero(mid)
    if status is None:
        row["errors"].append("COMM_SET_ZERO 无状态回包，未写 Flash（可重试）")
        row["result"] = "失败"
        return row
    bus.send(COMM_SAVE_PARAM, mid, [1, 2, 3, 4, 5, 6, 7, 8])
    bus.wait_status(mid, timeout=0.3)
    time.sleep(0.05)

    mech = bus.read_param_float(mid, IDX_MECH_POS, args.timeout)
    logical = logical_from_motor(mech, cal) if mech is not None else None
    row.update({"mech": mech, "logical": logical, "expected": cal.get("default")})
    if mech is None:
        row["errors"].append("置零后读不到 mechPos")
    elif abs(mech) > args.pos_tolerance:
        row["errors"].append(f"置零后 |mechPos|={abs(mech):.3f} > {args.pos_tolerance}")
    if logical is not None and cal.get("default") is not None:
        if abs(logical - cal["default"]) > STOP_POSE_TOL:
            row["errors"].append(
                f"置零后 logical={logical:+.3f} != 默认 {cal['default']:+.3f}")
    row["result"] = "OK" if not row["errors"] else "失败"
    return row


def print_rows(rows: list[dict]) -> None:
    print(f"\n{'joint':16s} {'id':>3s} {'mech':>9s} {'logical':>9s} {'default':>8s} "
          f"{'run':>4s} {'zsta':>4s} {'fault':>10s}  result")
    for r in rows:
        mech = f"{r.get('mech'):+.4f}" if r.get("mech") is not None else "n/a"
        logical = f"{r.get('logical'):+.4f}" if r.get("logical") is not None else "n/a"
        default = f"{r.get('expected'):+.3f}" if r.get("expected") is not None else "n/a"
        run = f"{int(round(r['run_mode']))}" if r.get("run_mode") is not None else "?"
        zsta = f"{r.get('zero_sta')}" if r.get("zero_sta") is not None else "?"
        fault = f"0x{r['fault']:08X}" if r.get("fault") is not None else "n/a"
        print(f"{r['name']:16s} {r['id']:3d} {mech:>9s} {logical:>9s} {default:>8s} "
              f"{run:>4s} {zsta:>4s} {fault:>10s}  {r.get('result', '')}")
        for err in r.get("errors", []):
            print(f"    ❌ {err}")
        for warn in r.get("warnings", []):
            print(f"    ⚠️  {warn}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--channels", nargs="+", default=list(CHANNEL_MOTOR_IDS),
                        choices=list(CHANNEL_MOTOR_IDS))
    parser.add_argument("--ids", default=None, help="只处理这些 ID（逗号/空格分隔）")
    parser.add_argument("--apply", action="store_true", help="真正置零并写 Flash")
    parser.add_argument("--dry-run", action="store_true", help="只读预检（默认）")
    parser.add_argument("--yes", action="store_true", help="跳过交互确认")
    parser.add_argument("--fix-zero-sta", action="store_true",
                        help="读 0x7029，不为 1 时写 1+保存，然后要求断电重启")
    parser.add_argument("--skip-pose-check", action="store_true",
                        help="跳过站立姿态/offset 预检（不推荐）")
    parser.add_argument("--pose-tolerance", type=float, default=0.15,
                        help="预检 logical 与默认姿态的容差 (rad)")
    parser.add_argument("--pos-tolerance", type=float, default=STOP_POS_TOL,
                        help="置零后 |mechPos| 容差 (rad)")
    parser.add_argument("--timeout", type=float, default=0.5)
    parser.add_argument("--master-id", type=lambda x: int(x, 0), default=0xFD)
    args = parser.parse_args()
    if args.dry_run:
        args.apply = False
    ids = parse_ids(args.ids) if args.ids else None
    targets = build_targets(args.channels, ids)
    calib = load_calibration(args.config)

    if os.geteuid() != 0:
        print("⚠️  需要 root / CAP_NET_RAW: sudo python3 hardware/motor/zero_all_motors.py",
              file=sys.stderr)
        return 2
    if deploy_running():
        print("❌ opendoge_deploy 正在运行，请先停止再置零（避免并发控制）。",
              file=sys.stderr)
        return 2
    if old_bridge_running():
        print("❌ 检测到旧的 usb2can_vcan_bridge.py 进程；先停掉旧 DingLab 方案。",
              file=sys.stderr)
        return 2

    for ch in args.channels:
        problems = interface_problems(ch)
        if problems:
            print(f"❌ {ch}: {problems}", file=sys.stderr)
            print("   期望：原生 SocketCAN、UP、1 Mbps、无 CAN FD。", file=sys.stderr)
            return 2

    print("=" * 78)
    print("EL05 站立位置零" + ("（APPLY：会写 Flash）" if args.apply else "（dry-run）"))
    print("=" * 78)
    print("安全前提：机器人已悬挂/支撑，四条腿在 hip=0 / thigh=0.8 / calf=-1.6。")
    print("已确认物理腿→CAN 映射与 direction（机身装反过必须先 jog 复核）。")
    if args.apply and not args.yes:
        answer = input("确认继续？输入 YES: ").strip()
        if answer != "YES":
            print("已取消。")
            return 2

    rows: list[dict] = []
    failed = False
    for ch in args.channels:
        channel_targets = [(c, mid) for c, mid in targets if c == ch]
        if not channel_targets:
            continue
        bus = El05Bus(ch, args.master_id)
        try:
            bus.open()
            for _, mid in channel_targets:
                cal = calib[NAME_BY_ID[mid]]
                row = preflight(bus, mid, cal, args)
                if args.fix_zero_sta and row.get("zero_sta") == 0:
                    print(f"  {row['name']} (ID={mid}) zero_sta=0 -> 写 1 并保存 ...")
                    if write_zero_sta(bus, mid):
                        print("    ✅ 已写 1；请给电机断电重启后再运行本脚本置零。")
                        return 0
                    print("    ❌ 写 0x7029 失败/读回不为 1；请用厂商工具处理。",
                          file=sys.stderr)
                    return 1
                if args.apply and not row["errors"]:
                    print(f"  {row['name']} (ID={mid}) 置零中 ...")
                    row = apply_one(bus, mid, cal, args)
                rows.append(row)
                if row["errors"]:
                    failed = True
        finally:
            bus.close()

    print_rows(rows)
    print("\n" + "=" * 78)
    if failed:
        print("❌ 有失败项：不要部署。按上面的错误处理后再重试。")
        return 1
    if args.apply:
        print("✅ 全部置零并保存。下一步：")
        print("  1. 给电机 24V 动力电断电重启（保持站立姿态不变）；")
        print("  2. sudo python3 hardware/motor/watch_angles.py --once")
        print("     期望 raw mechPos≈0，logical=(0,0.8,-1.6)；")
        print("  3. 再跑 bash scripts/start_robot.sh low-gain")
    else:
        print("✅ 预检通过（dry-run，未写入）。确认无误后加 --apply。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
