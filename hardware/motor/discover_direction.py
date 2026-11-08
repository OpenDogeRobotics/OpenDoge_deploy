#!/usr/bin/env python3
"""
OpenDoge 电机方向自动检测 — 轻推关节, 无需掰到站立位。

背景:
  如果重新安装了电机后需要确定物理上电机方向和仿真里是否符合，
  通常可以使用同目录下的watch脚本，但是需要自己肉眼测数据，比较麻烦，这个脚本可以解决这个问题

原理:
  每个关节在 URDF 中有明确定义的正方向。轻推关节朝 URDF 正方向,
  观察 mechPos 变化: mechPos 增大 → direction=+1, 减小 → direction=-1。

用法:
  python3 hardware/motor/discover_direction.py
  python3 hardware/motor/discover_direction.py --leg FL   # 单腿模式

操作 (比掰到站立位简单得多):
  1. 机器人趴地、已标零
  2. 脚本提示"推 [关节名] 向 [方向]"
  3. 轻推 ~5-10° (约 0.1-0.2 rad), 5s 窗口内完成
  4. 脚本自动检测 mechPos 变化方向, 判定 direction
  5. 按 Enter 进下一个关节, 按 q 保存退出
  6. 重复 12 次即可全部完成
"""

import argparse
import json
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))
from hardware.motor.el05_motor_menu import (
    El05Bus, IDX_MECH_POS, DEFAULT_MOTORS, motor_label,
)

LOG_DIR = Path(__file__).resolve().parent / "calibration_logs"

JOINTS = [
    # (name, can, motor_id, urdf_positive_direction_description)
    # 注意 thigh：URDF 轴 (0,1,0)、连杆沿 -Z，q 增大时膝盖向 -X（尾部）
    # 移动，所以 URDF 正方向是“膝盖向后”，不是向前（旧文档写反了，
    # 见 docs/上机问题与代码修改记录.md 修改 54）。
    ("FL_hip_joint",   "can0", 1,   "向外推 (远离身体中线)"),
    ("FL_thigh_joint", "can0", 2,   "向后推 (膝盖往尾部方向)"),
    ("FL_calf_joint",  "can0", 3,   "伸直小腿 (脚远离身体)"),
    ("FR_hip_joint",   "can1", 4,   "向内推 (靠近身体中线)"),
    ("FR_thigh_joint", "can1", 5,   "向后推 (膝盖往尾部方向)"),
    ("FR_calf_joint",  "can1", 6,   "伸直小腿 (脚远离身体)"),
    ("RL_hip_joint",   "can2", 7,   "向外推 (远离身体中线)"),
    ("RL_thigh_joint", "can2", 8,   "向后推 (膝盖往尾部方向)"),
    ("RL_calf_joint",  "can2", 9,   "伸直小腿 (脚远离身体)"),
    ("RR_hip_joint",   "can3", 10,  "向内推 (靠近身体中线)"),
    ("RR_thigh_joint", "can3", 11,  "向后推 (膝盖往尾部方向)"),
    ("RR_calf_joint",  "can3", 12,  "伸直小腿 (脚远离身体)"),
]

LEG_JOINTS = {
    "FL": [0, 1, 2], "FR": [3, 4, 5],
    "RL": [6, 7, 8], "RR": [9, 10, 11],
}


def read_joint(bus: El05Bus, motor_id: int) -> Optional[float]:
    bus.drain()
    return bus.read_param_float(motor_id, IDX_MECH_POS, timeout=0.3)


def detect_direction(bus: El05Bus, motor_id: int, joint_name: str, description: str,
                     settle_s: float = 5.0, threshold: float = 0.03) -> tuple[int, float, list[float]]:
    """
    引导用户轻推关节, 记录 mechPos 轨迹, 判定 direction。

    返回: (direction, max_deviation, trajectory)
    """
    trajectory = []
    print(f"\n{'='*60}")
    print(f"📌 {joint_name}")
    print(f"   👉 {description}")
    print(f"   等待 {settle_s:.0f}s 建立基线 (请勿触碰)...")

    # Baseline
    start = time.monotonic()
    baseline_samples = []
    while time.monotonic() - start < 1.0:
        pos = read_joint(bus, motor_id)
        if pos is not None:
            baseline_samples.append(pos)
            trajectory.append((time.monotonic() - start, pos))
        time.sleep(0.1)

    if not baseline_samples:
        print(f"   ❌ 无响应")
        return 1, 0.0, trajectory

    baseline = np.mean(baseline_samples)
    print(f"   基线 mechPos = {baseline:+.4f} rad")

    # Monitor for deviation
    print(f"   ⏳ 请轻推关节 (不需要很大幅度, ~0.1 rad 即可)...")
    deadline = time.monotonic() + settle_s
    max_dev = 0.0
    max_pos = baseline
    detected = False
    direction = 1

    while time.monotonic() < deadline:
        pos = read_joint(bus, motor_id)
        if pos is not None:
            trajectory.append((time.monotonic() - start, pos))
            dev = pos - baseline
            if abs(dev) > abs(max_dev):
                max_dev = dev
                max_pos = pos
            if abs(dev) > threshold and not detected:
                detected = True
                direction = 1 if dev > 0 else -1
                sign_str = "+" if dev > 0 else "-"
                print(f"   ✅ 检测到! mechPos 变化: {baseline:+.4f} → {pos:+.4f} "
                      f"(Δ={dev:+.4f} rad, direction={'+1' if direction > 0 else '-1'})")
        time.sleep(0.1)

    if not detected:
        # Use max deviation for direction
        if abs(max_dev) > 0.005:
            direction = 1 if max_dev > 0 else -1
            print(f"   ⚠️  变化较小 (Δ={max_dev:+.4f} rad), 判定 direction={'+1' if direction > 0 else '-1'}")
        else:
            print(f"   ❌ 未检测到明显运动 (Δ={max_dev:+.4f} rad < {threshold}), 默认 direction=+1")

    print(f"   基线={baseline:+.4f}  最大偏离={max_pos:+.4f}  direction={direction:+d}")
    return direction, abs(max_dev), trajectory


def main():
    parser = argparse.ArgumentParser(description="OpenDoge 电机方向自动检测")
    parser.add_argument("--leg", choices=["FL", "FR", "RL", "RR"], default=None,
                        help="只处理指定腿")
    args = parser.parse_args()

    if args.leg:
        indices = LEG_JOINTS[args.leg.upper()]
        active_channels = {JOINTS[i][1] for i in indices}
    else:
        indices = list(range(12))
        active_channels = {"can0", "can1", "can2", "can3"}

    # Open buses
    buses = {}
    for ch in sorted(active_channels):
        bus = El05Bus(ch, 0xFD)
        try:
            bus.open()
            bus.drain()
            buses[ch] = bus
        except OSError as e:
            print(f"无法打开 {ch}: {e}")
            return 1

    # Fresh start — no resume
    state_path = LOG_DIR / "directions.json"
    directions = [1] * 12
    tested = [False] * 12
    deviations = [0.0] * 12

    try:
        for i in indices:
            name, ch, mid, desc = JOINTS[i]

            bus = buses[ch]
            direction, max_dev, trajectory = detect_direction(bus, mid, name, desc)

            directions[i] = direction
            deviations[i] = max_dev
            tested[i] = True

            # Save after each joint
            LOG_DIR.mkdir(parents=True, exist_ok=True)
            state_path.write_text(json.dumps({
                "directions": directions,
                "tested": tested,
                "deviations": deviations,
                "timestamp": datetime.now().isoformat(),
            }, indent=2))

            # Wait for user confirmation before next joint
            remaining = sum(1 for j in indices if not tested[j])
            if remaining > 0:
                resp = input(f"\n   📋 剩余 {remaining} 个关节, 按 Enter 继续 (q 退出): ").strip().lower()
                if resp == 'q':
                    print("用户退出, 已保存进度")
                    break

    except KeyboardInterrupt:
        print("\n\n用户中断, 已保存进度")

    finally:
        for bus in buses.values():
            bus.close()

    # ── Results ──
    n_tested = sum(tested)
    print(f"\n{'='*60}")
    print(f"方向检测结果 ({n_tested}/12)")
    print(f"{'='*60}")
    print(f"{'关节':<18s} {'direction':>9s} {'偏离幅度':>10s}")
    print("-" * 42)
    for i in range(12):
        status = "✅" if tested[i] else "⬜"
        name = JOINTS[i][0]
        print(f"  {status} {name:<16s} {directions[i]:>+9d} {deviations[i]:>10.4f} rad")

    if n_tested == 12:
        print(f"\n✅ 全部完成! 结果保存在 {state_path}")
        print(f"   将 direction 值复制到 deploy/configs/opendoge_deploy.conf")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
