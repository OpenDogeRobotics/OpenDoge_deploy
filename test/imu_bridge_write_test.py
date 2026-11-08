#!/usr/bin/env python3
"""dm_imu_bridge write-on-new-frame tests (no IMU hardware).

The bridge used to write imu.state at most every 5 ms even when a new
gyro/quat frame had just arrived. The deploy now refreshes its observation at
the inference tick, so that gate added avoidable latency. This test feeds a
fake reader and checks:

  - no write before both gyro and quat are present;
  - one write per new gyro/quat frame (not one per loop iteration);
  - idle loops do not rewrite the file;
  - the in-band t/gyro_seq/quat_seq advance with the frames.
"""

from __future__ import annotations

import sys
import threading
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "hardware" / "imu"))

import dm_imu_bridge as bridge  # noqa: E402

FAILS = []


def check(label: str, condition: bool) -> None:
    print(f"  {'PASS' if condition else 'FAIL'}  {label}")
    if not condition:
        FAILS.append(label)


class StopRun(Exception):
    pass


class FakeReader:
    """Yield fixed batches for the first calls, then idle, then stop."""

    def __init__(self, batches, stop_after: int = 30):
        self.batches = list(batches)
        self.calls = 0
        self.stop_after = stop_after

    def read_updates(self):
        self.calls += 1
        if self.calls > self.stop_after:
            raise StopRun
        if self.calls <= len(self.batches):
            return self.batches[self.calls - 1]
        return []


def run_bridge(reader, lowpass_hz: float = 0.0):
    writes = []
    original_write = bridge.atomic_write
    original_sleep = bridge.time.sleep
    bridge.atomic_write = lambda _path, text: writes.append(text)
    bridge.time.sleep = lambda _s: None  # keep the test fast
    order, signs = bridge.parse_axis_map("xyz", "1,1,1")

    def worker():
        try:
            bridge.run(reader, "/tmp/unused_imu.state", order, signs,
                       0.1, True, lowpass_hz, write_min_interval_s=0.0)
        except StopRun:
            pass

    thread = threading.Thread(target=worker)
    thread.start()
    thread.join(timeout=2.0)
    bridge.atomic_write = original_write
    bridge.time.sleep = original_sleep
    return writes


def field(text: str, key: str) -> str:
    for line in text.splitlines():
        if line.startswith(key + "="):
            return line.split("=", 1)[1]
    return ""


def main() -> int:
    print("[1] write only on new gyro/quat frames")
    batches = [
        [(0x02, (0.1, 0.2, 0.3))],                 # gyro only: not ready
        [(0x04, (1.0, 0.0, 0.0, 0.0))],            # quat -> first write
        [(0x02, (0.4, 0.5, 0.6))],                 # gyro -> second write
        [(0x04, (0.9, 0.1, 0.0, 0.0))],            # quat -> third write
        [], [], [], [], [], [], [], [], [], [],    # idle: no rewrite
    ]
    writes = run_bridge(FakeReader(batches, stop_after=30))
    check("3 writes for 4 frames after ready", len(writes) == 3)
    if len(writes) >= 3:
        check("first write has gyro/quat seq 1/1",
              field(writes[0], "gyro_seq") == "1" and field(writes[0], "quat_seq") == "1")
        check("second write advances gyro_seq to 2",
              field(writes[1], "gyro_seq") == "2" and field(writes[1], "quat_seq") == "1")
        check("third write advances quat_seq to 2",
              field(writes[2], "gyro_seq") == "2" and field(writes[2], "quat_seq") == "2")
        check("in-band t is populated", float(field(writes[0], "t")) > 0.0)

    print("[2] lowpass uses the measured frame interval")
    # First gyro 0, then gyro 1 with lowpass 40 Hz: the second file write must
    # be between the two raw samples, not the raw 1.0 and not 0.0.
    batches2 = [
        [(0x02, (0.0, 0.0, 0.0))],
        [(0x04, (1.0, 0.0, 0.0, 0.0))],
        [(0x02, (1.0, 1.0, 1.0))],
    ]
    writes2 = run_bridge(FakeReader(batches2, stop_after=20), lowpass_hz=40.0)
    check("lowpass writes at least 2 samples", len(writes2) >= 2)
    if len(writes2) >= 2:
        wx = float(field(writes2[-1], "wx"))
        check("filtered wx is between 0 and 1", 0.0 < wx < 1.0)

    print()
    if FAILS:
        print(f"FAILED: {len(FAILS)} check(s)")
        for item in FAILS:
            print(f"  - {item}")
        return 1
    print("ALL PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
