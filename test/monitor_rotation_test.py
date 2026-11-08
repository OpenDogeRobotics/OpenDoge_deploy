#!/usr/bin/env python3
"""monitor_status.py rolling-CSV tests (no hardware, no deploy required).

Covers:
  - create_managed_csv + prune_managed_logs: 6 runs -> 5 files, oldest removed
  - keep=1: only the current file survives
  - non-matching CSV / symlink are never pruned
  - --once with missing status: header-only file is removed (no slot used)
  - --once with a valid status: managed file with header + data
  - --csv PATH: legacy append (one header, rows from both runs), no rotation
  - --no-csv: no file
"""

from __future__ import annotations

import csv
import json
import subprocess
import sys
import tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
MONITOR = REPO_ROOT / "hardware" / "motor" / "monitor_status.py"
sys.path.insert(0, str(REPO_ROOT / "hardware" / "motor"))

import monitor_status  # noqa: E402

FAILS = []


def check(label: str, condition: bool) -> None:
    print(f"  {'PASS' if condition else 'FAIL'}  {label}")
    if not condition:
        FAILS.append(label)


def managed(dir_path: Path) -> list[Path]:
    return sorted(
        p for p in dir_path.iterdir()
        if p.is_file() and not p.is_symlink()
        and monitor_status.MANAGED_LOG_RE.match(p.name)
    )


def status_json(joint_count: int = 1) -> str:
    joints = []
    for i in range(joint_count):
        joints.append({
            "n": ["FL_hip_joint", "FL_thigh_joint", "FL_calf_joint"][i % 3],
            "q": 0.0, "target": 0.0, "dq": 0.0, "tau": 0.0,
            "temp": 30.0, "fault": 0, "fault_sta": 0,
            "age_ms": 1, "recv": True,
        })
    return json.dumps({
        "t": 1.0, "state": "active_rl", "active_cmd": True,
        "rl_inference": True, "can_rx": 2520, "can_tx": 2520, "can_err": 0,
        "max_late_us": 2, "fault_reason": "", "imu": {"wz": 0.0, "gz": -1.0},
        "joints": joints,
    })


def run_monitor(args: list[str]) -> subprocess.CompletedProcess:
    return subprocess.run(
        [sys.executable, str(MONITOR), *args],
        capture_output=True, text=True, timeout=20,
    )


def test_create_prune() -> None:
    print("[1] create + prune (keep=5)")
    with tempfile.TemporaryDirectory() as tmp:
        d = Path(tmp)
        paths = []
        for _ in range(6):
            csv_file, writer, path = monitor_status.create_managed_csv(d)
            check(f"created {path.name if path else None}", csv_file is not None)
            paths.append(path)
            csv_file.close()
            monitor_status.prune_managed_logs(d, 5, path)
        files = managed(d)
        check("5 files remain", len(files) == 5)
        check("oldest removed", paths[0] not in files)
        check("current kept", paths[-1] in files)
        check("all names match regex", all(monitor_status.MANAGED_LOG_RE.match(p.name) for p in files))


def test_keep_one() -> None:
    print("[2] keep=1")
    with tempfile.TemporaryDirectory() as tmp:
        d = Path(tmp)
        last = None
        for _ in range(3):
            csv_file, _writer, last = monitor_status.create_managed_csv(d)
            csv_file.close()
            monitor_status.prune_managed_logs(d, 1, last)
        files = managed(d)
        check("only current remains", files == [last])


def test_foreign_files_survive() -> None:
    print("[3] non-matching CSV / symlink survive")
    with tempfile.TemporaryDirectory() as tmp:
        d = Path(tmp)
        foreign = d / "monitor_status_latest.csv"
        foreign.write_text("keep me\n")
        target = d / "target.csv"
        target.write_text("target\n")
        link = d / "monitor_status_20200101_000000.csv"
        link.symlink_to(target)
        csv_file, _writer, current = monitor_status.create_managed_csv(d)
        csv_file.close()
        monitor_status.prune_managed_logs(d, 1, current)
        check("non-matching CSV kept", foreign.exists())
        check("symlink kept", link.is_symlink())
        check("current kept", current.exists())


def test_once_missing_status() -> None:
    print("[4] --once with missing status: no slot consumed")
    with tempfile.TemporaryDirectory() as tmp:
        d = Path(tmp)
        missing = d / "no_such_status.json"
        result = run_monitor([
            "--status", str(missing), "--log-dir", str(d),
            "--once",
        ])
        check("exit code 1", result.returncode == 1)
        check("no managed file left", managed(d) == [])


def test_once_valid_status() -> None:
    print("[5] --once with valid status: header + one data row per joint")
    with tempfile.TemporaryDirectory() as tmp:
        d = Path(tmp)
        status = d / "status.json"
        status.write_text(status_json(3))
        result = run_monitor([
            "--status", str(status), "--log-dir", str(d), "--once",
        ])
        check("exit code 0", result.returncode == 0)
        files = managed(d)
        check("one managed file", len(files) == 1)
        if files:
            rows = list(csv.reader(files[0].open(encoding="utf-8")))
            check("header + 3 rows", len(rows) == 4)
            check("header matches", rows[0] == monitor_status.CSV_HEADER)


def test_legacy_csv_append() -> None:
    print("[6] --csv PATH legacy append")
    with tempfile.TemporaryDirectory() as tmp:
        d = Path(tmp)
        status = d / "status.json"
        status.write_text(status_json(1))
        legacy = d / "legacy.csv"
        for _ in range(2):
            result = run_monitor([
                "--status", str(status), "--csv", str(legacy), "--once",
            ])
            check("exit code 0", result.returncode == 0)
        rows = list(csv.reader(legacy.open(encoding="utf-8")))
        check("one header + two data rows", len(rows) == 3)
        check("only one header", rows.count(monitor_status.CSV_HEADER) == 1)
        check("no managed file for --csv", managed(d) == [])


def test_no_csv() -> None:
    print("[7] --no-csv")
    with tempfile.TemporaryDirectory() as tmp:
        d = Path(tmp)
        status = d / "status.json"
        status.write_text(status_json(1))
        result = run_monitor([
            "--status", str(status), "--log-dir", str(d), "--no-csv", "--once",
        ])
        check("exit code 0", result.returncode == 0)
        check("no managed file", managed(d) == [])


def test_new_flags() -> None:
    print("[8] STALE_STATUS/GAP/FB_RATIO + CAN state")
    with tempfile.TemporaryDirectory() as tmp:
        d = Path(tmp)
        data = json.loads(status_json(1))
        data["joints"][0].update({
            "status_age_ms": 100, "max_gap_ms": 50,
            "expected": 100, "status_frames": 50,
        })
        data["can_buses"] = [{"name": "can0", "state": "ERROR-PASSIVE"}]
        data["imu_age_ms"] = 12
        status = d / "status.json"
        status.write_text(json.dumps(data))
        result = run_monitor([
            "--status", str(status), "--log-dir", str(d),
            "--no-csv", "--once",
        ])
        out = result.stdout + result.stderr
        check("STALE_STATUS", "STALE_STATUS" in out)
        check("GAP", "GAP" in out)
        check("FB_RATIO", "FB_RATIO" in out)
        check("CAN state", "can_state=can0:ERROR-PASSIVE" in out)
        data["imu_age_ms"] = 200
        status.write_text(json.dumps(data))
        result = run_monitor([
            "--status", str(status), "--log-dir", str(d),
            "--no-csv", "--once", "--imu-stale-ms", "50",
        ])
        check("IMU stale warning",
              "IMU age 200 ms" in (result.stdout + result.stderr))
        # A lifetime max_gap (startup) must not raise GAP while the current
        # status age is healthy.
        data["joints"][0].update({
            "status_age_ms": 4, "max_gap_ms": 400,
            "expected": 100, "status_frames": 100,
        })
        data["imu_age_ms"] = 1
        status.write_text(json.dumps(data))
        result = run_monitor([
            "--status", str(status), "--log-dir", str(d),
            "--no-csv", "--once",
        ])
        check("lifetime max_gap does not raise GAP",
              "GAP" not in (result.stdout + result.stderr))


def main() -> int:
    test_create_prune()
    test_keep_one()
    test_foreign_files_survive()
    test_once_missing_status()
    test_once_valid_status()
    test_legacy_csv_append()
    test_no_csv()
    test_new_flags()
    print()
    if FAILS:
        print(f"FAILED: {len(FAILS)}")
        for f in FAILS:
            print("  -", f)
        return 1
    print("All monitor rotation tests passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
