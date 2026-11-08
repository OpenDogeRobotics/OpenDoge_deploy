#!/usr/bin/env python3
"""record_highrate.py tests (no CAN hardware, no deploy required).

Covers:
  - EL05 type-1/type-2/type-17 frame decode against known values
  - calibration conversion (direction/offset/reduction)
  - managed CSV create + 5-run rotation, keep=1, foreign files survive
  - monitor-like flags (TRACK/DQ/STALE_STATUS/GAP/OUT)
  - end-to-end run() loop over a socketpair carrying synthetic CAN frames:
    CSV header/row widths, logical q/target, status/target ages, state
"""

from __future__ import annotations

import argparse
import csv
import json
import socket
import struct
import sys
import tempfile
import threading
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "hardware" / "motor"))

import record_highrate as rec  # noqa: E402

FAILS = []


def check(label: str, condition: bool) -> None:
    print(f"  {'PASS' if condition else 'FAIL'}  {label}")
    if not condition:
        FAILS.append(label)


def q_u(value: float, low: float, high: float) -> int:
    value = max(low, min(high, value))
    return int((value - low) * 65535.0 / (high - low))


def status_frame(motor: int, q: float, dq: float, tau: float, temp: float,
                 fault: int = 0, mode: int = 0) -> bytes:
    data2 = (mode << 14) | (fault << 8) | motor
    can_id = rec.build_ext_id(rec.COMM_STATUS, data2, 0xFF) | rec.CAN_EFF_FLAG
    payload = struct.pack(
        ">HHHH",
        q_u(q, rec.P_MIN, rec.P_MAX),
        q_u(dq, rec.V_MIN, rec.V_MAX),
        q_u(tau, rec.T_MIN, rec.T_MAX),
        int(round(temp * 10.0)),
    )
    return struct.pack("=IB3x8s", can_id, 8, payload)


def control_frame(motor: int, target: float, dq: float = 0.0,
                  kp: float = 12.0, kd: float = 0.5, tau_ff: float = 0.0) -> bytes:
    can_id = rec.build_ext_id(
        rec.COMM_CONTROL, q_u(tau_ff, rec.T_MIN, rec.T_MAX), motor) | rec.CAN_EFF_FLAG
    payload = struct.pack(
        ">HHHH",
        q_u(target, rec.P_MIN, rec.P_MAX),
        q_u(dq, rec.V_MIN, rec.V_MAX),
        q_u(kp, rec.KP_MIN, rec.KP_MAX),
        q_u(kd, rec.KD_MIN, rec.KD_MAX),
    )
    return struct.pack("=IB3x8s", can_id, 8, payload)


def test_decode() -> None:
    print("[1] frame decode")
    sf = status_frame(2, q=0.123, dq=-1.5, tau=0.75, temp=31.2, fault=2, mode=0)
    can_id, dlc, payload = struct.unpack("=IB3x8s", sf)
    decoded = rec.decode_frame(can_id & rec.CAN_EFF_MASK, payload[:dlc])
    check("status motor=2", decoded is not None and decoded["motor"] == 2)
    check("status q≈0.123", abs(decoded["q"] - 0.123) < 2e-3)
    check("status dq≈-1.5", abs(decoded["dq"] + 1.5) < 0.05)
    check("status tau≈0.75", abs(decoded["tau"] - 0.75) < 2e-3)
    check("status temp≈31.2", abs(decoded["temp"] - 31.2) < 0.11)
    check("status fault=2", decoded["fault"] == 2)

    cf = control_frame(5, target=0.4, dq=0.1)
    can_id, dlc, payload = struct.unpack("=IB3x8s", cf)
    decoded = rec.decode_frame(can_id & rec.CAN_EFF_MASK, payload[:dlc])
    check("control motor=5 (id in low byte)", decoded is not None and decoded["motor"] == 5)
    check("control target≈0.4", abs(decoded["q"] - 0.4) < 2e-3)
    check("control kp≈12", abs(decoded["kp"] - 12.0) < 0.02)

    value = 0.321
    payload = struct.pack("<H", rec.IDX_MECH_POS) + b"\x00\x00" + struct.pack("<f", value)
    can_id = rec.build_ext_id(rec.COMM_READ_PARAM, 2, 0xFF) | rec.CAN_EFF_FLAG
    decoded = rec.decode_frame(can_id & rec.CAN_EFF_MASK, payload)
    check("read reply value≈0.321", decoded is not None and abs(decoded["value"] - value) < 1e-6)


def test_calibration() -> None:
    print("[2] calibration conversion")
    calib = rec.load_calibration(rec.DEFAULT_CONFIG)
    check("config loaded FL_thigh direction=-1",
          abs(calib["FL_thigh_joint"]["direction"] + 1.0) < 1e-9)
    check("config loaded FL_thigh offset=0.8",
          abs(calib["FL_thigh_joint"]["offset"] - 0.8) < 1e-9)
    # motor 0.0 -> logical 0.8 (standing thigh); motor 0.4 -> logical 0.4
    check("motor 0 -> logical 0.8",
          abs(rec.to_logical_position(0.0, calib["FL_thigh_joint"]) - 0.8) < 1e-6)
    check("motor 0.4 -> logical 0.4",
          abs(rec.to_logical_position(0.4, calib["FL_thigh_joint"]) - 0.4) < 1e-6)
    # calf reduction 1.5: motor 0 -> -1.6
    check("calf motor 0 -> logical -1.6",
          abs(rec.to_logical_position(0.0, calib["FL_calf_joint"]) + 1.6) < 1e-6)
    check("calf joint torque scales by 1.5",
          abs(rec.to_joint_torque(2.0, calib["FL_calf_joint"]) - 3.0) < 1e-9)


def test_managed_csv() -> None:
    print("[3] managed CSV rotation")
    with tempfile.TemporaryDirectory() as tmp:
        d = Path(tmp)
        paths = []
        for _ in range(6):
            f, _w, path = rec.create_managed_csv(d)
            check(f"created {path.name if path else None}", f is not None)
            paths.append(path)
            f.close()
            rec.prune_managed_logs(d, 5, path)
        files = sorted(p for p in d.iterdir() if rec.MANAGED_LOG_RE.match(p.name))
        check("5 files remain", len(files) == 5)
        check("oldest removed", paths[0] not in files)
        check("current kept", paths[-1] in files)

        foreign = d / "highrate_latest.csv"
        foreign.write_text("keep me\n")
        target = d / "target.csv"
        target.write_text("target\n")
        link = d / "highrate_20200101_000000.csv"
        link.symlink_to(target)
        f, _w, current = rec.create_managed_csv(d)
        f.close()
        rec.prune_managed_logs(d, 1, current)
        check("non-matching CSV kept", foreign.exists())
        check("symlink kept", link.is_symlink())
        check("keep=1 keeps current only", current.exists())


def test_flags() -> None:
    print("[4] monitor-like flags")
    calib = rec.load_calibration(rec.DEFAULT_CONFIG)
    latest = {name: rec.blank_joint() for name in rec.JOINT_NAMES}
    now = 100.0
    item = latest["FL_hip_joint"]
    item.update({
        "q": 0.6, "target": 0.0, "dq": 20.0, "tau": 0.1, "temp": 30.0,
        "fault": 0, "status_t": now - 0.050, "pos_t": now - 0.050,
        "target_t": now - 0.050,
    })
    args = argparse.Namespace(
        dq_warn=15.0, tracking_warn=0.5, status_stale_ms=30.0,
        max_gap_ms=12.0, target_stale_ms=30.0, imu_stale_ms=100.0,
        out_margin=0.05,
    )
    flags = rec.build_flags(now, latest, calib, "active_rl", 10.0,
                            {"can0", "can1", "can2", "can3"}, now - 10.0, args)
    check("TRACK flagged", "TRACK:FL_hip_joint" in flags)
    check("DQ flagged", "DQ:FL_hip_joint" in flags)
    check("STALE_STATUS flagged", "STALE_STATUS:FL_hip_joint" in flags)
    check("GAP flagged", "GAP:FL_hip_joint" in flags)
    check("TGT_STALE flagged", "TGT_STALE:FL_hip_joint" in flags)
    check("OUT flagged", "OUT:FL_hip_joint" in flags)


def _fake_open_can(peers: dict):
    def opener(interface: str) -> socket.socket:
        local, peer = socket.socketpair(socket.AF_UNIX, socket.SOCK_DGRAM)
        local.setblocking(False)
        peers[interface] = peer
        return local
    return opener


def test_end_to_end() -> None:
    print("[5] end-to-end loop over socketpair")
    with tempfile.TemporaryDirectory() as tmp:
        d = Path(tmp)
        status = d / "status.json"
        status.write_text(json.dumps({
            "state": "active_rl", "fault_reason": "",
            "imu_age_ms": 5, "joints": [],
        }))
        imu = d / "imu.state"
        imu.write_text("wx=0\nwy=0\nwz=0\ngx=0\ngy=0\ngz=-1\nt=0\ngyro_seq=1\nquat_seq=1\n")
        command = d / "command.state"
        command.write_text(
            "vx=0.1\nvy=0.2\nyaw_rate=0.3\nactive=true\nestop=false\n"
            "position_control=false\nrl_inference=true\nclear_fault=false\n"
            "low_gain_mode=false\n"
        )

        peers: dict = {}
        original_open = rec.open_can
        rec.open_can = _fake_open_can(peers)
        old_argv = sys.argv
        sys.argv = [
            "record_highrate.py",
            "--interfaces", "can0",
            "--log-dir", str(d),
            "--duration", "0.8",
            "--rate-hz", "200",
            "--status", str(status),
            "--imu", str(imu),
            "--command", str(command),
            "--imu-stale-ms", "5000",
        ]
        error = []

        def runner() -> None:
            try:
                rec.run(rec.parse_args())
            except Exception as exc:  # noqa: BLE001
                error.append(exc)

        thread = threading.Thread(target=runner, daemon=True)
        thread.start()

        deadline = time.monotonic() + 2.0
        while not peers and time.monotonic() < deadline:
            time.sleep(0.01)
        check("fake CAN socket opened", bool(peers))

        stop = threading.Event()

        def sender() -> None:
            peer = peers["can0"]
            while not stop.is_set():
                # FL_thigh = motor 2 on can0. Motor q=0.0 -> logical 0.8;
                # target motor q=0.4 -> logical 0.4.
                peer.send(status_frame(2, q=0.0, dq=0.0, tau=0.05, temp=28.0))
                peer.send(control_frame(2, target=0.4))
                time.sleep(0.005)

        send_thread = threading.Thread(target=sender, daemon=True)
        send_thread.start()
        thread.join(timeout=5.0)
        stop.set()
        send_thread.join(timeout=1.0)
        rec.open_can = original_open
        sys.argv = old_argv

        check("run returned", not thread.is_alive())
        check("no exception", not error)
        files = sorted(p for p in d.iterdir() if rec.MANAGED_LOG_RE.match(p.name))
        check("one managed CSV", len(files) == 1)
        if not files:
            return
        rows = list(csv.reader(files[0].open(encoding="utf-8")))
        check("header matches", rows[0] == rec.CSV_HEADER)
        check("has data rows", len(rows) > 10)
        if len(rows) > 10:
            check("all row widths match header",
                  all(len(r) == len(rec.CSV_HEADER) for r in rows[1:]))
            header = rows[0]
            q_idx = header.index("FL_thigh_joint_q")
            tgt_idx = header.index("FL_thigh_joint_target")
            state_idx = header.index("state")
            q_vals = [float(r[q_idx]) for r in rows[1:] if r[q_idx]]
            tgt_vals = [float(r[tgt_idx]) for r in rows[1:] if r[tgt_idx]]
            check("q rows present", len(q_vals) > 5)
            check("q≈0.8 (motor 0 -> logical 0.8)",
                  bool(q_vals) and abs(sum(q_vals) / len(q_vals) - 0.8) < 0.01)
            check("target≈0.4 (motor 0.4 -> logical 0.4)",
                  bool(tgt_vals) and abs(sum(tgt_vals) / len(tgt_vals) - 0.4) < 0.01)
            check("state from status JSON", rows[1][state_idx] == "active_rl")


def main() -> int:
    test_decode()
    test_calibration()
    test_managed_csv()
    test_flags()
    test_end_to_end()
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
