#!/usr/bin/env bash
# Bring up can0-3 as classic CAN 1 Mbps, with retry + gs_usb USB reset.
#
# This is the ExecStart of opendoge-can.service. If ip link fails (e.g. the
# CHIWU board was power-cycled and gs_usb reports "Couldn't start device
# (err=-110)"), reset the 1d50:606f USB devices, re-run
# prepare_can_interfaces.sh, and retry. A board with no 24V cannot be fixed
# here; the service will eventually fail with a clear message.
#
# Environment:
#   CAN_EXPECT_DRIVER     gs_usb (default) | vendor driver | auto
#   CAN_SETUP_ATTEMPTS    default 3
#   CAN_SETUP_SETTLE_SEC  default 2
#   CAN_USB_VID/PID       forwarded to reset_can_usb.sh (default 1d50/606f)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IFACES=(can0 can1 can2 can3)
EXPECTED="${CAN_EXPECT_DRIVER:-gs_usb}"
ATTEMPTS="${CAN_SETUP_ATTEMPTS:-3}"
SETTLE="${CAN_SETUP_SETTLE_SEC:-2}"

setup_once() {
  local iface drv
  for iface in "${IFACES[@]}"; do
    if [ ! -e "/sys/class/net/${iface}" ]; then
      echo "${iface}: does not exist" >&2
      return 1
    fi
    drv="$(basename "$(readlink -f "/sys/class/net/${iface}/device/driver" 2>/dev/null)" 2>/dev/null || true)"
    if [ "${EXPECTED}" != "auto" ] && [ -n "${EXPECTED}" ] && [ "${drv}" != "${EXPECTED}" ]; then
      echo "${iface}: expected driver ${EXPECTED}, got ${drv:-none}" >&2
      return 1
    fi
    ip link set "${iface}" down 2>/dev/null || true
    if ! ip link set "${iface}" type can bitrate 1000000 fd off loopback off; then
      echo "${iface}: ip link set type can failed" >&2
      return 1
    fi
    if ! ip link set up "${iface}"; then
      echo "${iface}: ip link set up failed" >&2
      return 1
    fi
    echo "${iface} up: classic CAN 1000000 bps, driver=${drv:-unknown}"
  done
}

for attempt in $(seq 1 "${ATTEMPTS}"); do
  echo "CAN setup attempt ${attempt}/${ATTEMPTS}"
  if setup_once; then
    exit 0
  fi
  if [ "${attempt}" -lt "${ATTEMPTS}" ]; then
    echo "attempt ${attempt} failed; resetting gs_usb USB devices ..." >&2
    if ! "${SCRIPT_DIR}/reset_can_usb.sh" --no-service; then
      echo "USB reset unavailable; waiting ${SETTLE}s before retry ..." >&2
    fi
    sleep "${SETTLE}"
    # USB re-enumeration can change interface names; re-apply the naming.
    "${SCRIPT_DIR}/prepare_can_interfaces.sh" || true
  fi
done

echo "CAN setup failed after ${ATTEMPTS} attempts." >&2
echo "  1) 检查 CHIWU 板 24V/LED、USB 线/Hub 供电" >&2
echo "  2) 如果电机断电会把板子一起复位：给 CHIWU 单独供电，或彻底断电后重上电" >&2
echo "  3) 再执行: sudo scripts/reset_can_usb.sh" >&2
echo "  4) journalctl -k | grep -E 'gs_usb|Couldn.t start device'" >&2
exit 1
