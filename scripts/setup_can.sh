#!/usr/bin/env bash
# 配置单个经典 CAN 接口 (EL05 使用 1 Mbps 经典帧, 不要开 CAN FD)。
set -euo pipefail

IFACE="${1:-can0}"
BITRATE="${2:-1000000}"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run with sudo: sudo $0 ${IFACE} ${BITRATE}" >&2
  exit 1
fi

if [[ ! -e "/sys/class/net/${IFACE}" ]]; then
  echo "ERROR: ${IFACE} does not exist (check USB-CAN adapter / driver)" >&2
  exit 1
fi

DRIVER="$(basename "$(readlink -f "/sys/class/net/${IFACE}/device/driver" 2>/dev/null)" 2>/dev/null || true)"
# CAN_EXPECT_DRIVER: gs_usb (default), a vendor driver name, or "auto".
EXPECT="${CAN_EXPECT_DRIVER:-gs_usb}"
if [[ "${EXPECT}" != "auto" && -n "${EXPECT}" && "${DRIVER}" != "${EXPECT}" ]]; then
  echo "WARN: ${IFACE} driver is '${DRIVER:-unknown}', expected '${EXPECT}'" >&2
  echo "      (set CAN_EXPECT_DRIVER=auto to silence, e.g. for CHIWU/vendor drivers)" >&2
fi

ip link set down "${IFACE}" 2>/dev/null || true
# fd off 是显式的：EL05 是经典 CAN，不能用 CAN FD 模式。
if ! ip link set "${IFACE}" type can bitrate "${BITRATE}" fd off loopback off; then
  echo "ERROR: failed to configure ${IFACE} (bitrate=${BITRATE}, fd=off)" >&2
  echo "       check iproute2/kernel support for 'fd off'" >&2
  exit 1
fi
ip link set up "${IFACE}"

echo "${IFACE} is up: classic CAN (fd off) at ${BITRATE} bps, driver=${DRIVER:-unknown}"
ip -details -statistics link show "${IFACE}"
