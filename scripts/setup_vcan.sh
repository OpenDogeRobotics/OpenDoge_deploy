#!/usr/bin/env bash
# 创建/启用一个或多个 vcan 接口（供 USB2CAN -> vcan 桥接使用）。
#
# 用法:
#   sudo bash scripts/setup_vcan.sh
#   sudo bash scripts/setup_vcan.sh can0 can1 can2 can3
#
# 已存在的 vcan 会直接 UP；如果同名接口是真实 CAN，脚本会拒绝删除。

set -euo pipefail

IFACES=("$@")
if [[ ${#IFACES[@]} -eq 0 ]]; then
  IFACES=(can0)
fi

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run with sudo: sudo $0 ${IFACES[*]}" >&2
  exit 1
fi

modprobe vcan

for IFACE in "${IFACES[@]}"; do
  if ip link show "${IFACE}" &>/dev/null; then
    if ip -details link show "${IFACE}" | grep -q "vcan"; then
      ip link set up "${IFACE}"
      echo "${IFACE}: existing vcan, set UP"
    else
      echo "ERROR: ${IFACE} exists but is not vcan; refusing to delete it" >&2
      exit 1
    fi
  else
    ip link add dev "${IFACE}" type vcan
    ip link set up "${IFACE}"
    echo "${IFACE}: vcan created and UP"
  fi
  ip -details link show "${IFACE}" | head -2
done
