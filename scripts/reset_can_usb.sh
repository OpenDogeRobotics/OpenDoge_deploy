#!/usr/bin/env bash
# Reset the CHIWU/gs_usb USB adapters (authorized 0/1) and optionally restart
# opendoge-can. Use after a motor power cycle resets the CAN board:
#   gs_usb ... can0: Couldn't start device (err=-110)
#   ip link set can0 up -> RTNETLINK answers: Connection timed out
#
# The CHIWU board's USB side can stay enumerated while its CAN controller/24V
# side is not ready. A USB authorized reset (or a real board power cycle)
# re-initialises the gs_usb firmware.
#
# Usage:
#   sudo scripts/reset_can_usb.sh              # reset + restart opendoge-can
#   sudo scripts/reset_can_usb.sh --no-service # only reset USB; caller reconfigures
#   scripts/reset_can_usb.sh --dry-run         # list matching devices, no write
#
# Environment:
#   CAN_USB_VID / CAN_USB_PID   default 1d50 / 606f (gs_usb/CHIWU)
#   CAN_RESET_TIMEOUT           wait for can0-3 after reset, default 15 s
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VID="${CAN_USB_VID:-1d50}"
PID="${CAN_USB_PID:-606f}"
TIMEOUT="${CAN_RESET_TIMEOUT:-15}"
DRY_RUN=0
RESTART=1

for arg in "$@"; do
  case "${arg}" in
    --dry-run) DRY_RUN=1 ;;
    --no-service) RESTART=0 ;;
    -h|--help)
      sed -n '1,24p' "${BASH_SOURCE[0]}"
      exit 0
      ;;
    *)
      echo "unknown argument: ${arg}" >&2
      exit 2
      ;;
  esac
done

if [ "${DRY_RUN}" -eq 0 ] && [ "$(id -u)" -ne 0 ]; then
  echo "请用 sudo 运行（需要写 /sys/bus/usb/devices/*/authorized）" >&2
  exit 2
fi

mapfile -t devices < <(
  for d in /sys/bus/usb/devices/*; do
    [ -e "${d}/idVendor" ] || continue
    [ "$(cat "${d}/idVendor" 2>/dev/null)" = "${VID}" ] || continue
    [ "$(cat "${d}/idProduct" 2>/dev/null)" = "${PID}" ] || continue
    echo "${d}"
  done | sort -V
)

if [ "${#devices[@]}" -eq 0 ]; then
  echo "没有找到 ${VID}:${PID} 的 gs_usb 设备。" >&2
  echo "  检查 CHIWU 板 USB 线/供电；如果板子 24V 和电机共开关，先恢复板子电源。" >&2
  exit 1
fi

echo "找到 ${#devices[@]} 个 ${VID}:${PID} gs_usb 设备:"
printf '  %s\n' "${devices[@]}"

if [ "${DRY_RUN}" -eq 1 ]; then
  echo "dry-run: 未写 authorized，也未重启服务。"
  exit 0
fi

for d in "${devices[@]}"; do
  if [ ! -e "${d}/authorized" ]; then
    echo "警告: ${d}/authorized 不存在，跳过" >&2
    continue
  fi
  echo "reset ${d}"
  echo 0 > "${d}/authorized"
  sleep 0.3
  echo 1 > "${d}/authorized"
done

deadline=$((SECONDS + TIMEOUT))
while [ "${SECONDS}" -lt "${deadline}" ]; do
  if [ -e /sys/class/net/can0 ] && [ -e /sys/class/net/can1 ] && \
     [ -e /sys/class/net/can2 ] && [ -e /sys/class/net/can3 ]; then
    break
  fi
  sleep 0.5
done

if [ "${RESTART}" -eq 1 ]; then
  echo "重启 opendoge-can.service ..."
  systemctl restart opendoge-can.service
  systemctl --no-pager status opendoge-can.service | head -6 || true
else
  echo "USB reset 完成（--no-service）；由调用方重新 prepare/setup。"
fi
