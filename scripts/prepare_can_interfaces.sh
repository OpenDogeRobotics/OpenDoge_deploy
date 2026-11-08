#!/usr/bin/env bash
# 把 CHIWU/gs_usb 的 4 个 CAN 接口稳定命名为 can0-can3。
#
# 背景：CHIWU 板内部是 4 个独立的 gs_usb USB 设备（1d50:606f），Linux 内核
# 按枚举顺序创建 canX。如果本次启动里 can0-3 已经被 vcan 用过，新设备会
# 变成 can4-can7。本脚本按 USB 路径排序（板子 CAN1-CAN4 → 1-3.6.1..4），
# 把前 4 个 gs_usb 接口改名为 can0-can3；已经是 can0-3 时不做任何事。
#
# 由 opendoge-can.service 以 root 调用；也可手动 sudo 运行。
set -euo pipefail

WANT=(can0 can1 can2 can3)
TIMEOUT="${CAN_RENAME_TIMEOUT:-60}"
DRY_RUN=0
[ "${1:-}" = "--dry-run" ] && DRY_RUN=1
EXPECT_VID="${CAN_USB_VID:-1d50}"
EXPECT_PID="${CAN_USB_PID:-606f}"

collect() {
  local d name drv usb vid pid
  for d in /sys/class/net/*; do
    name=$(basename "$d")
    drv=$(basename "$(readlink -f "$d/device/driver" 2>/dev/null)" 2>/dev/null || true)
    [ "$drv" = "gs_usb" ] || continue
    usb=$(dirname "$(readlink -f "$d/device" 2>/dev/null)") || continue
    vid=$(cat "$usb/idVendor" 2>/dev/null || true)
    pid=$(cat "$usb/idProduct" 2>/dev/null || true)
    if [ -n "$EXPECT_VID" ] && [ "$vid" != "$EXPECT_VID" ]; then continue; fi
    if [ -n "$EXPECT_PID" ] && [ "$pid" != "$EXPECT_PID" ]; then continue; fi
    printf '%s|%s|%s|%s\n' "$usb" "$name" "$vid" "$pid"
  done | sort -V
}

deadline=$((SECONDS + TIMEOUT))
while true; do
  mapfile -t rows < <(collect)
  if [ "${#rows[@]}" -ge 4 ]; then
    break
  fi
  if [ "$SECONDS" -ge "$deadline" ]; then
    echo "prepare_can_interfaces: 找到 ${#rows[@]} 个 gs_usb(${EXPECT_VID}:${EXPECT_PID}) 接口，期望 4 个" >&2
    printf '  %s\n' "${rows[@]:-none}" >&2
    echo "  检查 CHIWU 板 USB、dmesg、gs_usb 驱动；" >&2
    echo "  如果板子 VID/PID 不同，用 CAN_USB_VID=xxxx CAN_USB_PID=xxxx 覆盖。" >&2
    exit 1
  fi
  sleep 0.5
done

names=()
serials=()
for row in "${rows[@]:0:4}"; do
  IFS='|' read -r usb name _vid _pid <<< "$row"
  names+=("$name")
  serials+=("$(cat "$usb/serial" 2>/dev/null || echo "?")")
done

if [ "${names[0]}" = can0 ] && [ "${names[1]}" = can1 ] && \
   [ "${names[2]}" = can2 ] && [ "${names[3]}" = can3 ]; then
  echo "prepare_can_interfaces: can0-can3 已经正确"
  for i in 0 1 2 3; do
    echo "  ${WANT[$i]} <- ${names[$i]} (serial ${serials[$i]})"
  done
  exit 0
fi

if [ "$DRY_RUN" -eq 1 ]; then
  echo "prepare_can_interfaces: 将按以下顺序改名（dry-run，未执行）:"
  for i in 0 1 2 3; do
    echo "  ${names[$i]} -> ${WANT[$i]} (serial ${serials[$i]})"
  done
  exit 0
fi

# 目标名不能被其他（非源）接口占用。
for t in "${WANT[@]}"; do
  if [ -e "/sys/class/net/$t" ]; then
    is_source=0
    for n in "${names[@]}"; do
      [ "$n" = "$t" ] && is_source=1
    done
    if [ "$is_source" -eq 0 ]; then
      echo "prepare_can_interfaces: $t 已存在且不是 gs_usb 源接口，拒绝改名" >&2
      exit 1
    fi
  fi
done

# 两阶段改名，避免源/目标重名冲突。
for idx in 0 1 2 3; do
  ip link set "${names[$idx]}" down 2>/dev/null || true
  ip link set "${names[$idx]}" name "can_gs_tmp_$idx"
done
for idx in 0 1 2 3; do
  ip link set "can_gs_tmp_$idx" name "${WANT[$idx]}"
done

echo "prepare_can_interfaces: 已改名（按 USB 路径排序）:"
for idx in 0 1 2 3; do
  echo "  ${WANT[$idx]} <- ${names[$idx]} (serial ${serials[$idx]})"
done
echo "  接口现在是 DOWN；opendoge-can.service 会设置 1 Mbps / fd off / UP。"
