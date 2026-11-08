#!/usr/bin/env bash
# ⚠️ 当前硬件为赤乌（CHIWU）原生 SocketCAN can0-can3；本脚本仅作
# gs_usb/达妙备选参考。当前接入见 docs/usb2can/chiwu_usb2can.md；
# 旧 DingLab 方案见 docs/legacy/README.md。
#
# 达妙 DM-USB2CANFD (gs_usb/SocketCAN 固件) 驱动安装助手 — RK3588 / aarch64。
#
# 默认只做检查（--check），不修改系统。确认无误后再用 --apply。
#
# 前置：模块已刷成 gs_usb/SocketCAN 固件（见 docs/usb2can/dm_usb2canfd.md）。
#
# 用法:
#   bash scripts/setup_dm_usb2canfd.sh --check
#   sudo bash scripts/setup_dm_usb2canfd.sh --apply
#   sudo bash scripts/setup_dm_usb2canfd.sh --apply \
#        --driver-dir ~/dm-tools/gs_usb_drives/gs_usb_sppe_RK_board/gs_usb_6.1
#   sudo bash scripts/setup_dm_usb2canfd.sh --apply --rebuild   # 强制本地编译
#
# 注意：模块安装到 /lib/modules/$(uname -r)/updates/，不会覆盖 in-tree gs_usb.ko。

set -euo pipefail

APPLY=0
REBUILD=0
DRIVER_DIR=""

usage() {
    awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "$0"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --check) APPLY=0 ;;
        --apply) APPLY=1 ;;
        --rebuild) REBUILD=1 ;;
        --driver-dir) DRIVER_DIR="${2:?--driver-dir 需要路径}"; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "未知参数: $1" >&2; usage; exit 1 ;;
    esac
    shift
done

KVER="$(uname -r)"
KMAJOR="$(echo "${KVER}" | cut -d. -f1,2)"

case "${KMAJOR}" in
    5.10|5.15) DEFAULT_SUBDIR="gs_usb_5.1" ;;
    6.1)       DEFAULT_SUBDIR="gs_usb_6.1" ;;
    6.8)       DEFAULT_SUBDIR="gs_usb_6.8" ;;
    *)         DEFAULT_SUBDIR="gs_usb_6.1" ;;
esac

if [[ -z "${DRIVER_DIR}" ]]; then
    DRIVER_DIR="${HOME}/dm-tools/gs_usb_drives/gs_usb_sppe_RK_board/${DEFAULT_SUBDIR}"
fi

# 安装到 updates/ 而不是覆盖 in-tree 模块；depmod 会优先选择 updates/。
UPDATES_DIR="/lib/modules/${KVER}/updates"
MODULE_PATH="${UPDATES_DIR}/gs_usb.ko"
STOCK_PATH="/lib/modules/${KVER}/kernel/drivers/net/can/usb/gs_usb.ko"

echo "=== 达妙 USB2CANFD gs_usb 驱动检查 ==="
echo "kernel     : ${KVER}"
echo "driver dir : ${DRIVER_DIR}"
echo "mode       : $([[ ${APPLY} -eq 1 ]] && echo APPLY || echo CHECK)"
echo

if [[ ! -d "${DRIVER_DIR}" ]]; then
    echo "❌ 驱动目录不存在: ${DRIVER_DIR}" >&2
    echo "   请先 clone: git clone https://gitee.com/kit-miao/dm-tools.git ~/dm-tools" >&2
    echo "   或指定 --driver-dir /path/to/gs_usb_${KMAJOR}" >&2
    exit 1
fi

KO="${DRIVER_DIR}/gs_usb.ko"
if [[ -f "${KO}" ]]; then
    MOD_VERMAGIC="$(modinfo -F vermagic "${KO}" 2>/dev/null || true)"
    echo "prebuilt   : ${KO}"
    echo "vermagic   : ${MOD_VERMAGIC:-<无法读取>}"
    # vermagic 的第一段必须与 uname -r 完全一致；CONFIG_MODVERSIONS 打开时
    # 还需要符号 CRC 匹配，预编译模块不匹配就只能本地编译。
    if [[ "${MOD_VERMAGIC%% *}" != "${KVER}" ]]; then
        echo "⚠️  预编译模块与当前内核不匹配，需要在板子上重新编译"
        REBUILD=1
    fi
else
    echo "prebuilt   : 不存在，将使用源码编译"
    REBUILD=1
fi

echo "stock mod  : ${STOCK_PATH}"
if [[ -f /boot/config-${KVER} ]]; then
    grep -E '^CONFIG_(CAN_GS_USB|MODVERSIONS)=' "/boot/config-${KVER}" 2>/dev/null || true
elif [[ -r /proc/config.gz ]]; then
    zgrep -E '^CONFIG_(CAN_GS_USB|MODVERSIONS)=' /proc/config.gz 2>/dev/null || true
fi
if command -v mokutil >/dev/null; then
    echo "secure boot: $(mokutil --sb-state 2>/dev/null | head -1 || echo unknown)"
fi
echo

if [[ ${REBUILD} -eq 1 ]]; then
    if ! command -v make >/dev/null; then
        echo "❌ 缺少 make，请安装: sudo apt install build-essential linux-headers-$(uname -r)" >&2
        exit 1
    fi
    if [[ ! -d "/lib/modules/${KVER}/build" ]]; then
        echo "❌ 缺少内核头文件: sudo apt install linux-headers-${KVER}" >&2
        exit 1
    fi
    if [[ ${APPLY} -eq 0 ]]; then
        echo "   检查模式下不编译。APPLY 时会执行: make -C ${DRIVER_DIR}"
    else
        echo "编译中 ..."
        make -C "${DRIVER_DIR}" -j"$(nproc)"
        KO="${DRIVER_DIR}/gs_usb.ko"
    fi
fi

if [[ ${APPLY} -eq 0 ]]; then
    echo
    echo "检查完成。确认后执行:"
    echo "  sudo bash $0 --apply --driver-dir ${DRIVER_DIR}"
    exit 0
fi

if [[ ${EUID} -ne 0 ]]; then
    echo "❌ --apply 需要 root: sudo bash $0 --apply ..." >&2
    exit 1
fi

echo
echo "停止服务并放下接口（释放 gs_usb）..."
systemctl stop opendoge-can 2>/dev/null || true
for i in 0 1 2 3; do ip link set "can${i}" down 2>/dev/null || true; done

echo "卸载旧模块并加载新模块 ..."
modprobe -r gs_usb 2>/dev/null || true
modprobe can
modprobe can-dev
modprobe usbcore

if ! insmod "${KO}"; then
    echo "❌ insmod 失败。检查 dmesg 和模块 vermagic/签名。" >&2
    exit 1
fi

echo "永久安装到 ${UPDATES_DIR}（保留 in-tree 模块不动）..."
install -D -m 0644 "${KO}" "${MODULE_PATH}"
depmod -a
modprobe gs_usb 2>/dev/null || true

echo "当前加载路径: $(modinfo -F filename gs_usb 2>/dev/null || echo n/a)"
echo "loaded vermagic: $(modinfo -F vermagic gs_usb 2>/dev/null || echo n/a)"

echo
echo "=== 当前 CAN 接口 ==="
ip -br link | grep -E "can[0-9]" || echo "  (还没有 canX：确认模块已刷 gs_usb 固件并重新插拔)"
echo
echo "=== udev 属性（用于稳定命名） ==="
for i in 0 1 2 3; do
    if [[ -e "/sys/class/net/can${i}" ]]; then
        echo "== can${i}"
        udevadm info -q property -n "can${i}" | \
            grep -E "ID_PATH|ID_SERIAL|ID_NET_NAME|DEVPATH" || true
    fi
done

cat <<EOF

下一步:
  1. 为每个接口配置经典 CAN 1 Mbps（显式关闭 fd）:
       sudo ip link set canX type can bitrate 1000000 fd off loopback off
       sudo ip link set canX up
       ip -details link show canX    # 不应出现 dbitrate / CAN_CTRLMODE_FD
  2. 校验映射:
       sudo python3 scripts/verify_can_mapping.py
  3. 若接口顺序不稳定:
       sudo bash scripts/setup_can_names.sh --freeze-current
     详细说明见 docs/usb2can/dm_usb2canfd.md 第 4 节。
EOF
