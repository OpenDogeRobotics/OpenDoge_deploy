#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SERVICES_DIR="${ROOT_DIR}/daemons"

# When the installer is run through sudo, $HOME is often /root. The user
# services must be installed into the invoking user's home and managed via
# that user's systemd manager (systemctl --user under sudo has no
# DBUS/XDG_RUNTIME_DIR and fails with "Failed to connect to bus").
RUN_USER="${SUDO_USER:-$(id -un)}"
RUN_UID="$(id -u "${RUN_USER}" 2>/dev/null || id -u)"
RUN_HOME="$(getent passwd "${RUN_USER}" 2>/dev/null | cut -d: -f6 || true)"
[ -n "${RUN_HOME}" ] || RUN_HOME="${HOME}"
SYSTEMD_USER_DIR="${RUN_HOME}/.config/systemd/user"
SYSTEMD_SYSTEM_DIR="/etc/systemd/system"

run_user() {
    if [ "$(id -u)" -eq 0 ] && [ "${RUN_USER}" != "root" ]; then
        sudo -u "${RUN_USER}" env \
            XDG_RUNTIME_DIR="/run/user/${RUN_UID}" \
            DBUS_SESSION_BUS_ADDRESS="unix:path=/run/user/${RUN_UID}/bus" \
            "$@"
    else
        "$@"
    fi
}

CAN_SERVICE="opendoge-can.service"
XBOXDRV_SERVICE="opendoge-xboxdrv.service"
IMU_SERVICE="opendoge-imu.service"
JOYSTICK_SERVICE="opendoge-joystick.service"

usage() {
    cat <<EOF
Usage: $0

  安装原生 SocketCAN 方案：
    - opendoge-can.service 配置 can0-can3 (经典 CAN, 1 Mbps, fd off)
    - opendoge-xboxdrv.service (Xbox 驱动/原生 js0 就绪)
    - opendoge-imu.service (DM-IMU-L1 bridge)
    - opendoge-joystick.service (手柄 bridge)

  同时会停用/清理旧的 DINGLAB vcan + USB2CAN bridge 单元（如存在）。
  旧的 `--vcan` / `--udev-rule` 模式已移除；DingLab 代码已归档到
  ~/DingLab usb转can代码。
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -h|--help) usage; exit 0 ;;
        *) echo "未知参数: $1" >&2; usage; exit 1 ;;
    esac
    shift
done

echo "=== OpenDoge Service Installer ==="
echo "mode: native SocketCAN (can0-can3)"
echo ""

# ═══════════════════════════════════════════════════════════════════
# Step 1: CAN interfaces (system oneshot)
# ═══════════════════════════════════════════════════════════════════
echo "[1/5] Installing CAN interface setup ..."

if ! command -v sudo &>/dev/null; then
    echo "      [SKIP] sudo not available"
    exit 1
fi

# 旧 DingLab 方案清理：先停 bridge，再删 vcan 接口，否则原生 can0-3
# 无法创建（同名接口已被 vcan 占用）。
echo "      Cleaning up old DINGLAB vcan/bridge units (if any) ..."
sudo systemctl disable --now opendoge-usb2can0 opendoge-usb2can1 opendoge-vcan 2>/dev/null || true
sudo systemctl reset-failed opendoge-usb2can0 opendoge-usb2can1 opendoge-vcan 2>/dev/null || true
for i in 0 1 2 3; do
    if ip -details link show "can${i}" 2>/dev/null | grep -q "vcan"; then
        sudo ip link delete "can${i}" 2>/dev/null || true
    fi
done
for unit in opendoge-vcan.service opendoge-usb2can0.service opendoge-usb2can1.service; do
    if [ -e "${SYSTEMD_SYSTEM_DIR}/${unit}" ]; then
        sudo rm -f "${SYSTEMD_SYSTEM_DIR}/${unit}"
        echo "      removed old unit ${unit}"
    fi
done
if [ -e /etc/udev/rules.d/usb_can.rules ]; then
    # 旧 DingLab udev 规则会生成 /dev/USB2CAN0/1；已归档，删除以免混淆。
    sudo rm -f /etc/udev/rules.d/usb_can.rules
    sudo udevadm control --reload-rules 2>/dev/null || true
    echo "      removed old udev rule usb_can.rules (archived in ~/DingLab usb转can代码)"
fi
sudo systemctl daemon-reload

chmod +x "${ROOT_DIR}/scripts/prepare_can_interfaces.sh"
sed "s|__REPO_ROOT__|${ROOT_DIR}|g" "${SERVICES_DIR}/${CAN_SERVICE}" \
    | sudo tee "${SYSTEMD_SYSTEM_DIR}/${CAN_SERVICE}" >/dev/null
sudo systemctl daemon-reload
sudo systemctl enable "${CAN_SERVICE}"
echo "      Installed → ${SYSTEMD_SYSTEM_DIR}/${CAN_SERVICE}"
echo "      Starting ..."
sudo systemctl restart "${CAN_SERVICE}" || echo "      [WARN] CAN setup failed (USB-CAN adapter not plugged in?)"

# ═══════════════════════════════════════════════════════════════════
# Step 2: xboxdrv (system service)
# ═══════════════════════════════════════════════════════════════════
echo ""
echo "[2/5] Installing gamepad driver/readiness (system service) ..."

if ! command -v xboxdrv &>/dev/null; then
    echo "      [WARN] xboxdrv not found at /usr/bin/xboxdrv"
    echo "      Install with: sudo apt install xboxdrv"
    echo "      (only needed for the 413d:2104 Xbox dongle path)"
fi

# 服务模板用 __REPO_ROOT__ 引用 scripts/start_gamepad.sh
sed "s|__REPO_ROOT__|${ROOT_DIR}|g" \
    "${SERVICES_DIR}/${XBOXDRV_SERVICE}" \
    | sudo tee "${SYSTEMD_SYSTEM_DIR}/${XBOXDRV_SERVICE}" >/dev/null
chmod +x "${ROOT_DIR}/scripts/start_gamepad.sh"
sudo systemctl daemon-reload
sudo systemctl enable "${XBOXDRV_SERVICE}"
echo "      Installed → ${SYSTEMD_SYSTEM_DIR}/${XBOXDRV_SERVICE}"
echo "      Starting ..."
sudo systemctl restart "${XBOXDRV_SERVICE}" || echo "      [WARN] gamepad service failed to start (receiver not plugged in?)"

# ═══════════════════════════════════════════════════════════════════
# Step 3: Wait for js0 (needed by joystick bridge)
# ═══════════════════════════════════════════════════════════════════
echo ""
echo "[3/5] Waiting for /dev/input/js0 (up to 15s) ..."
for i in $(seq 1 30); do
    if [ -e /dev/input/js0 ]; then
        echo "      /dev/input/js0 ready ($(cat /sys/class/input/js0/device/name 2>/dev/null))"
        break
    fi
    sleep 0.5
done
if [ ! -e /dev/input/js0 ]; then
    echo "      [WARN] /dev/input/js0 did not appear — check:"
    echo "        sudo systemctl status opendoge-xboxdrv"
    echo "        lsusb | grep -E '413d:2104|3537:1041|3537:2106'"
    echo "        journalctl -u opendoge-xboxdrv -n 30"
fi

# ═══════════════════════════════════════════════════════════════════
# Step 4: IMU bridge (user service)
# ═══════════════════════════════════════════════════════════════════
# Make sure the invoking user's systemd manager/runtime dir exists before
# using `systemctl --user` (sudo alone has no DBUS/XDG_RUNTIME_DIR).
if command -v loginctl &>/dev/null; then
    loginctl enable-linger "${RUN_USER}" 2>/dev/null || true
fi
echo ""
echo "[4/5] Installing DM-IMU-L1 bridge (user service) ..."
# IMU 串口默认是 root:dialout 0660；user service 打不开，需要 udev 规则。
if [[ -f "${ROOT_DIR}/hardware/imu/99-opendoge-imu.rules" ]]; then
    sudo cp "${ROOT_DIR}/hardware/imu/99-opendoge-imu.rules" /etc/udev/rules.d/
    sudo udevadm control --reload-rules
    sudo udevadm trigger --action=change --subsystem-match=tty 2>/dev/null || true
    echo "      Installed IMU udev rule → /etc/udev/rules.d/99-opendoge-imu.rules"
    echo "      (if permission still denied, unplug/replug the IMU or reboot)"
fi
run_user install -d -m 0755 "${SYSTEMD_USER_DIR}"
# 服务模板里写死了 /home/orangepi/Opendoge/OpenDoge_firmware，
# 安装时替换为当前仓库实际路径（Ubuntu 24.04 默认用户通常不是 orangepi）。
tmp_unit="$(mktemp)"
chmod 0644 "${tmp_unit}"
sed -e "s|/home/orangepi/Opendoge/OpenDoge_firmware|${ROOT_DIR}|g" \
    -e "s|__REPO_ROOT__|${ROOT_DIR}|g" \
    "${SERVICES_DIR}/${IMU_SERVICE}" > "${tmp_unit}"
run_user install -m 0644 "${tmp_unit}" "${SYSTEMD_USER_DIR}/${IMU_SERVICE}"
rm -f "${tmp_unit}"
run_user systemctl --user daemon-reload
run_user systemctl --user enable "${IMU_SERVICE}"
echo "      Installed → ${SYSTEMD_USER_DIR}/${IMU_SERVICE}"
echo "      Starting ..."
run_user systemctl --user restart "${IMU_SERVICE}" || echo "      [WARN] IMU bridge failed to start (IMU not plugged in?)"

# ═══════════════════════════════════════════════════════════════════
# Step 5: Joystick bridge (user service)
# ═══════════════════════════════════════════════════════════════════
echo ""
echo "[5/5] Installing joystick bridge (user service) ..."
tmp_unit="$(mktemp)"
chmod 0644 "${tmp_unit}"
sed -e "s|/home/orangepi/Opendoge/OpenDoge_firmware|${ROOT_DIR}|g" \
    -e "s|__REPO_ROOT__|${ROOT_DIR}|g" \
    "${SERVICES_DIR}/${JOYSTICK_SERVICE}" > "${tmp_unit}"
run_user install -m 0644 "${tmp_unit}" "${SYSTEMD_USER_DIR}/${JOYSTICK_SERVICE}"
rm -f "${tmp_unit}"
run_user systemctl --user daemon-reload
run_user systemctl --user enable "${JOYSTICK_SERVICE}"
run_user systemctl --user restart "${JOYSTICK_SERVICE}" || echo "      [WARN] joystick bridge failed to start (no js0?)"
echo "      Installed → ${SYSTEMD_USER_DIR}/${JOYSTICK_SERVICE}"

# ═══════════════════════════════════════════════════════════════════
# Enable lingering so user services start at boot
# ═══════════════════════════════════════════════════════════════════
if command -v loginctl &>/dev/null; then
    loginctl enable-linger "${RUN_USER}" 2>/dev/null || true
    echo ""
    echo "      Lingering enabled for ${RUN_USER}"
fi

# ═══════════════════════════════════════════════════════════════════
# Summary
# ═══════════════════════════════════════════════════════════════════
echo ""
echo "=== Installation Complete ==="
echo ""
echo "System services (start at boot, require sudo to manage):"
echo "  systemctl status opendoge-can              # native CAN can0-3"
echo "  systemctl status opendoge-xboxdrv          # Xbox controller driver"
echo ""
echo "User services (start at boot via lingering):"
echo "  systemctl --user status opendoge-imu       # DM-IMU-L1 bridge"
echo "  systemctl --user status opendoge-joystick  # Joystick bridge"
echo ""
echo "Logs:"
echo "  journalctl -u opendoge-can -f              # CAN setup"
echo "  journalctl -u opendoge-xboxdrv -f          # xboxdrv"
echo "  journalctl --user -u opendoge-imu -f       # IMU bridge"
echo "  journalctl --user -u opendoge-joystick -f  # Joystick bridge"
echo ""
echo "Verify:"
echo "  ip -details link show can0                 # UP / bitrate 1000000 / no FD"
echo "  cat /tmp/opendoge_imu.state                # IMU data"
echo "  watch -n 0.2 cat /tmp/opendoge_command.state  # Joystick commands"
echo ""
echo "Disable:"
echo "  sudo systemctl disable opendoge-can opendoge-xboxdrv"
echo "  systemctl --user disable opendoge-imu opendoge-joystick"
