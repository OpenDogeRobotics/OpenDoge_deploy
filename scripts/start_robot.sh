#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-damping}"
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

CAN_IFACES="${CAN_IFACES:-can0 can1 can2 can3}"
CAN_BITRATE="${CAN_BITRATE:-1000000}"
COMMAND_FILE="${COMMAND_FILE:-/tmp/opendoge_command.state}"
IMU_FILE="${IMU_FILE:-/tmp/opendoge_imu.state}"
STATUS_FILE="${STATUS_FILE:-/tmp/opendoge_status.json}"
CONFIG_FILE="${CONFIG_FILE:-${ROOT_DIR}/deploy/configs/opendoge_deploy.conf}"
DEPLOY_BIN="${DEPLOY_BIN:-${ROOT_DIR}/install/opendoge_deploy/bin/opendoge_deploy}"
POLICY_BACKEND="${POLICY_BACKEND:-onnx}"
POLICY_PATH="${POLICY_PATH:-}"
REALTIME_ARGS="${REALTIME_ARGS-}"
# Default to SCHED_FIFO + CPU 0 for the control loop. Requires CAP_SYS_NICE on
# the binary (one-time: sudo setcap cap_sys_nice+ep install/.../opendoge_deploy);
# if the cap is missing the deploy prints a warning and continues at normal
# priority. Override with REALTIME_ARGS="" to disable.
if [ -z "${REALTIME_ARGS}" ]; then
  REALTIME_ARGS="--realtime --cpu 0"
fi
CALIB_OUTPUT="${CALIB_OUTPUT:-/tmp/opendoge_calibration.conf}"
CALIB_CHANNEL="${CALIB_CHANNEL:-can0}"

usage() {
  cat <<EOF
Usage: $0 [dry|damping|policy|calibrate|low-gain|verify]

Modes:
  dry         Dry-run deploy (no hardware, no policy, 2s)
  damping     Real deploy with PD damping (no policy)
  policy      Real deploy with ONNX policy inference
  low-gain    Real deploy, low-gain static standing test
  calibrate   Motor calibration on a single CAN channel
  verify      Check all services and interfaces are ready, then exit

Prerequisites:
  CAN interfaces (can0-3): managed by opendoge-can.service
  IMU bridge:              managed by opendoge-imu.service (user)
  Joystick bridge:         managed by opendoge-joystick.service (user)
  Xbox driver:             managed by opendoge-xboxdrv.service

  Install all services once with: bash scripts/install_services.sh

Environment:
  CAN_IFACES       CAN interfaces, default: "can0 can1 can2 can3"
  CAN_BITRATE      CAN bitrate, default: 1000000
  COMMAND_FILE     command.state path, default: /tmp/opendoge_command.state
  IMU_FILE         imu.state path, default: /tmp/opendoge_imu.state
  CONFIG_FILE      deploy config path
  POLICY_PATH      ONNX path, default: <repo>/policy/opendoge_flat.onnx
  POLICY_BACKEND   policy backend, default: onnx
  REALTIME_ARGS    optional deploy realtime args, e.g. "--realtime --cpu 0"
  CALIB_OUTPUT     calibration output path, default: /tmp/opendoge_calibration.conf
  CALIB_CHANNEL    CAN channel for calibration, default: can0
EOF
}

require_file() {
  local path="$1"
  local label="$2"
  if [[ ! -e "${path}" ]]; then
    echo "${label} not found: ${path}" >&2
    exit 1
  fi
}

write_safe_command() {
  mkdir -p "$(dirname "${COMMAND_FILE}")" "$(dirname "${IMU_FILE}")"
  # Only seed files if they don't exist (services may already have written real data)
  if [[ ! -s "${COMMAND_FILE}" ]]; then
    cat >"${COMMAND_FILE}" <<EOF
vx=0.0
vy=0.0
yaw_rate=0.0
active=false
estop=false
clear_fault=false
low_gain_mode=false
EOF
  fi
  if [[ ! -s "${IMU_FILE}" ]]; then
    cat >"${IMU_FILE}" <<EOF
wx=0.0
wy=0.0
wz=0.0
gx=0.0
gy=0.0
gz=-1.0
EOF
  fi
}

verify_can() {
  local ok=true
  local iface details drv
  for iface in ${CAN_IFACES}; do
    if ! ip link show "${iface}" &>/dev/null; then
      echo "  ✗ ${iface} does not exist (check the USB-CAN adapter / driver)" >&2
      ok=false
      continue
    fi

    details="$(ip -details link show "${iface}" 2>/dev/null || true)"
    drv="$(basename "$(readlink -f "/sys/class/net/${iface}/device/driver" 2>/dev/null)" 2>/dev/null || true)"

    if echo "${details}" | grep -q "vcan"; then
      echo "  ✗ ${iface} is a vcan interface — old DingLab bridge still active?" >&2
      echo "    sudo systemctl disable --now opendoge-vcan opendoge-usb2can0 opendoge-usb2can1" >&2
      ok=false
    elif ! ip link show "${iface}" | grep -q "state UP"; then
      echo "  ✗ ${iface} exists but is DOWN — sudo systemctl restart opendoge-can" >&2
      ok=false
    elif ! echo "${details}" | grep -q "bitrate 1000000"; then
      echo "  ✗ ${iface} UP but bitrate is not 1000000" >&2
      ok=false
    elif echo "${details}" | grep -qE "dbitrate|CAN_CTRLMODE_FD"; then
      echo "  ✗ ${iface} has CAN FD enabled — EL05 requires classic CAN" >&2
      ok=false
    else
      echo "  ✓ ${iface} UP, classic CAN 1000000 bps, driver=${drv:-unknown}"
    fi
  done
  if ! ${ok}; then
    echo ""
    echo "Troubleshooting:" >&2
    echo "  sudo systemctl status opendoge-can" >&2
    echo "  sudo journalctl -u opendoge-can -n 40" >&2
    echo "  sudo scripts/reset_can_usb.sh   # CHIWU 板断电/复位后, 或 gs_usb err=-110" >&2
    echo "  lsusb ; for i in 0 1 2 3; do readlink -f /sys/class/net/can\$i/device/driver; done" >&2
    echo "  sudo journalctl -k | grep -Ei 'usb.*disconnect|can[0-9]|gs_usb|Couldn.t start device'  # USB enumeration" >&2
    return 1
  fi
  return 0
}

verify_service() {
  local service="$1"
  local manager="$2"  # "system" or "user"
  local label="$3"

  if [[ "${manager}" == "user" ]]; then
    if systemctl --user is-active --quiet "${service}" 2>/dev/null; then
      echo "  ✓ ${label} (${service}) active"
      return 0
    else
      echo "  ✗ ${label} (${service}) not running — restart with: systemctl --user restart ${service}" >&2
      return 1
    fi
  else
    if systemctl is-active --quiet "${service}" 2>/dev/null; then
      echo "  ✓ ${label} (${service}) active"
      return 0
    else
      echo "  ✗ ${label} (${service}) not running — restart with: sudo systemctl restart ${service}" >&2
      return 1
    fi
  fi
}

verify_all() {
  local all_ok=true

  echo "Verifying services and interfaces ..."
  echo ""

  echo "CAN interfaces:"
  verify_can || all_ok=false

  echo ""
  echo "Daemon services:"
  verify_service "opendoge-xboxdrv" "system" "Xbox driver" || all_ok=false
  verify_service "opendoge-joystick" "user" "Joystick bridge" || all_ok=false
  verify_service "opendoge-imu" "user" "IMU bridge" || all_ok=false

  echo ""
  if ${all_ok}; then
    echo "✓ All services ready."
    return 0
  else
    echo "✗ Some services are not ready. See above for restart commands." >&2
    return 1
  fi
}

run_deploy() {
  local args=("$@")
  require_file "${DEPLOY_BIN}" "deploy binary"
  require_file "${CONFIG_FILE}" "deploy config"
  "${DEPLOY_BIN}" \
    --config "${CONFIG_FILE}" \
    --command-file "${COMMAND_FILE}" \
    --imu-file "${IMU_FILE}" \
    --status-file "${STATUS_FILE}" \
    ${REALTIME_ARGS} \
    "${args[@]}"
}

case "${MODE}" in
  -h|--help|help)
    usage
    ;;

  verify)
    verify_all
    ;;

  dry)
    write_safe_command
    run_deploy --policy-backend none --duration-sec 2
    ;;

  damping)
    verify_all
    write_safe_command
    run_deploy --real --enable --clear-fault --policy-backend none
    ;;

  policy)
    if [[ -z "${POLICY_PATH}" ]]; then
      POLICY_PATH="${ROOT_DIR}/policy/opendoge_flat.onnx"
    fi
    require_file "${POLICY_PATH}" "policy (default: policy/opendoge_flat.onnx)"
    echo "Using policy: ${POLICY_PATH}"
    verify_all
    write_safe_command
    run_deploy --real --enable --clear-fault \
      --policy-backend "${POLICY_BACKEND}" \
      --policy-path "${POLICY_PATH}"
    ;;

  calibrate)
    echo "Verifying CAN interfaces ..."
    verify_can
    echo ""
    echo "Running motor calibration on ${CALIB_CHANNEL}..."
    "${ROOT_DIR}/hardware/motor/el05_calibrate.py" \
      --channel "${CALIB_CHANNEL}" \
      --config "${CONFIG_FILE}" \
      | tee "${CALIB_OUTPUT}"
    echo ""
    echo "Calibration written to ${CALIB_OUTPUT}"
    echo "Append its contents to ${CONFIG_FILE} before running damping or policy mode."
    ;;

  low-gain)
    verify_all
    # --low-gain forces the mode in the deploy itself. The pre-written command
    # file alone is not enough: opendoge-joystick rewrites it at ~100 Hz from
    # gamepad state (low_gain_mode defaults false), so the flag is authoritative.
    cat >"${COMMAND_FILE}" <<EOF
vx=0.0
vy=0.0
yaw_rate=0.0
active=false
estop=false
clear_fault=false
low_gain_mode=true
EOF
    run_deploy --real --enable --clear-fault --policy-backend none --low-gain
    echo "Low-gain mode exited. Use BACK button on joystick to toggle low_gain_mode."
    ;;

  *)
    echo "Unknown mode: ${MODE}" >&2
    usage >&2
    exit 1
    ;;
esac
