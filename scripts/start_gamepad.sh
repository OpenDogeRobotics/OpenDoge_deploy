#!/bin/sh
# OpenDoge gamepad readiness / driver wrapper.
#
# Supported 2.4G receivers:
#   413d:2104  Xbox 360 clone dongle  -> xboxdrv creates /dev/input/js0
#   3537:1041  Zikway HID gamepad     -> kernel hid-generic/joydev creates js0
#
# The old opendoge-xboxdrv.service only waited for 413d:2104. This wrapper
# keeps the service active for either receiver:
#   - 413d:2104: exec xboxdrv (same as before)
#   - 3537:1041: wait for /dev/input/js0, then sleep forever; the kernel driver
#                 already provides js0, so xboxdrv is not needed.
#
# 3537:2106 is a different mode of the Zikway receiver and does NOT create js0;
# the wrapper prints a hint and keeps waiting for 1041.

set -eu

log() {
    echo "opendoge-gamepad: $*" >&2
}

# 30 s total wait for a supported receiver
i=0
while [ "$i" -lt 60 ]; do
    if lsusb 2>/dev/null | grep -q "3537:1041"; then
        log "Zikway 3537:1041 detected; waiting for kernel /dev/input/js0"
        j=0
        while [ "$j" -lt 20 ]; do
            if [ -e /dev/input/js0 ]; then
                log "native js0 ready; kernel joydev handles input"
                exec /bin/sleep infinity
            fi
            sleep 0.5
            j=$((j + 1))
        done
        log "3537:1041 present but /dev/input/js0 did not appear after 10s"
        exit 1
    fi

    if lsusb 2>/dev/null | grep -q "413d:2104"; then
        log "Xbox dongle 413d:2104 detected; starting xboxdrv"
        exec /usr/bin/xboxdrv \
            --device-by-id 413d:2104 \
            --type xbox360 \
            --detach-kernel-driver \
            --dbus disabled
    fi

    if lsusb 2>/dev/null | grep -q "3537:2106"; then
        # Only print once per second to avoid spamming the journal.
        if [ $((i % 2)) -eq 0 ]; then
            log "receiver is in 3537:2106 mode (no js0); replug/switch mode to 3537:1041"
        fi
    fi

    sleep 0.5
    i=$((i + 1))
done

log "no supported gamepad receiver after 30s (expected 413d:2104 or 3537:1041)"
exit 1
