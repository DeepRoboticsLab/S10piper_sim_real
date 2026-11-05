#!/usr/bin/env bash
set -euo pipefail

if [[ -n "${PIPER_CAN_PORT:-}" ]]; then
    printf '%s\n' "$PIPER_CAN_PORT"
    exit 0
fi

shopt -s nullglob
for netdev in /sys/class/net/can*; do
    driver_path="$(readlink -f "$netdev/device/driver" 2>/dev/null || true)"
    if [[ "$(basename "$driver_path")" == "gs_usb" ]]; then
        basename "$netdev"
        exit 0
    fi
done

echo "No gs_usb USB-CAN interface was found; check the adapter and gs_usb module." >&2
exit 1
