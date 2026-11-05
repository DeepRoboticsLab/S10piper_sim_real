#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CAN_PORT="${PIPER_CAN_PORT:-$($ROOT_DIR/bin/find_can.sh)}"

echo "Piper USB-CAN: $CAN_PORT"
echo "driver: $(basename "$(readlink -f "/sys/class/net/$CAN_PORT/device/driver")")"
ip -details -statistics link show dev "$CAN_PORT"
