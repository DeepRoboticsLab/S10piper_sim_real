#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CAN_PORT="${1:-$($ROOT_DIR/bin/find_can.sh)}"

if [[ ! "$CAN_PORT" =~ ^[A-Za-z0-9_.:-]+$ ]]; then
    echo "Invalid CAN interface name: $CAN_PORT" >&2
    exit 2
fi

if [[ "$(id -u)" -eq 0 ]]; then
    ADMIN=()
else
    ADMIN=(sudo)
fi

"${ADMIN[@]}" ip link set dev "$CAN_PORT" down 2>/dev/null || true
"${ADMIN[@]}" ip link set dev "$CAN_PORT" type can bitrate 1000000 restart-ms 100
"${ADMIN[@]}" ip link set dev "$CAN_PORT" up

ip -details -statistics link show dev "$CAN_PORT"
