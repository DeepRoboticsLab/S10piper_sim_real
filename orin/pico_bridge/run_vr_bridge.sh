#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="${S10_PIPER_VR_PYTHON:-/usr/bin/python3}"
PXREA_SERVICE_ROOT="${PXREA_SERVICE_ROOT:-/opt/apps/roboticsservice}"

if [[ ! -x "$PYTHON_BIN" ]]; then
    echo "Python不存在: $PYTHON_BIN" >&2
    exit 1
fi

# Static protocol/mapping checks do not need PXREA RoboticsService.
for arg in "$@"; do
    if [[ "$arg" == "--self-test" ]]; then
        exec "$PYTHON_BIN" -u "$SCRIPT_DIR/s10_piper_vr_bridge.py" "$@"
    fi
done

# PXREARobotSDK is a localhost gRPC client. Start the installed local service
# when it is not already listening; do not launch a duplicate instance.
if ! ss -ltn 2>/dev/null | grep -q ':60061 '; then
    if [[ ! -x "$PXREA_SERVICE_ROOT/runService.sh" ]]; then
        echo "PXREA服务启动脚本不存在: $PXREA_SERVICE_ROOT/runService.sh" >&2
        exit 1
    fi
    echo "启动本机PXREA RoboticsService..."
    (
        cd "$PXREA_SERVICE_ROOT"
        nohup bash ./runService.sh \
            >"/tmp/s10_roboticsservice_${USER}.log" 2>&1
    )
    for _ in $(seq 1 50); do
        ss -ltn 2>/dev/null | grep -q ':60061 ' && break
        sleep 0.1
    done
    if ! ss -ltn 2>/dev/null | grep -q ':60061 '; then
        echo "PXREA RoboticsService未能监听127.0.0.1:60061" >&2
        exit 1
    fi
fi

exec "$PYTHON_BIN" -u "$SCRIPT_DIR/s10_piper_vr_bridge.py" "$@"

