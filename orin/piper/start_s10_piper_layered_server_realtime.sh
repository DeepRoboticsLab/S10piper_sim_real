#!/usr/bin/env bash
set -euo pipefail
PIPER_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$PIPER_ROOT/deploy.conf"
export PIPER_ROOT PIPER_CAN_PORT
export PIPER_FIND_CAN="$PIPER_ROOT/bin/find_can.sh"
export PIPER_CAN_UP="$PIPER_ROOT/bin/can_up.sh"

# The release does not bundle a virtual environment. Prefer one when the user
# created it, otherwise use the normal system/user Python installation.
if [[ -n "${PIPER_PYTHON:-}" ]]; then
    PYTHON_BIN="$PIPER_PYTHON"
elif [[ -x "$PIPER_ROOT/.venv/bin/python" ]]; then
    PYTHON_BIN="$PIPER_ROOT/.venv/bin/python"
else
    PYTHON_BIN="$(command -v python3 || true)"
fi
if [[ -z "$PYTHON_BIN" || ! -x "$PYTHON_BIN" ]]; then
    echo 'Python 3 was not found; install python3 first.' >&2
    exit 1
fi
if ! "$PYTHON_BIN" -c 'import numpy, piper_sdk' >/dev/null 2>&1; then
    echo "Required Python packages are missing for: $PYTHON_BIN" >&2
    echo 'Install numpy and piper_sdk for this interpreter, or set PIPER_PYTHON to an interpreter that already has them.' >&2
    exit 1
fi

# Prevent two arm services from controlling the same Piper simultaneously.
exec 9>/tmp/s10_piper_arm_exclusive.lock
if ! flock -n 9; then
    echo 'Another MIT/layered arm service is running; stop it cleanly first.' >&2
    exit 1
fi
if pgrep -f '[p]ython[^ ]* .*app/(s10_piper_mit_server|piper_arm_server)\.py' >/dev/null; then
    echo 'An existing Piper service is still running; stop it cleanly first.' >&2
    exit 1
fi

cd "$PIPER_ROOT"
exec "$PYTHON_BIN" app/piper_arm_server.py \
    --port "$PIPER_SERVER_PORT" \
    --allow-ip "$PIPER_ALLOWED_IPS" \
    --speed 50 \
    --max-step 0.08 \
    --training-home-speed 50 \
    --training-home-max-step 0.08 \
    --link6-rate 100 \
    --auto-recover \
    "$@"
