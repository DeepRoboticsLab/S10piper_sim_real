#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export S10_PIPER_VARIANT=vr
exec "$SCRIPT_DIR/run.sh" "$@"
