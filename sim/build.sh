#!/usr/bin/env bash
set -euo pipefail
TASK_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_ROOT="$(cd -- "$TASK_ROOT/.." && pwd)"
cmake -S "$PACKAGE_ROOT" -B "$PACKAGE_ROOT/build"
cmake --build "$PACKAGE_ROOT/build" --target rl_deploy -j20
cmake --install "$PACKAGE_ROOT/build" --prefix "$TASK_ROOT"
