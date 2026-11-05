#!/usr/bin/env bash
set -euo pipefail
TASK_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
case "${1:-sim}" in
    sim) exec "$TASK_ROOT/sim/build.sh" ;;
    real) exec "$TASK_ROOT/103/rl_deploy/build.sh" realtime ;;
    real-vr) exec "$TASK_ROOT/103/rl_deploy/build.sh" vr ;;
    -h|--help|help)
        echo './build.sh sim       # 默认：本机仿真，build/rl_deploy'
        echo './build.sh real      # 在103上编译键盘分层版'
        echo './build.sh real-vr   # 在103上只编译VR版'
        echo '每次只编译所选模式，-j20。';;
    *) echo 'Usage: ./build.sh sim|real|real-vr' >&2; exit 2 ;;
esac
