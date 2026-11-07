#!/usr/bin/env bash
set -euo pipefail
TASK_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TASK_MODE="${1:-sim}"
if (( $# )); then shift; fi
case "$TASK_MODE" in
    sim)
        exec "$TASK_ROOT/sim/run.sh" "$@"
        ;;
    real|real-vr)
        if (( $# )); then echo 'real/real-vr 不接受附加参数；按说明配置远端部署。' >&2; exit 2; fi
        echo '实物入口：会申请103的SDK控制。请先确认支撑、清场、急停和Orin反馈；退出其他控制程序。'
        if [[ "$TASK_MODE" == real ]]; then
            exec ssh -tt user@10.21.33.103 \
                'cd /home/user/rl_deploy && ./run.sh'
        else
            exec ssh -tt user@10.21.33.103 \
                'cd /home/user/rl_deploy && ./run_vr.sh'
        fi
        ;;
    orin-arm)
        if (( $# )); then exit 2; fi
        echo '实物机械臂入口：反馈有效后将自动低速回零，请先清空完整运动路径。'
        exec ssh -tt user@10.21.41.12 \
            'cd /home/user/piper && ./start_s10_piper_layered_server_realtime.sh'
        ;;
    orin-vr)
        if (( $# )); then exit 2; fi
        exec ssh -tt user@10.21.41.12 \
            'cd /home/user/pico_bridge && ./run_vr_bridge.sh'
        ;;
    help|-h|--help)
        echo '同一份S10/Piper策略/状态机/键盘/分层IK；仅切换实物或本机仿真接口。'
        echo './run.sh sim [--headless] [--port-base 11880] [--physics-substeps 2]'
        echo './run.sh real       # 实物分层键盘版，SSH至103'
        echo './run.sh real-vr    # 原实物VR版，SSH至103'
        echo './run.sh orin-arm   # 实物共用机械臂服务（启动会回零）'
        echo './run.sh orin-vr    # 实物S10/Piper Pico桥接（默认1.0倍）'
        ;;
    *) echo "未知接口: $TASK_MODE；用 ./run.sh help 查看。" >&2; exit 2;;
esac
