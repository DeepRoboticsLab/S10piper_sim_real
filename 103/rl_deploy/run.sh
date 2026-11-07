#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/deploy.conf"
export S10_PIPER_WORKSPACE="$SCRIPT_DIR"
if [[ "${S10_PIPER_VARIANT:-standard}" == "vr" ]]; then
    export S10_PIPER_INSTALL_DIR="$SCRIPT_DIR/install_vr"
else
    export S10_PIPER_INSTALL_DIR="$SCRIPT_DIR/install"
fi
export S10_POLICY_PATH="$SCRIPT_DIR/policy/ppo/policy.onnx"
export S10_PIPER_MODE=real
export PIPER_SERVER_IP="${PIPER_SERVER_IP:-10.21.41.12}"
export PIPER_SERVER_PORT="${PIPER_SERVER_PORT:-9999}"

if [[ "${S10_PIPER_VARIANT:-standard}" == "vr" ]]; then
    export S10_PIPER_VR_PORT="${S10_PIPER_VR_PORT:-8089}"
    export S10_PIPER_VR_BASE="${S10_PIPER_VR_BASE:-1}"
    export S10_PIPER_VR_MODE="${S10_PIPER_VR_MODE:-0}"
    echo 'S10/Piper VR分层版：S10同款Orin PXREA中继→103，策略50Hz，机械臂目标100Hz，Y±0.50m。'
    echo '左摇杆上下=前后、左右=转向；右Grip=机械臂相对位移；右Trigger=夹爪；右B=Home。'
else
    echo 'S10/Piper实时版：103→Orin 机械臂目标 100 Hz，Y目标±0.50 m。'
fi
echo '保留关节限位、工作空间与反馈保护；请安全支撑、清场并准备急停。'

BINARY="${S10_PIPER_INSTALL_DIR}/rl_deploy/lib/rl_deploy/rl_deploy"
ROS_SETUP="${S10_ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
DRDDS_SETUP="$SCRIPT_DIR/install_drdds/setup.bash"
INSTALL_SETUP="${S10_PIPER_INSTALL_DIR}/setup.bash"
if [[ ! -f "$ROS_SETUP" || ! -f "$DRDDS_SETUP" || ! -f "$INSTALL_SETUP" || ! -x "$BINARY" || ! -s "$BINARY" ]]; then
    echo "S10 deployment is not built. Expected: $BINARY" >&2
    exit 1
fi
if ! command -v readelf >/dev/null || ! readelf -h "$BINARY" >/dev/null 2>&1; then
    echo "Invalid deployment ELF; SDK mode was not requested: $BINARY" >&2
    exit 1
fi

source "$ROS_SETUP"
source "$DRDDS_SETUP"
source "$INSTALL_SETUP"
set -u
exec 9>/tmp/rl_deploy.lock
if ! flock -n 9 || pgrep -x rl_deploy >/dev/null; then
    echo 'Another S10/Piper controller process is already running.' >&2
    exit 1
fi

sdk_request_mode() {
    local command="$1" deadline="${2:-20}" output status result
    if output=$(timeout --kill-after=2 "$deadline" ros2 service call \
            /MOTION_SDK_MODE drdds/srv/StdSrvInt32 "{command: $command}" 2>&1); then
        status=0
    else
        status=$?
    fi
    printf '%s\n' "$output"
    if [[ "$status" -ne 0 ]]; then
        echo "[SDK] 服务调用失败或超时（退出码=$status）；未启动策略。" >&2
        return "$status"
    fi
    result=$(printf '%s\n' "$output" | sed -nE \
        's/.*result[=:][[:space:]]*(-?[0-9]+).*/\1/p' | tail -n 1)
    if [[ ! "$result" =~ ^[0-9]+$ ]]; then
        echo '[SDK] 响应result无效；未启动策略。' >&2
        return 65
    fi
}

sdk_mode_entered=0
cleanup() {
    local status=$?
    trap - EXIT INT TERM
    if [[ "$sdk_mode_entered" -eq 1 ]]; then
        sdk_request_mode 0 8 >/dev/null 2>&1 || true
    fi
    exit "$status"
}
trap cleanup EXIT INT TERM

echo 'Entering S10 SDK mode at 200 Hz...'
sdk_request_mode 200 20
sdk_mode_entered=1
export S10_REAL_READ_ONLY=0
# Keep this shell alive so cleanup releases SDK mode after the controller exits.
"$BINARY" "$@"
