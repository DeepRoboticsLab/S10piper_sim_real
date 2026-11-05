#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MODE="${1:-all}"
case "$MODE" in
    all) VARIANTS=(realtime vr) ;;
    realtime|vr) VARIANTS=("$MODE") ;;
    *) echo 'Usage: ./build.sh [all|realtime|vr]' >&2; exit 2 ;;
esac
source /opt/ros/jazzy/setup.bash
cd "$SCRIPT_DIR"
export CMAKE_BUILD_PARALLEL_LEVEL=20
export MAKEFLAGS=-j20

# Build the official DEEPRobotics interface package as a project-local Jazzy
# overlay. Nothing is installed into /opt/ros or any other system directory.
DRDDS_SOURCE="$SCRIPT_DIR/vendor/drdds"
DRDDS_BUILD="$SCRIPT_DIR/build_drdds"
DRDDS_INSTALL="$SCRIPT_DIR/install_drdds"
DRDDS_LOG="$SCRIPT_DIR/log_drdds"
if [[ ! -f "$DRDDS_SOURCE/package.xml" ]]; then
    echo "Bundled drdds source is missing: $DRDDS_SOURCE" >&2
    exit 1
fi
colcon --log-base "$DRDDS_LOG" build \
    --base-paths "$DRDDS_SOURCE" \
    --build-base "$DRDDS_BUILD" --install-base "$DRDDS_INSTALL" \
    --packages-select drdds --executor sequential --parallel-workers 1
source "$DRDDS_INSTALL/setup.bash"

for VARIANT in "${VARIANTS[@]}"; do
    case "$VARIANT" in
        realtime) VR=OFF; BUILD_DIR=build; INSTALL_DIR=install; LOG_DIR=log ;;
        vr) VR=ON; BUILD_DIR=build_vr; INSTALL_DIR=install_vr; LOG_DIR=log_vr ;;
    esac
    echo "Building rl_deploy variant: $VARIANT"
    colcon --log-base "$LOG_DIR" build \
        --base-paths . --build-base "$BUILD_DIR" --install-base "$INSTALL_DIR" \
        --packages-select rl_deploy --executor sequential --parallel-workers 1 \
        --cmake-args -DBUILD_HARDWARE=ON -DBUILD_PLATFORM=arm \
        -DS10_POLICY_FILE=policy.onnx \
        "-DS10_POLICY_NAME=policy" \
        -DS10_PIPER_REALTIME=ON "-DS10_VR_CONTROL=$VR"
done
