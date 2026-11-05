#!/usr/bin/env bash
set -euo pipefail
TASK_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -z "${SIM_PYTHON:-}" ]]; then
    if [[ -n "${CONDA_PREFIX:-}" ]]; then
        if [[ ! -x "$CONDA_PREFIX/bin/python" ]]; then
            echo 'No Python in the active Conda environment; activate the simulation environment.' >&2
            exit 2
        fi
        SIM_PYTHON="$CONDA_PREFIX/bin/python"
    elif [[ -x "$TASK_ROOT/../.venv-sim/bin/python" ]]; then
        SIM_PYTHON="$TASK_ROOT/../.venv-sim/bin/python"
    else
        SIM_PYTHON=python3
    fi
fi
echo "[SIM] Python: $SIM_PYTHON"
if ! "$SIM_PYTHON" -c 'import mujoco,numpy,typing_extensions'; then
    echo 'Missing simulation dependencies. Activate your Conda environment and install sim/requirements-lock.txt.' >&2
    exit 2
fi
if [[ ! -x "$TASK_ROOT/bin/rl_deploy" ]]; then
    "$TASK_ROOT/build.sh"
fi
echo '[SIM] Loopback only; no physical ROS/CAN/SSH connection.'
exec "$SIM_PYTHON" "$TASK_ROOT/simulation/run_pair.py" "$@"
