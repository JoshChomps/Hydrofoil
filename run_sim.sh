#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"
VENV_PY=".venv/bin/python"
PY=""

if [ -x "$VENV_PY" ] && "$VENV_PY" -c "import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)" >/dev/null 2>&1; then
    PY="$VENV_PY"
else
    for cand in python3 python; do
        if command -v "$cand" >/dev/null 2>&1 && "$cand" -c "import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)" >/dev/null 2>&1; then
            PY="$cand"
            break
        fi
    done
fi

if [ -z "$PY" ]; then
    echo "Python 3.10+ not found. Install Python 3.10+ and re-run." >&2
    exit 1
fi

if ! "$PY" -c "import mujoco, numpy, scipy, matplotlib, plotly" >/dev/null 2>&1; then
    if [ "$PY" != "$VENV_PY" ]; then
        echo "Creating virtual environment in .venv..."
        "$PY" -m venv .venv
        PY="$VENV_PY"
    fi
    echo "Installing requirements..."
    "$PY" -m pip install -r requirements.txt
fi

echo ""
echo "Hydrofoil Sim Launcher (Python: $PY)"
echo "  [1] Web Dashboard + Boat Editor + 3D Viewer Launcher (default)"
echo "  [2] Manned 275 kg 3D Viewer - Supervisor + Waves + Noisy Sensors"
echo "  [3] Manned 275 kg 3D Viewer - Supervisor + Calm"
echo "  [4] Model 25 kg 3D Viewer   - Raw LQR/PID + Calm"
echo "  [5] Model Consistency Check"
echo "  [6] Run Pytest Suite"
echo ""
read -r -p "Select option [1-6] (Enter for 1): " CHOICE
CHOICE="${CHOICE:-1}"

case "$CHOICE" in
    1) exec "$PY" -m hydrofoil.sim_server ;;
    2) exec "$PY" -m hydrofoil.viewer --preset manned --supervisor --sensors noisy --waves ;;
    3) exec "$PY" -m hydrofoil.viewer --preset manned --supervisor ;;
    4) exec "$PY" -m hydrofoil.viewer ;;
    5) exec "$PY" -m hydrofoil.consistency ;;
    6) exec "$PY" -m pytest -v ;;
    *) echo "Invalid choice: $CHOICE" >&2; exit 1 ;;
esac
