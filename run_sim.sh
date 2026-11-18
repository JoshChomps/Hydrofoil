#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"
VENV_PY=".venv/bin/python"

if [ ! -x "$VENV_PY" ]; then
    if command -v python3 >/dev/null 2>&1; then
        SYS_PY=python3
    elif command -v python >/dev/null 2>&1; then
        SYS_PY=python
    else
        echo "Python 3 not found. Install Python 3.10+ and re-run." >&2
        exit 1
    fi
    echo "Creating virtual environment in .venv..."
    "$SYS_PY" -m venv .venv
    echo "Installing requirements..."
    "$VENV_PY" -m pip install -r requirements.txt
elif ! "$VENV_PY" -c "import mujoco, numpy, scipy, matplotlib, plotly" >/dev/null 2>&1; then
    echo "Installing missing requirements..."
    "$VENV_PY" -m pip install -r requirements.txt
fi

echo ""
echo "Hydrofoil Sim Launcher"
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
    1) exec "$VENV_PY" -m hydrofoil.sim_server ;;
    2) exec "$VENV_PY" -m hydrofoil.viewer --preset manned --supervisor --sensors noisy --waves ;;
    3) exec "$VENV_PY" -m hydrofoil.viewer --preset manned --supervisor ;;
    4) exec "$VENV_PY" -m hydrofoil.viewer ;;
    5) exec "$VENV_PY" -m hydrofoil.consistency ;;
    6) exec "$VENV_PY" -m pytest -v ;;
    *) echo "Invalid choice: $CHOICE" >&2; exit 1 ;;
esac
