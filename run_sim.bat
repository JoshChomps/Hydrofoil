@echo off
setlocal enabledelayedexpansion
pushd "%~dp0"

set "VENV_PY=%~dp0.venv\Scripts\python.exe"

if not exist "%VENV_PY%" (
    echo Checking Python...
    where py >nul 2>&1
    if %errorlevel%==0 (
        set "SYS_PY=py -3"
    ) else (
        where python >nul 2>&1
        if %errorlevel%==0 (
            set "SYS_PY=python"
        ) else (
            echo Python 3 was not found on PATH. Install Python 3.10+ and re-run.
            pause
            popd
            exit /b 1
        )
    )
    echo Creating virtual environment in .venv...
    !SYS_PY! -m venv .venv
    if errorlevel 1 (
        echo Failed to create .venv.
        pause
        popd
        exit /b 1
    )
    echo Installing requirements...
    "%VENV_PY%" -m pip install -r requirements.txt
    if errorlevel 1 (
        echo Failed to install requirements.
        pause
        popd
        exit /b 1
    )
) else (
    "%VENV_PY%" -c "import mujoco, numpy, scipy, matplotlib, plotly" >nul 2>&1
    if errorlevel 1 (
        echo Installing missing requirements...
        "%VENV_PY%" -m pip install -r requirements.txt
        if errorlevel 1 (
            pause
            popd
            exit /b 1
        )
    )
)

echo.
echo Hydrofoil Sim Launcher
echo   [1] Manned 275 kg - Supervisor + Waves + Noisy Sensors (default)
echo   [2] Manned 275 kg - Supervisor + Calm
echo   [3] Model 25 kg   - Raw LQR/PID + Calm
echo   [4] Model Consistency Check
echo   [5] Run Pytest Suite
echo.
set "CHOICE=1"
set /p "CHOICE=Select option [1-5] (Enter for 1): "

if "%CHOICE%"=="1" (
    "%VENV_PY%" -m hydrofoil.viewer --preset manned --supervisor --sensors noisy --waves
) else if "%CHOICE%"=="2" (
    "%VENV_PY%" -m hydrofoil.viewer --preset manned --supervisor
) else if "%CHOICE%"=="3" (
    "%VENV_PY%" -m hydrofoil.viewer
) else if "%CHOICE%"=="4" (
    "%VENV_PY%" -m hydrofoil.consistency
    pause
) else if "%CHOICE%"=="5" (
    "%VENV_PY%" -m pytest -v
    pause
) else (
    echo Invalid choice: %CHOICE%
    pause
    popd
    exit /b 1
)

if errorlevel 1 (
    echo.
    echo Command exited with an error.
    pause
)
popd
