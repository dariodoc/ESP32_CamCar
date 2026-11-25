@echo off
setlocal
set "VENV_PY=%USERPROFILE%\.venvs\mujoco-studio\Scripts\python.exe"
set "STUDIOD=%USERPROFILE%\.gemini\config\plugins\robotics\skills\mujoco-studio-sidecar\scripts\studiod.py"
set "SIM_DIR=%~dp0"
if "%SIM_DIR:~-1%"=="\" set "SIM_DIR=%SIM_DIR:~0,-1%"

if not exist "%VENV_PY%" (
    echo [ERROR] No se encontro el entorno virtual en: %VENV_PY%
    pause
    exit /b 1
)

echo Iniciando MuJoCo Studio para ESP32 4WD Mecanum CamCar...
echo Abre http://localhost:8080/ en tu navegador o el panel MuJoCo Studio en Antigravity.
echo Tambien puedes conectar Main.py usando IP: 127.0.0.1
echo Presiona Ctrl+C para detener la simulacion.
echo.
"%VENV_PY%" "%STUDIOD%" --workspace_dir="%SIM_DIR%"
pause
