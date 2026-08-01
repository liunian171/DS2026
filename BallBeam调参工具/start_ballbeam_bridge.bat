@echo off
setlocal EnableExtensions
chcp 65001 >nul

cd /d "%~dp0"

set "K230_PORT=COM7"
set "UNO_PORT=COM9"

if not "%~1"=="" set "K230_PORT=%~1"
if not "%~2"=="" set "UNO_PORT=%~2"

set "PYTHON_EXE=%~dp0.venv\Scripts\python.exe"
set "BRIDGE_SCRIPT=%~dp0tools\pc_serial_bridge.py"

echo ============================================================
echo   BallBeam K230D -^> PC -^> UNO bridge
echo ============================================================
echo K230D port : %K230_PORT%
echo UNO port   : %UNO_PORT%
echo.

if not exist "%PYTHON_EXE%" (
    echo [ERROR] Python virtual environment was not found:
    echo         "%PYTHON_EXE%"
    echo.
    echo Create it with:
    echo   python -m venv .venv
    echo   .venv\Scripts\python.exe -m pip install -r requirements.txt
    echo.
    pause
    exit /b 2
)

if not exist "%BRIDGE_SCRIPT%" (
    echo [ERROR] Bridge script was not found:
    echo         "%BRIDGE_SCRIPT%"
    echo.
    pause
    exit /b 2
)

powershell -NoProfile -Command "$ports = @(Get-CimInstance Win32_SerialPort | ForEach-Object DeviceID); if ($ports -contains '%K230_PORT%') { exit 0 } else { exit 1 }"
if errorlevel 1 (
    echo [ERROR] K230D port %K230_PORT% was not found.
    echo Check Device Manager or run this BAT with explicit ports:
    echo   %~nx0 COM7 COM9
    echo.
    pause
    exit /b 3
)

powershell -NoProfile -Command "$ports = @(Get-CimInstance Win32_SerialPort | ForEach-Object DeviceID); if ($ports -contains '%UNO_PORT%') { exit 0 } else { exit 1 }"
if errorlevel 1 (
    echo [ERROR] UNO port %UNO_PORT% was not found.
    echo Check Device Manager or run this BAT with explicit ports:
    echo   %~nx0 COM7 COM9
    echo.
    pause
    exit /b 3
)

echo Before continuing:
echo   1. K230D must already be running /sdcard/main.py.
echo   2. The K230D output should contain B,frame,position,state lines.
echo   3. Disconnect or close the CanMV serial connection to release %K230_PORT%.
echo   4. Close Arduino Serial Monitor to release %UNO_PORT%.
echo   5. Manually level the beam before entering the three PID gains.
echo.
echo Opening %UNO_PORT% resets the UNO and defines that startup pose as command zero.
echo After all three gains are entered, the bridge sends them automatically.
echo The UNO then runs the motor on every vision frame (state^>=1) until the
echo ball stabilizes at the center.
echo.

set "PID_KP="
set "PID_KI="
set "PID_KD="
set /p "PID_KP=KP: "
set /p "PID_KI=KI: "
set /p "PID_KD=KD: "

if "%PID_KP%"=="" (
    echo [ERROR] KP is required.
    pause
    exit /b 4
)
if "%PID_KI%"=="" (
    echo [ERROR] KI is required.
    pause
    exit /b 4
)
if "%PID_KD%"=="" (
    echo [ERROR] KD is required.
    pause
    exit /b 4
)

echo.
echo Starting automatic closed loop with PID %PID_KP% %PID_KI% %PID_KD% ...

"%PYTHON_EXE%" "%BRIDGE_SCRIPT%" --k230 "%K230_PORT%" --uno "%UNO_PORT%" --quiet-uno-status --pid "%PID_KP%" "%PID_KI%" "%PID_KD%"
set "BRIDGE_EXIT=%ERRORLEVEL%"

echo.
if "%BRIDGE_EXIT%"=="0" (
    echo [OK] Bridge stopped. DISARM was requested before the ports closed.
) else (
    echo [ERROR] Bridge exited with code %BRIDGE_EXIT%.
    echo Make sure CanMV and Arduino Serial Monitor are not holding the ports.
)
echo.
pause
exit /b %BRIDGE_EXIT%
