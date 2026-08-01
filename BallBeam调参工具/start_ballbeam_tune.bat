@echo off
setlocal EnableExtensions
cd /d "%~dp0"

echo ============================================================
echo   BallBeam interactive tuning mode
echo ============================================================
echo.
echo   Before continuing:
echo     1. Manually level the beam to horizontal. The UNO will be
echo        reset on start, and that pose becomes command zero.
echo     2. K230D must already be running /sdcard/main.py.
echo     3. The ball must NOT be on the beam.
echo.
echo   After start: place the ball at the LEFT end of the beam.
echo   The closed loop starts automatically.
echo.
echo   Commands (type and press Enter at any time):
echo     PID 10 2 10     - set KP KI KD
echo     RATE 1500       - set max pulse rate
echo     ACCEL 8000      - set acceleration
echo     DEADBAND 5      - set deadband mm
echo     SIGN 1 or -1    - flip control direction
echo     STATUS          - show current status
echo     RETURN          - return to zero
echo     DISARM          - lock motor output
echo     quit            - exit and send DISARM
echo.
pause

.venv\Scripts\python.exe -u tools\pc_serial_bridge.py --k230 COM7 --uno COM9 --pid 10 2 10 --reset-uno --no-periodic-status

echo.
echo Bridge exited.
pause
