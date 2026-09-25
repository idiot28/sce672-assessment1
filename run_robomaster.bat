@echo off
setlocal enabledelayedexpansion
title DJI RoboMaster EP - Autonomous Control and Teleoperation
color 0B

:: Set working directory
set "SCRIPT_DIR=%~dp0"
cd /d "%SCRIPT_DIR%"

:CHECK_RUNTIME
:: Check if real Windows Python is installed (test with actual execution to bypass Windows Store stub)
set "RUN_MODE=WSL"
set "PY_CMD=wsl -d Ubuntu-22.04 python3"

python -c "import sys" >nul 2>&1
if %ERRORLEVEL% equ 0 (
    set "RUN_MODE=WINDOWS"
    set "PY_CMD=python"
)

:MENU
cls
echo ==============================================================================
echo           DJI RoboMaster EP Core - Control and Perception Suite
echo                SCE 672: Advanced Robotics - KFUPM
echo ==============================================================================
echo.
echo  Detected Environment: %RUN_MODE%
echo.
echo  Please select an operation to execute:
echo.
echo    [1] Autonomous Pick-and-Place  -- Wired USB / RNDIS
echo    [2] Autonomous Pick-and-Place  -- Direct Wi-Fi / AP
echo    [3] Keyboard Teleop and Telemetry  -- Wired USB / RNDIS
echo    [4] Keyboard Teleop and Telemetry  -- Direct Wi-Fi / AP
echo.
echo    [5] Arm and Gripper Calibration Test (Test reach, height, grip on bottle)
echo    [6] Test Autonomous Pick-and-Place in SIMULATION / MOCK Mode (No Robot)
echo    [7] Benchmark and Ping Robot Gateways
echo    [8] Exit
echo.
echo ==============================================================================
set /p "CHOICE=Enter your choice [1-8]: "

if "%CHOICE%"=="1" goto OPTION_1
if "%CHOICE%"=="2" goto OPTION_2
if "%CHOICE%"=="3" goto OPTION_3
if "%CHOICE%"=="4" goto OPTION_4
if "%CHOICE%"=="5" goto OPTION_5
if "%CHOICE%"=="6" goto OPTION_6
if "%CHOICE%"=="7" goto OPTION_7
if "%CHOICE%"=="8" goto OPTION_8

echo [!] Invalid selection. Please choose a number from 1 to 8.
pause
goto MENU

:OPTION_1
echo.
echo [*] Launching Autonomous Pick-and-Place over Wired RNDIS...
if "%RUN_MODE%"=="WINDOWS" (
    python ep_autonomous_pick_and_place.py --conn_type rndis
) else (
    wsl -d Ubuntu-22.04 bash -c "cd '/mnt/d/phd/sce672 - advanced robotics/project' && python3 ep_autonomous_pick_and_place.py --conn_type rndis"
)
goto END

:OPTION_2
echo.
echo [*] Launching Autonomous Pick-and-Place over Direct Wi-Fi AP...
if "%RUN_MODE%"=="WINDOWS" (
    python ep_autonomous_pick_and_place.py --conn_type ap
) else (
    wsl -d Ubuntu-22.04 bash -c "cd '/mnt/d/phd/sce672 - advanced robotics/project' && python3 ep_autonomous_pick_and_place.py --conn_type ap"
)
goto END

:OPTION_3
echo.
echo [*] Launching Keyboard Teleop and 10Hz Telemetry Logger over RNDIS...
echo     Controls: W/S = Forward/Back, A/D = Strafe, Q/E = Yaw, Space = Stop, Esc = Exit
if "%RUN_MODE%"=="WINDOWS" (
    python ep_teleop_sdk.py --conn_type rndis --duration 30
) else (
    wsl -d Ubuntu-22.04 bash -c "cd '/mnt/d/phd/sce672 - advanced robotics/project' && python3 ep_teleop_sdk.py --conn_type rndis --duration 30"
)
goto END

:OPTION_4
echo.
echo [*] Launching Keyboard Teleop and 10Hz Telemetry Logger over Direct Wi-Fi AP...
echo     Controls: W/S = Forward/Back, A/D = Strafe, Q/E = Yaw, Space = Stop, Esc = Exit
if "%RUN_MODE%"=="WINDOWS" (
    python ep_teleop_sdk.py --conn_type ap --duration 30
) else (
    wsl -d Ubuntu-22.04 bash -c "cd '/mnt/d/phd/sce672 - advanced robotics/project' && python3 ep_teleop_sdk.py --conn_type ap --duration 30"
)
goto END

:OPTION_5
echo.
echo [*] Launching Arm and Gripper Grasp Calibration Test...
echo     Place robot 15-25 cm in front of the bottle before starting.
set /p "CONN_CALIB=Select connection type (1=RNDIS/USB, 2=AP/Wi-Fi) [1]: "
set "CALIB_MODE=rndis"
if "%CONN_CALIB%"=="2" set "CALIB_MODE=ap"
if "%RUN_MODE%"=="WINDOWS" (
    python test_arm_grasp.py --conn_type %CALIB_MODE%
) else (
    wsl -d Ubuntu-22.04 bash -c "cd '/mnt/d/phd/sce672 - advanced robotics/project' && python3 test_arm_grasp.py --conn_type %CALIB_MODE%"
)
goto END

:OPTION_6
echo.
echo [*] Launching Autonomous Pick-and-Place in MOCK SIMULATION Mode...
if "%RUN_MODE%"=="WINDOWS" (
    python ep_autonomous_pick_and_place.py --mock
) else (
    wsl -d Ubuntu-22.04 bash -c "cd '/mnt/d/phd/sce672 - advanced robotics/project' && python3 ep_autonomous_pick_and_place.py --mock --no-gui"
)
goto END

:OPTION_7
echo.
echo [*] Testing connection to robot gateways...
echo.
echo --- Testing Wired RNDIS Gateway: 192.168.42.2 ---
ping -n 4 192.168.42.2
echo.
echo --- Testing Direct Hotspot AP Gateway: 192.168.12.1 ---
ping -n 4 192.168.12.1
echo.
pause
goto MENU

:OPTION_8
echo Exiting...
exit /b 0

:END
echo.
echo ==============================================================================
echo [*] Operation completed.
echo ==============================================================================
pause
goto MENU
