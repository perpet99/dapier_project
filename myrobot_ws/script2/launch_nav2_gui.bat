@echo off
setlocal

REM Launch Nav2 via the shared shell script in WSL Ubuntu-24.04
where wsl >nul 2>&1
if errorlevel 1 (
  echo [ERROR] wsl command not found. Please run this on Windows with WSL installed.
  exit /b 1
)

echo Starting launcher script in WSL (Ubuntu-24.04)...
echo Press Ctrl+C in this window to stop the launch.
echo.

wsl -d Ubuntu-24.04 -- bash -lc "cd /home/perpet/gazebo && ./launch_nav2_gui.sh"

set EXIT_CODE=%ERRORLEVEL%
echo.
echo Nav2 launch exited with code %EXIT_CODE%.
if not "%EXIT_CODE%"=="0" (
  echo.
  echo Launch failed. Keep this window and check error lines above.
  pause
)
exit /b %EXIT_CODE%
