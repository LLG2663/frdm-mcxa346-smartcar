@echo off
REM Starts a tiny local web server so the browser exposes the Web Serial API.
REM Web Serial is NOT available on file:// pages - you must use http://localhost.
cd /d "%~dp0"

where python >nul 2>nul
if errorlevel 1 echo [X] python not found in PATH. Install Python, or run this from the RT-Thread Env console. & pause & exit /b 1

echo [1/2] starting http server on port 8000 (a new window opens, close it to stop)
start "path-planner-http" cmd /k python -m http.server 8000

timeout /t 2 >nul
echo [2/2] opening http://localhost:8000/smartcar-host.html
start "" http://localhost:8000/smartcar-host.html
