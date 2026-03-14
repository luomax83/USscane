@echo off
setlocal
cd /d "%~dp0host_app"

if exist ".venv\Scripts\pythonw.exe" (
    start "" ".venv\Scripts\pythonw.exe" "main.py"
    exit /b 0
)

if exist ".venv\Scripts\python.exe" (
    start "" ".venv\Scripts\python.exe" "main.py"
    exit /b 0
)

echo Python virtual environment not found: %cd%\.venv
pause
