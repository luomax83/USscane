@echo off
chcp 65001 > nul
title 360度扫描超声上位机
cd /d "%~dp0"

echo ============================================
echo   360度扫描超声上位机
echo ============================================
echo.

REM 检查虚拟环境
if not exist ".venv\Scripts\python.exe" (
    echo [错误] 虚拟环境不存在，请先运行 setup.bat 创建环境
    echo.
    pause
    exit /b 1
)

REM 激活虚拟环境并运行
echo 正在启动上位机程序...
echo.
.venv\Scripts\python.exe main.py

if errorlevel 1 (
    echo.
    echo [错误] 程序异常退出，错误代码: %errorlevel%
    pause
)

endlocal
