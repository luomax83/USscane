@echo off
chcp 65001 > nul
title 创建上位机虚拟环境
cd /d "%~dp0"

echo ============================================
echo   创建上位机Python虚拟环境
echo ============================================
echo.

REM 检查Python是否安装
python --version >nul 2>&1
if errorlevel 1 (
    echo [错误] 未找到Python，请先安装Python 3.8或更高版本
    echo.
    pause
    exit /b 1
)

echo Python版本:
python --version
echo.

REM 检查pip是否可用
python -m pip --version >nul 2>&1
if errorlevel 1 (
    echo [警告] pip模块不可用
    echo 尝试使用ensurepip...
    python -m ensurepip --default-pip
)

echo.
echo 正在安装依赖包...
echo.

REM 安装依赖
.venv\Scripts\pip.exe install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple

if errorlevel 1 (
    echo.
    echo [错误] 依赖安装失败
    pause
    exit /b 1
)

echo.
echo ============================================
echo   环境设置完成！
echo ============================================
echo.
echo 可以运行 "启动上位机.bat" 启动程序
echo.
pause
