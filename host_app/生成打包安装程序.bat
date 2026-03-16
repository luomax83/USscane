@echo off
setlocal
chcp 65001 > nul
set "ROOT=%~dp0"
powershell -NoProfile -ExecutionPolicy Bypass -File "%ROOT%build_installer.ps1"
exit /b %errorlevel%
