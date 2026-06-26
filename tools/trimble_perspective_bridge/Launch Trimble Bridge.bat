@echo off
setlocal

set "APP_DIR=%~dp0"
cd /d "%APP_DIR%"

where pyw.exe >nul 2>nul
if %errorlevel%==0 (
    start "" pyw.exe "%APP_DIR%windows_app.py"
    exit /b 0
)

start "" py.exe "%APP_DIR%windows_app.py"
