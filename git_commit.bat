@echo off
setlocal enabledelayedexpansion

set MSG=%~1
if "%MSG%"=="" (
    set /p MSG="Enter commit message: "
)

if "%MSG%"=="" (
    echo [ERROR] Commit message cannot be empty.
    exit /b 1
)

echo.
echo Staging changes...
git add -A

echo Creating local commit...
git commit -m "%MSG%"
if %ERRORLEVEL% neq 0 (
    echo [ERROR] Commit failed or no changes to commit.
    exit /b %ERRORLEVEL%
)

echo.
echo [SUCCESS] Local commit created successfully.
