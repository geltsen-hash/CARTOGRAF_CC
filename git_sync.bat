@echo off
setlocal enabledelayedexpansion

echo ===================================================
echo [GIT SYNC] Local Commit + Remote Push
echo ===================================================

set MSG=%~1
if "%MSG%"=="" (
    set /p MSG="Enter commit message: "
)

if "%MSG%"=="" (
    echo [ERROR] Commit message cannot be empty.
    exit /b 1
)

echo.
echo 1. Staging files...
git add -A

echo 2. Committing locally...
git commit -m "%MSG%"
if %ERRORLEVEL% neq 0 (
    echo [NOTE] No new changes to commit or commit failed.
)

echo.
echo 3. Pushing to origin main...
git push origin main
if %ERRORLEVEL% neq 0 (
    echo.
    echo [WARNING] Push failed (remote error or network issue).
    echo Your commit is saved locally. Run git_push.bat when connection is restored.
    exit /b %ERRORLEVEL%
)

echo.
echo [SUCCESS] Synchronized: Committed and pushed to origin/main.
