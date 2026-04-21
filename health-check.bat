@echo off
REM Health check script for Book RAG System

echo ========================================
echo Book RAG System - Health Check
echo ========================================
echo.

setlocal enabledelayedexpansion

REM Check backend venv
echo [1/5] Checking backend virtual environment...
if exist "backend\.venv\Scripts\activate.bat" (
    echo ✓ Virtual environment found
) else (
    echo ✗ Virtual environment NOT found - Run: npm run setup
    goto :error
)

REM Check backend .env
echo [2/5] Checking backend configuration...
if exist "backend\.env" (
    echo ✓ backend\.env found
    findstr /C:"ANTHROPIC_API_KEY" backend\.env >nul
    if !errorlevel! equ 0 (
        findstr /C:"sk-ant" backend\.env >nul
        if !errorlevel! equ 0 (
            echo ✓ ANTHROPIC_API_KEY configured
        ) else (
            echo ⚠ ANTHROPIC_API_KEY not set (translation may not work)
        )
    )
) else (
    echo ✗ backend\.env NOT found
    goto :error
)

REM Check frontend .env
echo [3/5] Checking frontend configuration...
if exist "frontend\.env" (
    echo ✓ frontend\.env found
    findstr /C:"REACT_APP_BACKEND_URL" frontend\.env >nul
    if !errorlevel! equ 0 (
        echo ✓ REACT_APP_BACKEND_URL configured
    ) else (
        echo ⚠ REACT_APP_BACKEND_URL not set
    )
) else (
    echo ✗ frontend\.env NOT found
    goto :error
)

REM Check if ports are available
echo [4/5] Checking if ports are available...
netstat -ano | findstr :8000 >nul
if !errorlevel! equ 0 (
    echo ⚠ Port 8000 is already in use
) else (
    echo ✓ Port 8000 is available
)

netstat -ano | findstr :3000 >nul
if !errorlevel! equ 0 (
    echo ⚠ Port 3000 is already in use
) else (
    echo ✓ Port 3000 is available
)

REM Check dependencies
echo [5/5] Checking dependencies...
if exist "backend\.venv\Lib\site-packages\fastapi" (
    echo ✓ FastAPI installed
) else (
    echo ✗ FastAPI not found - Run: npm run setup
)

if exist "frontend\node_modules\react" (
    echo ✓ React installed
) else (
    echo ✗ React not found - Run: npm run setup
)

echo.
echo ========================================
echo Health check complete!
echo ========================================
echo.
echo Next steps:
echo 1. Run: start-dev.bat
echo 2. Wait 10-15 seconds for services to start
echo 3. Open: http://localhost:3000
echo.
pause
exit /b 0

:error
echo.
echo ✗ Health check failed!
echo Run: npm run setup
echo.
pause
exit /b 1
