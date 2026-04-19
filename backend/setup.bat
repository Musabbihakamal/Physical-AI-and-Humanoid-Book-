@echo off
REM Complete Backend Setup Script
echo ========================================
echo Backend Setup
echo ========================================
echo.

echo Step 1: Creating virtual environment...
if not exist .venv (
    python -m venv .venv
)

echo Step 2: Activating virtual environment...
call .venv\Scripts\activate
if errorlevel 1 (
    echo ERROR: Failed to activate virtual environment
    pause
    exit /b 1
)

echo Step 3: Upgrading pip...
python -m pip install --upgrade pip

echo Step 4: Installing dependencies...
pip install -r requirements.txt
if errorlevel 1 (
    echo ERROR: Failed to install dependencies
    pause
    exit /b 1
)

echo.
echo ========================================
echo Setup Complete!
echo ========================================
echo.
echo To start the backend, run: start.bat
echo To ingest content, run: ingest.bat
pause
