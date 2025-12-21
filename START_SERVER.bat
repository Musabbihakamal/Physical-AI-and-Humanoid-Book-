@echo off
echo.
echo ===============================================
echo Starting Backend Server with OAuth Support
echo ===============================================
echo.

REM Navigate to backend directory
cd /d "%~dp0\backend"

REM Check if venv exists, if not create it
if not exist "venv" (
    echo Creating virtual environment...
    python -m venv venv
)

REM Activate virtual environment
call venv\Scripts\activate.bat

REM Install dependencies if requirements.txt exists
if exist "requirements.txt" (
    echo Installing dependencies...
    pip install -r requirements.txt
)

REM Start the backend server
echo Starting backend server on port 8000...
uvicorn src.api.main:app --host 0.0.0.0 --port 8000 --reload

pause