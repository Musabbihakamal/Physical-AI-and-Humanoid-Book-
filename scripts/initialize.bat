@echo off
REM Project Initialization Script for Windows
REM This script sets up the complete project environment

echo 🚀 Initializing Book + RAG Bot + Multi-Agent System...

REM Check if Python 3.11+ is available
python --version >nul 2>&1
if errorlevel 1 (
    echo ❌ Python is not installed. Please install Python 3.11+
    exit /b 1
)

for /f "tokens=2" %%i in ('python --version 2^>^&1') do set PYTHON_VERSION=%%i
echo Python version: %PYTHON_VERSION%

REM Check if Node.js 18+ is available
node --version >nul 2>&1
if errorlevel 1 (
    echo ❌ Node.js is not installed. Please install Node.js 18+
    exit /b 1
)

for /f "tokens=1" %%i in ('node --version') do set NODE_VERSION=%%i
set NODE_VERSION=%NODE_VERSION:v=%
echo Node.js version: %NODE_VERSION%

echo ✅ Prerequisites check passed

REM Install backend dependencies
echo 📦 Installing backend dependencies...
cd backend
if exist "requirements.txt" (
    python -m venv venv
    call venv\Scripts\activate.bat
    python -m pip install --upgrade pip
    pip install -r requirements.txt
    echo ✅ Backend dependencies installed
) else (
    echo ⚠️  requirements.txt not found in backend
)
cd ..

REM Install frontend dependencies
echo 📦 Installing frontend dependencies...
cd frontend
if exist "package.json" (
    npm install
    echo ✅ Frontend dependencies installed
) else (
    echo ⚠️  package.json not found in frontend
)
cd ..

REM Create example environment files if they don't exist
echo 📝 Creating environment configuration...
cd backend
if not exist ".env" (
    if exist ".env.example" (
        copy .env.example .env
        echo ✅ Created backend .env file
    )
)
cd ..\frontend
if not exist ".env" (
    if exist ".env.example" (
        copy .env.example .env
        echo ✅ Created frontend .env file
    )
)
cd ..

REM Initialize database if needed
echo 💾 Initializing database...
cd backend
if exist "src\database\database.py" (
    call venv\Scripts\activate.bat
    python -c "
import sys
import os
sys.path.insert(0, os.path.join(os.getcwd(), 'src'))
try:
    from database.database import init_db
    init_db()
    print('✅ Database initialized')
except ImportError:
    print('⚠️  Could not initialize database automatically')
except Exception as e:
    print(f'⚠️  Database initialization error: {e}')
"
)
cd ..

echo 🎉 Project initialization complete!
echo.
echo 📋 Next steps:
echo 1. Update environment variables in backend/.env and frontend/.env
echo 2. Start backend: cd backend && python -m src.main (or use start_backend.py)
echo 3. Start frontend: cd frontend && npm start
echo 4. Access the application at http://localhost:3000