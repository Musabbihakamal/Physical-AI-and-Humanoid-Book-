@echo off
REM Start development environment for Book RAG System

echo Starting Book RAG System...
echo.

REM Check if backend venv exists
if not exist "backend\.venv\Scripts\activate.bat" (
    echo Error: Virtual environment not found. Run 'npm run setup' first.
    pause
    exit /b 1
)

REM Start backend and frontend concurrently
echo Starting backend on port 8001...
echo Starting frontend on port 3000...
echo.

REM Open two terminal windows
start "Backend" cmd /k "cd backend && .venv\Scripts\activate.bat && set PORT=8001 && python -m src.main"
start "Frontend" cmd /k "cd frontend && npm start"

echo.
echo Services starting...
echo Backend: http://localhost:8001
echo Frontend: http://localhost:3000
echo.
pause
