@echo off
REM Start Backend Server
echo Starting backend server on port 8001...
call .venv\Scripts\activate
set PORT=8001
python -m src.main
