@echo off
REM Physical AI & Humanoid Robotics Book - Windows Setup Script
REM This script automates the setup and deployment of the complete system

echo ===========================================
echo Physical AI & Humanoid Robotics Book Setup
echo ===========================================

REM Check prerequisites
echo Checking prerequisites...

where docker >nul 2>nul
if errorlevel 1 (
    echo Error: Docker is not installed or not in PATH. Please install Docker Desktop for Windows.
    pause
    exit /b 1
)

where docker-compose >nul 2>nul
if errorlevel 1 (
    echo Error: Docker Compose is not installed or not in PATH.
    pause
    exit /b 1
)

where git >nul 2>nul
if errorlevel 1 (
    echo Error: Git is not installed or not in PATH.
    pause
    exit /b 1
)

echo All prerequisites found!

REM Check if running in a repository directory
if not exist "package.json" (
    echo Repository files not found. Please run this script from the project directory.
    pause
    exit /b 1
)

REM Check for .env file
if not exist ".env" (
    echo Creating .env file from example...
    if exist ".env.example" (
        copy .env.example .env
        echo Created .env file. Please edit it to add your API keys and database URLs.
        echo After editing, run this script again.
        pause
        exit /b 0
    ) else (
        echo Error: .env.example file not found. Cannot create .env file.
        pause
        exit /b 1
    )
)

REM Build and start the system
echo Building and starting the system...

REM Pull latest images
docker-compose pull

REM Build images
docker-compose build

REM Start services
docker-compose up -d

echo Waiting for services to start...
timeout /t 10 /nobreak >nul

REM Check if services are running
echo Checking service status...
docker-compose ps

REM Run database migrations if needed
echo Running database migrations...
docker-compose exec backend python -m alembic upgrade head

echo.
echo ===========================================
echo Setup completed successfully!
echo ===========================================
echo.
echo Services are now running:
echo - Frontend (Docusaurus): http://localhost:3000
echo - Backend API: http://localhost:8000
echo - PostgreSQL: localhost:5432
echo - Qdrant: localhost:6333
echo.
echo To view logs: docker-compose logs -f
echo To stop services: docker-compose down
echo To restart services: docker-compose restart
echo.
echo The Physical AI & Humanoid Robotics Book system is now ready to use!
echo ===========================================

pause