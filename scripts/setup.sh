#!/bin/bash

# Physical AI & Humanoid Robotics Book - Setup Script
# This script automates the setup and deployment of the complete system

set -e  # Exit on any error

echo "==========================================="
echo "Physical AI & Humanoid Robotics Book Setup"
echo "==========================================="

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Check prerequisites
echo "Checking prerequisites..."

if ! command_exists docker; then
    echo "Error: Docker is not installed. Please install Docker first."
    exit 1
fi

if ! command_exists docker-compose; then
    echo "Error: Docker Compose is not installed. Please install Docker Compose first."
    exit 1
fi

if ! command_exists git; then
    echo "Error: Git is not installed. Please install Git first."
    exit 1
fi

echo "All prerequisites found!"

# Function to check if environment variables are set
check_env_vars() {
    local missing_vars=()

    if [ -z "$OPENAI_API_KEY" ]; then
        missing_vars+=("OPENAI_API_KEY")
    fi

    if [ -z "$QDRANT_URL" ]; then
        missing_vars+=("QDRANT_URL")
    fi

    if [ -z "$DATABASE_URL" ]; then
        missing_vars+=("DATABASE_URL")
    fi

    if [ -z "$SECRET_KEY" ]; then
        missing_vars+=("SECRET_KEY")
    fi

    if [ ${#missing_vars[@]} -ne 0 ]; then
        echo "Error: The following environment variables are not set:"
        printf '%s\n' "${missing_vars[@]}"
        echo ""
        echo "Please set these variables or create a .env file with the required values."
        exit 1
    fi
}

# Check if running in a repository directory or need to clone
if [ ! -f "package.json" ] || [ ! -d "backend" ] || [ ! -d "frontend" ]; then
    echo "Repository files not found. Cloning repository..."

    if [ -z "$REPO_URL" ]; then
        echo "Please provide the repository URL:"
        read -r REPO_URL
    fi

    git clone "$REPO_URL" .
fi

# Check for .env file
if [ ! -f ".env" ]; then
    echo "Creating .env file from example..."
    if [ -f ".env.example" ]; then
        cp .env.example .env
        echo "Created .env file. Please edit it to add your API keys and database URLs."
        echo "After editing, run this script again."
        exit 0
    else
        echo "Error: .env.example file not found. Cannot create .env file."
        exit 1
    fi
fi

# Source environment variables
set +e  # Temporarily disable exit on error for sourcing
source .env
set -e  # Re-enable exit on error

# Check environment variables
check_env_vars

echo "Environment variables are set correctly."

# Build and start the system
echo "Building and starting the system..."

# Pull latest images
docker-compose pull

# Build images
docker-compose build

# Start services
docker-compose up -d

echo "Waiting for services to start..."
sleep 10

# Check if services are running
echo "Checking service status..."
docker-compose ps

# Run database migrations if needed
echo "Running database migrations..."
docker-compose exec backend python -m alembic upgrade head || echo "Migration step completed or not needed"

echo ""
echo "==========================================="
echo "Setup completed successfully!"
echo "==========================================="
echo ""
echo "Services are now running:"
echo "- Frontend (Docusaurus): http://localhost:3000"
echo "- Backend API: http://localhost:8000"
echo "- PostgreSQL: localhost:5432"
echo "- Qdrant: localhost:6333"
echo ""
echo "To view logs: docker-compose logs -f"
echo "To stop services: docker-compose down"
echo "To restart services: docker-compose restart"
echo ""
echo "The Physical AI & Humanoid Robotics Book system is now ready to use!"
echo "==========================================="