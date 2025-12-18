#!/bin/bash
# Project Initialization Script
# This script sets up the complete project environment

set -e  # Exit on any error

echo "🚀 Initializing Book + RAG Bot + Multi-Agent System..."

# Check if Python 3.11+ is available
if ! command -v python3 &> /dev/null; then
    echo "❌ Python3 is not installed. Please install Python 3.11+"
    exit 1
fi

PYTHON_VERSION=$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')
if [[ $(echo "$PYTHON_VERSION 3.11" | tr " " "\n" | sort -V | head -n1) != "3.11" ]]; then
    echo "❌ Python 3.11+ is required. Current version: $PYTHON_VERSION"
    exit 1
fi

# Check if Node.js 18+ is available
if ! command -v node &> /dev/null; then
    echo "❌ Node.js is not installed. Please install Node.js 18+"
    exit 1
fi

NODE_VERSION=$(node -v | sed 's/v//')
if [[ $(echo "$NODE_VERSION 18" | tr " " "\n" | sort -V | head -n1) != "18" ]]; then
    echo "❌ Node.js 18+ is required. Current version: $NODE_VERSION"
    exit 1
fi

echo "✅ Prerequisites check passed"

# Install backend dependencies
echo "📦 Installing backend dependencies..."
cd backend
if [ -f "requirements.txt" ]; then
    python3 -m venv venv
    source venv/bin/activate  # On Windows use: venv\Scripts\activate
    pip install --upgrade pip
    pip install -r requirements.txt
    echo "✅ Backend dependencies installed"
else
    echo "⚠️  requirements.txt not found in backend"
fi
cd ..

# Install frontend dependencies
echo "📦 Installing frontend dependencies..."
cd frontend
if [ -f "package.json" ]; then
    npm install
    echo "✅ Frontend dependencies installed"
else
    echo "⚠️  package.json not found in frontend"
fi
cd ..

# Create example environment files if they don't exist
echo "📝 Creating environment configuration..."
cd backend
if [ ! -f ".env" ]; then
    cp .env.example .env
    echo "✅ Created backend .env file"
fi
cd ../frontend
if [ ! -f ".env" ]; then
    cp .env.example .env
    echo "✅ Created frontend .env file"
fi
cd ..

# Initialize database if needed
echo "💾 Initializing database..."
cd backend
if [ -f "src/database/database.py" ]; then
    source venv/bin/activate  # On Windows use: venv\Scripts\activate
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
fi
cd ..

echo "🎉 Project initialization complete!"
echo ""
echo "📋 Next steps:"
echo "1. Update environment variables in backend/.env and frontend/.env"
echo "2. Start backend: cd backend && python -m src.main (or use start_backend.py)"
echo "3. Start frontend: cd frontend && npm start"
echo "4. Access the application at http://localhost:3000"