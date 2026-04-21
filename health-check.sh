#!/bin/bash
# Health check script for Book RAG System

echo "========================================"
echo "Book RAG System - Health Check"
echo "========================================"
echo ""

# Check backend venv
echo "[1/5] Checking backend virtual environment..."
if [ -f "backend/.venv/bin/activate" ]; then
    echo "✓ Virtual environment found"
else
    echo "✗ Virtual environment NOT found - Run: npm run setup"
    exit 1
fi

# Check backend .env
echo "[2/5] Checking backend configuration..."
if [ -f "backend/.env" ]; then
    echo "✓ backend/.env found"
    if grep -q "ANTHROPIC_API_KEY" backend/.env; then
        if grep -q "sk-ant" backend/.env; then
            echo "✓ ANTHROPIC_API_KEY configured"
        else
            echo "⚠ ANTHROPIC_API_KEY not set (translation may not work)"
        fi
    fi
else
    echo "✗ backend/.env NOT found"
    exit 1
fi

# Check frontend .env
echo "[3/5] Checking frontend configuration..."
if [ -f "frontend/.env" ]; then
    echo "✓ frontend/.env found"
    if grep -q "REACT_APP_BACKEND_URL" frontend/.env; then
        echo "✓ REACT_APP_BACKEND_URL configured"
    else
        echo "⚠ REACT_APP_BACKEND_URL not set"
    fi
else
    echo "✗ frontend/.env NOT found"
    exit 1
fi

# Check if ports are available
echo "[4/5] Checking if ports are available..."
if lsof -Pi :8000 -sTCP:LISTEN -t >/dev/null 2>&1; then
    echo "⚠ Port 8000 is already in use"
else
    echo "✓ Port 8000 is available"
fi

if lsof -Pi :3000 -sTCP:LISTEN -t >/dev/null 2>&1; then
    echo "⚠ Port 3000 is already in use"
else
    echo "✓ Port 3000 is available"
fi

# Check dependencies
echo "[5/5] Checking dependencies..."
if [ -d "backend/.venv/lib/python"* ]; then
    echo "✓ Python packages installed"
else
    echo "✗ Python packages not found - Run: npm run setup"
fi

if [ -d "frontend/node_modules/react" ]; then
    echo "✓ React installed"
else
    echo "✗ React not found - Run: npm run setup"
fi

echo ""
echo "========================================"
echo "Health check complete!"
echo "========================================"
echo ""
echo "Next steps:"
echo "1. Run: ./start-dev.sh"
echo "2. Wait 10-15 seconds for services to start"
echo "3. Open: http://localhost:3000"
echo ""
