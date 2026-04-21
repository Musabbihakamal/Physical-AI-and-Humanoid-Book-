#!/bin/bash
# Start development environment for Book RAG System

echo "Starting Book RAG System..."
echo ""

# Check if backend venv exists
if [ ! -f "backend/.venv/bin/activate" ]; then
    echo "Error: Virtual environment not found. Run 'npm run setup' first."
    exit 1
fi

# Start backend and frontend concurrently
echo "Starting backend on port 8001..."
echo "Starting frontend on port 3000..."
echo ""

# Start backend in background
(cd backend && source .venv/bin/activate && PORT=8001 python -m src.main) &
BACKEND_PID=$!

# Start frontend in background
(cd frontend && npm start) &
FRONTEND_PID=$!

echo ""
echo "Services starting..."
echo "Backend: http://localhost:8001"
echo "Frontend: http://localhost:3000"
echo ""
echo "Press Ctrl+C to stop all services"
echo ""

# Wait for both processes
wait $BACKEND_PID $FRONTEND_PID
