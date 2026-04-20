#!/bin/bash
# Test runner script for translation and RAG bot integration

echo "=========================================="
echo "Translation & RAG Bot Integration Tests"
echo "=========================================="
echo ""

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if backend is running
echo "Checking backend connectivity..."
if curl -s http://localhost:8001/health > /dev/null; then
    echo -e "${GREEN}✓ Backend is running${NC}"
else
    echo -e "${RED}✗ Backend is not running${NC}"
    echo "Start backend with: cd backend && python main.py"
    exit 1
fi

echo ""
echo "Running backend integration tests..."
cd backend

# Run pytest with coverage
python -m pytest tests/test_integration.py -v --tb=short --cov=src --cov-report=term-missing

TEST_RESULT=$?

echo ""
echo "=========================================="
if [ $TEST_RESULT -eq 0 ]; then
    echo -e "${GREEN}✓ All tests passed!${NC}"
else
    echo -e "${RED}✗ Some tests failed${NC}"
fi
echo "=========================================="

exit $TEST_RESULT
