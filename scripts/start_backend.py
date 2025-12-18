#!/usr/bin/env python3
"""
Script to start the backend server with proper Python path setup
"""
import sys
import os
from pathlib import Path

# Add the project root directory to the Python path
project_root = Path(__file__).parent
sys.path.insert(0, str(project_root))

# Now start the uvicorn server
if __name__ == "__main__":
    import uvicorn
    uvicorn.run("backend.src.api.main:app", host="127.0.0.1", port=8000, reload=True)