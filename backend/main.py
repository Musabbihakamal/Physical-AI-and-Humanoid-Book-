"""
Main entry point for Railway deployment.
Imports the FastAPI app from src.api.main
"""
import sys
import os

# Add the current directory to Python path for Railway deployment
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Import the FastAPI app
from src.api.main import app

# Expose app at module level for ASGI servers
__all__ = ["app"]

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=int(os.environ.get("PORT", 8002)))