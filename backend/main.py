"""
Main entry point for Railway deployment.
Imports the FastAPI app from src.api.main
"""
from src.api.main import app

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8002)  # Use port 8002 to avoid conflicts