"""
Main entry point for the multi-agent book generation system backend.
"""
import sys
import os
# Add the project root directory to the Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

import uvicorn
from .api.main import app
from .utils.logging import setup_logging
from .config import settings
import logging


def main():
    """
    Main function to run the FastAPI application.
    """
    # Set up logging
    setup_logging(
        log_level=settings.LOG_LEVEL if hasattr(settings, 'LOG_LEVEL') else 'INFO',
        log_file="app.log"
    )

    logger = logging.getLogger(__name__)
    logger.info("Starting Book + RAG Bot + Multi-Agent System backend...")

    # Run the application
    uvicorn.run(
        "backend.src.api.main:app",
        host="0.0.0.0",
        port=8000,
        reload=True,  # Set to False in production
        log_level="info"
    )


if __name__ == "__main__":
    main()