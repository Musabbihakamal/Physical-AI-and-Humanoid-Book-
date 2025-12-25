"""
Main entry point for the book generation system backend.
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
    # On Heroku, avoid file logging due to ephemeral filesystem
    import os
    if os.getenv("DYNO"):  # DYNO is set on Heroku
        setup_logging(
            log_level=settings.LOG_LEVEL if hasattr(settings, 'LOG_LEVEL') else 'INFO',
            log_file=None  # Don't write to file on Heroku, use console only
        )
    else:
        setup_logging(
            log_level=settings.LOG_LEVEL if hasattr(settings, 'LOG_LEVEL') else 'INFO',
            log_file="app.log"
        )

    logger = logging.getLogger(__name__)
    logger.info("Starting Book + RAG Bot backend...")

    # Validate required API keys at startup
    validate_api_keys(logger)

    # Run the application
    import os
    reload_option = os.getenv("RELOAD", "False").lower() == "true"
    port = int(os.getenv("PORT", 8000))  # Use PORT from environment, default to 8000
    uvicorn.run(
        "backend.src.api.main:app",
        host="0.0.0.0",
        port=port,
        reload=reload_option,
        log_level="info"
    )


def validate_api_keys(logger):
    """
    Validate that required API keys are configured at startup.
    """
    # Check for translation API keys
    has_openai_key = bool(settings.OPENAI_API_KEY)
    has_claude_key = bool(settings.CLAUDE_API_KEY)

    if not has_openai_key and not has_claude_key:
        logger.warning("Warning: No translation API keys configured. Translation functionality will be limited.")
        logger.warning("Please set either OPENAI_API_KEY or CLAUDE_API_KEY in your environment variables.")
    else:
        if has_openai_key:
            logger.info("OpenAI API key is configured")
        if has_claude_key:
            logger.info("Claude API key is configured")

    # Log which keys are available (without exposing the actual keys)
    available_keys = []
    if has_openai_key:
        available_keys.append("OPENAI_API_KEY")
    if has_claude_key:
        available_keys.append("CLAUDE_API_KEY")

    if available_keys:
        logger.info(f"Available API keys: {', '.join(available_keys)}")
    else:
        logger.warning("No API keys are configured - translation services will not work")


if __name__ == "__main__":
    main()