"""
Error handling utilities for the multi-agent book generation system.
"""
import logging
from typing import Any, Dict
from fastapi import HTTPException, status
from fastapi.responses import JSONResponse


class AgentError(Exception):
    """Base exception for agent-related errors."""
    def __init__(self, message: str, error_code: str = "AGENT_ERROR"):
        self.message = message
        self.error_code = error_code
        super().__init__(self.message)


class ValidationError(Exception):
    """Exception for validation errors."""
    def __init__(self, message: str, field: str = None):
        self.message = message
        self.field = field
        super().__init__(self.message)


class DatabaseError(Exception):
    """Exception for database-related errors."""
    def __init__(self, message: str, operation: str = None):
        self.message = message
        self.operation = operation
        super().__init__(self.message)


async def handle_exception(request: Any, exc: Exception) -> JSONResponse:
    """
    Global exception handler for the application.

    Args:
        request: The request object
        exc: The exception that occurred

    Returns:
        JSONResponse with error details
    """
    # Use a generic logger - actual configuration should be handled at app startup
    logger = logging.getLogger(__name__)

    if isinstance(exc, ValidationError):
        logger.warning(f"Validation error: {exc.message}")
        return JSONResponse(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            content={
                "error": "VALIDATION_ERROR",
                "message": exc.message,
                "field": exc.field
            }
        )
    elif isinstance(exc, DatabaseError):
        logger.error(f"Database error: {exc.message}")
        return JSONResponse(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            content={
                "error": "DATABASE_ERROR",
                "message": "A database error occurred",
                "details": exc.message
            }
        )
    elif isinstance(exc, AgentError):
        logger.error(f"Agent error: {exc.message}")
        return JSONResponse(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            content={
                "error": exc.error_code,
                "message": exc.message
            }
        )
    elif isinstance(exc, HTTPException):
        logger.warning(f"HTTP error: {exc.detail}")
        return JSONResponse(
            status_code=exc.status_code,
            content={
                "error": "HTTP_ERROR",
                "message": str(exc.detail)
            }
        )
    else:
        logger.error(f"Unexpected error: {str(exc)}", exc_info=True)
        return JSONResponse(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            content={
                "error": "INTERNAL_ERROR",
                "message": "An unexpected error occurred"
            }
        )