"""
Rate limiting configuration and utilities for the RAG API
"""
from slowapi import Limiter, _rate_limit_exceeded_handler
from slowapi.util import get_remote_address
from slowapi.errors import RateLimitExceeded
from fastapi import Request, HTTPException
from fastapi.responses import JSONResponse
import os
from typing import Optional

# Optional Redis import - falls back to in-memory if not available
try:
    import redis
    REDIS_AVAILABLE = True
except ImportError:
    redis = None
    REDIS_AVAILABLE = False

# Initialize Redis connection for rate limiting storage
def get_redis_client() -> Optional[object]:
    """
    Get Redis client for rate limiting storage.
    Falls back to in-memory storage if Redis is not available.
    """
    if not REDIS_AVAILABLE:
        return None

    redis_url = os.getenv("REDIS_URL")
    if redis_url:
        try:
            return redis.from_url(redis_url, decode_responses=True)
        except Exception:
            return None
    return None

# Rate limiting key function that considers authentication
def get_rate_limit_key(request: Request) -> str:
    """
    Generate rate limiting key based on user authentication status.
    Authenticated users get per-user limits, anonymous users get per-IP limits.
    """
    # Try to get user from request state (set by auth middleware)
    user = getattr(request.state, 'user', None)
    if user and hasattr(user, 'id'):
        return f"user:{user.id}"

    # Fall back to IP-based limiting for anonymous users
    return f"ip:{get_remote_address(request)}"

# Initialize limiter with proper storage
redis_client = get_redis_client()
if redis_client:
    # Use Redis for distributed rate limiting
    limiter = Limiter(
        key_func=get_rate_limit_key,
        storage_uri=os.getenv("REDIS_URL", "redis://localhost:6379")
    )
else:
    # Use in-memory storage for development (default)
    limiter = Limiter(key_func=get_rate_limit_key)

# Custom rate limit exceeded handler
async def custom_rate_limit_handler(request: Request, exc: RateLimitExceeded):
    """
    Custom handler for rate limit exceeded errors.
    Returns structured JSON response with helpful information.
    """
    response = JSONResponse(
        status_code=429,
        content={
            "detail": {
                "message": "Rate limit exceeded. Please try again later.",
                "error_code": "rate_limit_exceeded"
            },
            "retry_after": exc.retry_after,
            "limit": str(exc.detail).split()[0] if exc.detail else "unknown"
        }
    )
    response.headers["Retry-After"] = str(exc.retry_after)
    return response

# Rate limiting decorators for different user types
class RateLimits:
    """Rate limiting configurations for different endpoints and user types"""

    # RAG Query limits - most restrictive due to AI API costs
    RAG_QUERY_ANONYMOUS = "3/minute;15/hour"      # Anonymous users: 3/min, 15/hour
    RAG_QUERY_AUTHENTICATED = "10/minute;60/hour"  # Auth users: 10/min, 60/hour

    # Chat history limits - less restrictive
    CHAT_HISTORY = "30/minute;200/hour"

    # Health check limits - very permissive
    HEALTH_CHECK = "60/minute"

    # Global service protection
    GLOBAL_LIMIT = "1000/hour"

def get_user_rate_limit(request: Request, anonymous_limit: str, auth_limit: str) -> str:
    """
    Return appropriate rate limit based on user authentication status.
    """
    user = getattr(request.state, 'user', None)
    if user and hasattr(user, 'id'):
        return auth_limit
    return anonymous_limit

# Middleware to set user in request state for rate limiting
async def set_user_for_rate_limiting(request: Request, call_next):
    """
    Middleware to extract user information for rate limiting.
    This should be called after authentication middleware.
    """
    response = await call_next(request)
    return response