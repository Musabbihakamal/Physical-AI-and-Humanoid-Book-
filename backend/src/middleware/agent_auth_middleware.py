"""
Authentication and rate limiting middleware for agent endpoints.
"""
from fastapi import Request, HTTPException, status
from sqlalchemy.orm import Session
from typing import Dict, Any
import time
import logging
from ..models.user import User
from ..models.agent_request import AgentRequest
from ..database.database import get_db

logger = logging.getLogger(__name__)


class AgentAuthMiddleware:
    """
    Middleware to handle agent authentication, rate limiting, and usage tracking.
    """

    def __init__(self):
        # In-memory rate limiting storage (in production, use Redis or similar)
        self.request_counts = {}
        self.rate_limit_window = 3600  # 1 hour window
        self.max_requests_per_hour = 100  # Max 100 requests per hour per user

    def check_rate_limit(self, user_id: str) -> bool:
        """
        Check if user has exceeded rate limit for agent requests.
        """
        current_time = int(time.time())
        window_start = current_time - self.rate_limit_window

        # Clean up old entries
        self.request_counts = {
            key: count for key, (timestamp, count)
            in self.request_counts.items()
            if timestamp > window_start
        }

        # Check if user has exceeded rate limit
        user_key = f"user:{user_id}"
        if user_key in self.request_counts:
            timestamp, count = self.request_counts[user_key]
            if count >= self.max_requests_per_hour and timestamp > window_start:
                return False
        return True

    def record_request(self, user_id: str):
        """
        Record a successful agent request for rate limiting.
        """
        current_time = int(time.time())
        user_key = f"user:{user_id}"

        if user_key in self.request_counts:
            timestamp, count = self.request_counts[user_key]
            if current_time - timestamp < self.rate_limit_window:
                self.request_counts[user_key] = (timestamp, count + 1)
            else:
                self.request_counts[user_key] = (current_time, 1)
        else:
            self.request_counts[user_key] = (current_time, 1)

    async def __call__(self, request: Request, call_next):
        """
        Middleware entry point.
        """
        # For agent endpoints, we expect authentication to be handled by dependencies
        # This middleware focuses on rate limiting and usage tracking
        response = await call_next(request)

        # Log successful agent requests for analytics
        if (request.url.path.startswith("/api/agents/") and
            response.status_code == 200 and
            hasattr(request.state, 'current_user')):

            user_id = request.state.current_user.id
            agent_type = self._extract_agent_type(request)

            if agent_type:
                self.record_request(user_id)
                logger.info(f"Agent request recorded for user {user_id}, type: {agent_type}")

        return response

    def _extract_agent_type(self, request: Request) -> str:
        """
        Extract agent type from request path.
        """
        path = request.url.path
        if "/code-explainer" in path:
            return "CODE_EXPLAINER"
        return "UNKNOWN"


# Rate limiting dependency for agent endpoints
def require_agent_access(user: User = None) -> bool:
    """
    Dependency to check if user has access to agent endpoints.
    """
    if not user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Authentication required"
        )

    # Check if user is active
    if not user.is_active:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Inactive user account"
        )

    # Check if user has exceeded rate limits
    middleware = AgentAuthMiddleware()
    if not middleware.check_rate_limit(str(user.id)):
        raise HTTPException(
            status_code=status.HTTP_429_TOO_MANY_REQUESTS,
            detail="Rate limit exceeded. Please try again later."
        )

    return True