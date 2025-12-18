"""
Rate limiting utilities for the multi-agent book generation system.
"""
import time
from typing import Dict, Optional
from threading import Lock
import logging


logger = logging.getLogger(__name__)


class RateLimiter:
    """
    Rate limiter to prevent abuse of agent endpoints.
    Implements a token bucket algorithm for rate limiting.
    """

    def __init__(self):
        self._buckets: Dict[str, Dict[str, float]] = {}  # {user_id: {endpoint: last_access_time}}
        self._lock = Lock()

        # Rate limits configuration
        self._limits = {
            'GLOSSARY_MAKER': {'requests': 10, 'window': 60},  # 10 requests per minute
            'CODE_EXPLAINER': {'requests': 10, 'window': 60},  # 10 requests per minute
            'QUIZ_CREATOR': {'requests': 5, 'window': 60},     # 5 requests per minute
            'CHAPTER_GENERATOR': {'requests': 3, 'window': 60}, # 3 requests per minute
            'default': {'requests': 10, 'window': 60}          # 10 requests per minute default
        }

    def is_allowed(self, user_id: str, endpoint: str, agent_type: str) -> bool:
        """
        Check if a request is allowed based on rate limits.

        Args:
            user_id: User ID making the request
            endpoint: API endpoint being called
            agent_type: Type of agent being used

        Returns:
            True if request is allowed, False otherwise
        """
        with self._lock:
            # Get rate limit configuration
            limit_config = self._limits.get(agent_type, self._limits['default'])
            max_requests = limit_config['requests']
            window = limit_config['window']

            # Create key for this user and endpoint
            key = f"{user_id}:{endpoint}"

            # Initialize bucket if it doesn't exist
            if key not in self._buckets:
                self._buckets[key] = []

            # Get current time
            current_time = time.time()

            # Remove requests older than the window
            self._buckets[key] = [
                req_time for req_time in self._buckets[key]
                if current_time - req_time < window
            ]

            # Check if we're under the limit
            if len(self._buckets[key]) < max_requests:
                # Add current request
                self._buckets[key].append(current_time)
                return True
            else:
                logger.warning(f"Rate limit exceeded for user {user_id}, endpoint {endpoint}, agent {agent_type}")
                return False

    def get_reset_time(self, user_id: str, endpoint: str, agent_type: str) -> Optional[float]:
        """
        Get the time when the rate limit will reset for this user/endpoint.

        Args:
            user_id: User ID making the request
            endpoint: API endpoint being called
            agent_type: Type of agent being used

        Returns:
            Time when rate limit will reset, or None if not limited
        """
        with self._lock:
            key = f"{user_id}:{endpoint}"

            if key not in self._buckets:
                return None

            limit_config = self._limits.get(agent_type, self._limits['default'])
            max_requests = limit_config['requests']
            window = limit_config['window']

            # Remove old requests
            current_time = time.time()
            self._buckets[key] = [
                req_time for req_time in self._buckets[key]
                if current_time - req_time < window
            ]

            if len(self._buckets[key]) >= max_requests:
                # Calculate reset time (the oldest request + window)
                oldest_request = min(self._buckets[key])
                return oldest_request + window
            else:
                return None


# Global rate limiter instance
rate_limiter = RateLimiter()


def check_rate_limit(user_id: str, endpoint: str, agent_type: str) -> Dict[str, any]:
    """
    Check rate limit for a request and return status.

    Args:
        user_id: User ID making the request
        endpoint: API endpoint being called
        agent_type: Type of agent being used

    Returns:
        Dictionary with rate limit status
    """
    allowed = rate_limiter.is_allowed(user_id, endpoint, agent_type)
    reset_time = rate_limiter.get_reset_time(user_id, endpoint, agent_type)

    return {
        'allowed': allowed,
        'reset_time': reset_time,
        'retry_after': reset_time - time.time() if reset_time else 0
    }