"""
Simple in-memory cache utility for improving performance of agent responses.
"""
import time
from typing import Any, Dict, Optional, Tuple
from threading import Lock


class SimpleCache:
    """
    A simple in-memory cache with TTL (Time To Live) functionality.
    """
    def __init__(self):
        self._cache: Dict[str, Tuple[Any, float]] = {}  # {key: (value, expiry_time)}
        self._lock = Lock()

    def get(self, key: str) -> Optional[Any]:
        """
        Get a value from the cache.

        Args:
            key: Cache key

        Returns:
            Cached value if found and not expired, None otherwise
        """
        with self._lock:
            if key in self._cache:
                value, expiry_time = self._cache[key]
                if time.time() < expiry_time:
                    return value
                else:
                    # Remove expired entry
                    del self._cache[key]
            return None

    def set(self, key: str, value: Any, ttl: int = 300) -> None:
        """
        Set a value in the cache with TTL.

        Args:
            key: Cache key
            value: Value to cache
            ttl: Time to live in seconds (default 300 seconds = 5 minutes)
        """
        with self._lock:
            expiry_time = time.time() + ttl
            self._cache[key] = (value, expiry_time)

    def delete(self, key: str) -> bool:
        """
        Delete a key from the cache.

        Args:
            key: Cache key to delete

        Returns:
            True if key existed and was deleted, False otherwise
        """
        with self._lock:
            if key in self._cache:
                del self._cache[key]
                return True
            return False

    def clear(self) -> None:
        """
        Clear all entries from the cache.
        """
        with self._lock:
            self._cache.clear()


# Global cache instance
agent_cache = SimpleCache()


def get_cache_key(agent_type: str, content_hash: str, user_id: Optional[str] = None) -> str:
    """
    Generate a cache key for agent responses.

    Args:
        agent_type: Type of agent (e.g., 'glossary_maker', 'code_explainer')
        content_hash: Hash of the input content
        user_id: Optional user ID for personalized caching

    Returns:
        Generated cache key
    """
    if user_id:
        return f"agent:{agent_type}:{content_hash}:{user_id}"
    else:
        return f"agent:{agent_type}:{content_hash}"